/*
 * FireworksEffects - Fireworks effects for Fourth of July
 * 
 * Modifications Copyright (c) 2019 Martin F. Falatic
 *
 * Based on code licensed "Free for all use" 22 Jun 2019 by Davepl www.reddit.com/u/davepl
 * https://www.reddit.com/r/arduino/comments/c3sd46/i_made_this_fireworks_effect_for_my_led_strips/
 *
 */

/* 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#elif defined(ARDUINO) // pre-1.0
  // No extras
#elif defined(ESP_PLATFORM)
  #include "../../main/arduinoish.hpp"
#endif

#include "esp32_digital_led_lib.h"
#include "fireworks_effects.h"
#include <deque>


#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))


FireworksEffects::GraphicsStub::GraphicsStub(strand_t * pStrand) {
  StrandPtr = pStrand;
  DotCount = StrandPtr->numPixels;
}


void FireworksEffects::GraphicsStub::FillSolid(uint32_t cval) {
  strand_t * strands [] = { StrandPtr };
  digitalLeds_resetPixels(strands, 1);
}


void FireworksEffects::GraphicsStub::SetPixels(double pos, int width, pixelColor_t c) {
  for (int i = -width/2; i <= width/2; i++) {
    if (pos+i >= 0 && pos+i < DotCount) {
      StrandPtr->pixels[int(pos+i)] = c;
    }
  }
}


void FireworksEffects::GraphicsStub::RefreshPixels() {
  strand_t * strands [] = { StrandPtr };
  digitalLeds_drawPixels(strands, 1);
}


// Each particle in the particle system remembers its color, birth time, postion, velocity, etc.
FireworksEffects::Particle::Particle(pixelColor_t * starColor, double pos, double maxSpeed)
{
  _position = pos;
  _velocity = randDouble() * maxSpeed * 2 - maxSpeed;
  _starColor.raw32 = starColor->raw32;
  _birthTime = millis();
  _lastUpdate = _birthTime;
}


double FireworksEffects::Particle::Age()
{
  return (millis() - _birthTime) / 1000.0;
}


void FireworksEffects::Particle::Update()
{
  // As the particle ages we actively fade its color and slow its speed
  double deltaTime = (millis() - _lastUpdate) / 1000.0;
  _position += _velocity * deltaTime;
  _lastUpdate = millis() * 1.0;
  _velocity -= (2 * _velocity * deltaTime);
  _starColor = adjustByUniformFactor(&_starColor, randDouble() * 0.1);
}


FireworksEffects::FireworksEffects(strand_t * pStrand)
{
  graphics = new GraphicsStub(pStrand);
}


void FireworksEffects::Render()
{
    // Randomly create some new stars this frame; the number we create is tied 
    // to the size of the display so that the display size can change and 
    // the "effect density" will stay the same

    const int min_width = 10;
    const int max_width = 50;
    const int max_speed_multiplier = 3.0;
    
    for (int iPass = 0; iPass < graphics->DotCount / max_width; iPass++)
    {
        if (randDouble() < NewParticleProbability && _Particles.size() == 0)
        {
            // Pick a random color and location.  
            // If you don't have FastLED palettes, all you need to do
            // here is generate a random color.

            int iStartPos = (int)(randDouble() * graphics->DotCount);
            pixelColor_t color;
            color.raw32 = colorChoices[int(randDouble() * COUNT_OF(colorChoices))];
            int c = int(randDouble() * (max_width - min_width) + min_width);
            double speed_multiplier = randDouble() * MaxSpeed * max_speed_multiplier;

            for (int i = 1; i < c; i++)
            {
                Particle particle(&color, iStartPos, randDouble() * speed_multiplier);
                _Particles.push_back(particle);
            }
        }
    }

    // In the degenerate case of particles not aging out for some reason, 
    // we need to set a pseudo-realistic upper bound, and the very number of     
    // possible pixels seems like a reasonable one

    while (_Particles.size() > graphics->DotCount) {
      _Particles.pop_front();
    }
    graphics->FillSolid(0x00000000); // This is hacky

    for (int i = 0; i < _Particles.size(); i++)
    {
        Particle * star = &(_Particles[i]);
        star->Update();

        pixelColor_t c;
        c.raw32 = (star->_starColor).raw32;
        double fade = 0.0;

        // If the star is brand new, it flashes white briefly.  
        // Otherwise it just fades over time.
        if (star->Age() > ParticlePreignitonTime && star->Age() < (ParticleIgnition + ParticlePreignitonTime))
        {
            c = pixelFromRGBW(0x3F, 0x3F, 0x3F, 0x3F);
        }
        else
        {
            // Figure out how much to fade and shrink the star based on 
            // its age relative to its lifetime
            double age = star->Age();
            if (age < ParticlePreignitonTime) {
                fade = 1.0 - (age / ParticlePreignitonTime);
            }
            else
            {
                age -= ParticlePreignitonTime;

                if (age < ParticleHoldTime + ParticleIgnition)
                    fade = 0.0;                  // Just born
                else if (age > ParticleHoldTime + ParticleIgnition + ParticleFadeTime)
                    fade = 1.0;                  // Black hole, all faded out
                else
                {
                    age -= (ParticleHoldTime + ParticleIgnition);
                    fade = (age / ParticleFadeTime);  // Fading star
                }
            }
            if (c.raw32 > 0) {
              c = adjustByUniformFactor(&c, fade);
            }
        }
        ParticleSize = (1 - fade) * 5.0;

        // Because (the original) supports antialiasing and partial pixels, this takes a
        // non-integer number of pixels to draw.  But if you just made it
        // plot 'ParticleSize' pixels in int form, you'd be 99% of the way there

        graphics->SetPixels(star->_position, (int)ParticleSize, c);
    }

    // Remove any particles who have completed their lifespan
    while (!_Particles.empty() && _Particles.front().Age() > ParticleHoldTime + ParticleIgnition + ParticleFadeTime)
    {
        _Particles.pop_front();
    }

    graphics->RefreshPixels();
}


//**************************************************************************//
