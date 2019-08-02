/*
 * FireworksEffect Fireworks Effect for Fourth of July
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
 
#ifndef FIREWORKS_EFFECTS_H
#define FIREWORKS_EFFECTS_H

#include <deque>

class FireworksEffects {

  public:
    FireworksEffects(strand_t * pStrand);

  //protected:
    class Particle {
      public:
        pixelColor_t _starColor;
        unsigned long _birthTime;
        unsigned long _lastUpdate;
        double _velocity;
        double _position;

        Particle(pixelColor_t * starColor, double pos, double maxSpeed);
        double Age();
        void Update();
    };

    class GraphicsStub {
      public:
        strand_t * StrandPtr;
        int DotCount;

        GraphicsStub(strand_t * pStrand);
        void FillSolid(uint32_t cval);
        void SetPixels(double pos, int width, pixelColor_t c);
        void RefreshPixels();
    };

    uint32_t colorChoices [24] = {  // RGB with 0x00 W values
        0xFFFF00, 0xFF7F00, 0xFF0000,
        0x7FFF00, 0x7F7F00, 0x7F0000,
        0x00FF00, 0x007F00, //0x000000,
        0x00FFFF, 0x00FF7F, 0x00FF00,
        0x007FFF, 0x007F7F, 0x007F00,
        0x0000FF, 0x00007F, //0x000000,
        0xFF00FF, 0x7F00FF, 0x0000FF,
        0xFF007F, 0x7F007F, 0x00007F,
        0xFF0000, 0x7F0000, //0x000000,
    };

    GraphicsStub * graphics;
    std::deque <Particle> _Particles;   // FIFO particles
    double MaxSpeed                  = 375.0;  // Max velocity
    double NewParticleProbability    =   0.1;  // Odds of new particle
    double ParticlePreignitonTime    =   0.0;  // How long to "wink"
    double ParticleIgnition          =   0.2;  // How long to "flash"
    double ParticleHoldTime          =   0.0;  // Main lifecycle time
    double ParticleFadeTime          =   2.0;  // Fade out time
    double ParticleSize              =   0.0;  // Size of the particle
    void Render();
};

#endif /* FIREWORKS_EFFECTS_H */


//**************************************************************************//
