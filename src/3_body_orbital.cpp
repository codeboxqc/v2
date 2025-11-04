#include <Arduino.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h>
#include <cmath>
#include <algorithm>

#include "bar.h"
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#define NUM_BODIES 3
#define G 10.0f
#define dt 0.0016f  //0.0008f
#define scale 18.0f
#define maxTrailLength 500 //400
#define transitionSpeed 0.001f
#define baseTransitionSpeed 0.004f
 
 #define SIM_RANGE 5.2f  //6.2
 

 struct Vec2 {
  float x, y;
  Vec2 operator+(const Vec2& v) const { return {x + v.x, y + v.y}; }
  Vec2 operator-(const Vec2& v) const { return {x - v.x, y - v.y}; }
  Vec2 operator*(float s) const { return {x * s, y * s}; }
  Vec2& operator+=(const Vec2& v) { x += v.x; y += v.y; return *this; }
  float mag() const { return sqrt(x * x + y * y); }
  Vec2 normalized() const { float m = mag(); return m > 0 ? (*this) * (1.0f / m) : Vec2{0, 0}; }
};

Vec2 lerpVec(const Vec2& a, const Vec2& b, float t) {
  return a * (1.0f - t) + b * t;
}

struct Body {
  Vec2 pos, vel, acc;
  float mass;
};

 

std::vector<Body> bodies;
Vec2 trails[NUM_BODIES][maxTrailLength];
int trailSizes[NUM_BODIES] = {0};
int currentShape = 0;
int nextShape = 1;
float transition = 3;
std::vector<Vec2> shapePos[30];
std::vector<Vec2> shapeVel[30];

void defineShapes() {
  // Shape 0: Figure-eight inspired
  shapePos[0] = { {-1, 0}, {1, 0}, {0, 0} };
  shapeVel[0] = { {0, 0.5}, {0, -0.5}, {0.5, 0} };
  shapePos[1] = { {-1, -0.577}, {1, -0.577}, {0, 1} };
  shapeVel[1] = { {0, 0.5}, {0, 0.5}, {0, -1} };
  shapePos[2] = { {-1.5, 0}, {0, 0}, {1.5, 0} };
  shapeVel[2] = { {0, 0.4}, {0, 0}, {0, -0.4} };
  shapePos[3] = { {-0.5, -0.289}, {0.5, -0.289}, {0, 0.577} };
  shapeVel[3] = { {0, 0.3}, {0, 0.3}, {0, -0.6} };
  shapePos[4] = { {-1.5, 0}, {1.5, 0}, {0, 0.5} };
  shapeVel[4] = { {0, 0.6}, {0, -0.6}, {0.6, 0} };
  shapePos[5] = { {-1, -0.3}, {0.8, -0.2}, {0, 0.8} };
  shapeVel[5] = { {0, 0.4}, {0, 0.4}, {0, -0.8} };
  shapePos[6] = { {-1.2, 0.2}, {1.2, 0.2}, {0, -0.4} };
  shapeVel[6] = { {0, 0.5}, {0, -0.5}, {0.5, 0} };
  shapePos[7] = { {-0.8, -0.4}, {0.8, -0.4}, {0, 0.8} };
  shapeVel[7] = { {0, 0.5}, {0, 0.5}, {0, -1} };
  shapePos[8] = { {0, -1}, {0, 0}, {0, 1} };
  shapeVel[8] = { {0.4, 0}, {0, 0}, {-0.4, 0} };
  shapePos[9] = { {-1.2, -0.7}, {1.2, -0.7}, {0, 1.4} };
  shapeVel[9] = { {0, 0.6}, {0, 0.6}, {0, -1.2} };
  shapePos[10] = { {-1.1, 0.1}, {1.1, 0.1}, {0, -0.5} };
  shapeVel[10] = { {0.1, 0.5}, {-0.1, -0.5}, {0.4, 0} };
  shapePos[11] = { {-1.3, 0}, {1.3, 0}, {0, 0.3} };
  shapeVel[11] = { {0, 0.6}, {0, -0.6}, {0.5, 0} };
  shapePos[12] = { {-0.9, -0.5}, {0.9, -0.5}, {0, 1.0} };
  shapeVel[12] = { {0.2, 0.4}, {-0.2, 0.4}, {0, -0.8} };
  shapePos[13] = { {-1.0, -0.2}, {1.0, -0.2}, {0, 0.6} };
  shapeVel[13] = { {0, 0.3}, {0, 0.3}, {0, -0.6} };
  shapePos[14] = { {-1.4, 0.2}, {1.4, 0.2}, {0, -0.6} };
  shapeVel[14] = { {0.1, 0.5}, {-0.1, -0.5}, {0.6, 0} };
  shapePos[15] = { {-0.6, -0.3}, {0.6, -0.3}, {0, 0.7} };
  shapeVel[15] = { {0, 0.4}, {0, 0.4}, {0, -0.8} };
  shapePos[16] = { {-1.5, -0.1}, {1.5, -0.1}, {0, 0.4} };
  shapeVel[16] = { {0, 0.5}, {0, -0.5}, {0.5, 0} };
  shapePos[17] = { {-1.0, -0.6}, {1.0, -0.6}, {0, 1.2} };
  shapeVel[17] = { {0.3, 0.5}, {-0.3, 0.5}, {0, -1.0} };
  shapePos[18] = { {-1.2, -0.4}, {1.2, -0.4}, {0, 0.8} };
  shapeVel[18] = { {0, 0.4}, {0, 0.4}, {0, -0.8} };
  shapePos[19] = { {-1.3, 0.3}, {1.3, 0.3}, {0, -0.6} };
  shapeVel[19] = { {0.2, 0.4}, {-0.2, -0.4}, {0.5, 0} };
  shapePos[20] = { {-1.3, -0.8}, {0.7, 0.9}, {0.6, -0.4} };
  shapeVel[20] = { {0.3, 0.6}, {-0.4, -0.3}, {0.2, 0.5} };
  shapePos[21] = { {-1.6, 0.3}, {1.4, -0.5}, {0.2, 1.1} };
  shapeVel[21] = { {0.1, 0.7}, {-0.3, 0.4}, {0.4, -0.9} };
  shapePos[22] = { {-0.7, 1.2}, {1.3, -0.3}, {-0.4, -0.9} };
  shapeVel[22] = { {0.5, -0.2}, {-0.6, 0.5}, {0.3, 0.4} };
  shapePos[23] = { {-1.1, -1.0}, {0.9, 0.8}, {0.3, 0.2} };
  shapeVel[23] = { {0.6, 0.3}, {-0.2, -0.6}, {0.4, 0.5} };
  shapePos[24] = { {-0.7, -0.7}, {0.7, -0.7}, {0, 1.0} };
  shapeVel[24] = { {0.3, 0.5}, {-0.3, 0.5}, {0, -0.7} };
  shapePos[25] = { {-1.0, 0.3}, {0.5, -0.9}, {0.5, 0.6} };
  shapeVel[25] = { {0.4, 0.3}, {-0.2, 0.6}, {-0.3, -0.5} };
  shapePos[26] = { {-0.6, 0}, {0.6, 0}, {0, -0.8} };
  shapeVel[26] = { {0.2, 0.6}, {-0.2, 0.6}, {0, -0.8} };
  shapePos[27] = { {-1.2, 0.5}, {0, -0.8}, {1.2, 0.3} };
  shapeVel[27] = { {0.5, 0.2}, {0, 0.7}, {-0.5, 0.3} };
  shapePos[28] = { {-0.8, 0.6}, {0.8, 0.6}, {0, -0.9} };
  shapeVel[28] = { {0.3, -0.4}, {-0.3, -0.4}, {0, 0.8} };
  shapePos[29] = { {-0.9, 0}, {0.45, 0.78}, {0.45, -0.78} };
  shapeVel[29] = { {0, 0.6}, {-0.52, -0.3}, {0.52, -0.3} };
}




 
void initBodies() {
  
  bodies.clear();
  if (shapePos[0].size() >= NUM_BODIES && shapeVel[0].size() >= NUM_BODIES) {
    for (int i = 0; i < NUM_BODIES; i++) {
      
      bodies.push_back({ shapePos[0][i], shapeVel[0][i], {0, 0}, 1.0f });
    }
  } else {
    
    for (int i = 0; i < NUM_BODIES; i++) {
      bodies.push_back({ {0, 0}, {0, 0}, {0, 0}, 1.0f });
    }
  }
  
}

void updateBodies2() {
  if (transition < 0.1f) {
    float t = transition / 0.1f;
    for (int i = 0; i < NUM_BODIES; i++) {
      bodies[i].pos = lerpVec(shapePos[currentShape][i], shapePos[nextShape][i], t);
      bodies[i].vel = lerpVec(shapeVel[currentShape][i], shapeVel[nextShape][i], t);
      // Clamp positions during transition
      bodies[i].pos.x = constrain(bodies[i].pos.x, -1.6f, 1.6f);
      bodies[i].pos.y = constrain(bodies[i].pos.y, -1.6f, 1.6f);
    }
  } else {
    for (int i = 0; i < NUM_BODIES; i++) {
      bodies[i].acc = {0, 0};
      for (int j = 0; j < NUM_BODIES; j++) {
        if (i != j) {
          Vec2 r = bodies[j].pos - bodies[i].pos;
          float d = r.mag();
          if (d > 0.01f) {
            float f = G / (d * d);
            bodies[i].acc += r.normalized() * f;
          }
        }
      }
    }
    for (int i = 0; i < NUM_BODIES; i++) {
      bodies[i].vel += bodies[i].acc * dt;
      bodies[i].pos += bodies[i].vel * dt;
      // Clamp positions to keep bodies on screen
      bodies[i].pos.x = constrain(bodies[i].pos.x, -1.6f, 1.6f);
      bodies[i].pos.y = constrain(bodies[i].pos.y, -1.6f, 1.6f);
       
    }
  }
}



void updateBodies() {
  if (transition < 0.1f) {
    float t = transition / 0.1f;
    for (int i = 0; i < NUM_BODIES; i++) {
      bodies[i].pos = lerpVec(shapePos[currentShape][i], shapePos[nextShape][i], t);
      bodies[i].vel = lerpVec(shapeVel[currentShape][i], shapeVel[nextShape][i], t);
      // Wrap positions during transition
      while (bodies[i].pos.x < -1.6f) bodies[i].pos.x += SIM_RANGE;
      while (bodies[i].pos.x > 1.6f) bodies[i].pos.x -= SIM_RANGE;
      while (bodies[i].pos.y < -1.6f) bodies[i].pos.y += SIM_RANGE;
      while (bodies[i].pos.y > 1.6f) bodies[i].pos.y -= SIM_RANGE;
    }
  } else {
    for (int i = 0; i < NUM_BODIES; i++) {
      bodies[i].acc = {0, 0};
      for (int j = 0; j < NUM_BODIES; j++) {
        if (i != j) {
          Vec2 r = bodies[j].pos - bodies[i].pos;
          // Wrap relative distances for gravitational calculations
          while (r.x < -1.6f) r.x += SIM_RANGE;
          while (r.x > 1.6f) r.x -= SIM_RANGE;
          while (r.y < -1.6f) r.y += SIM_RANGE;
          while (r.y > 1.6f) r.y -= SIM_RANGE;
          float d = r.mag();
          if (d > 0.01f) {
            float f = G / (d * d);
            bodies[i].acc += r.normalized() * f;
          }
        }
      }
    }
    for (int i = 0; i < NUM_BODIES; i++) {
      bodies[i].vel += bodies[i].acc * dt;
      bodies[i].pos += bodies[i].vel * dt;
      // Wrap positions to keep bodies in simulation range
      while (bodies[i].pos.x < -1.6f) {
        bodies[i].pos.x += SIM_RANGE;
       
      }
      while (bodies[i].pos.x > 1.6f) {
        bodies[i].pos.x -= SIM_RANGE;
       
      }
      while (bodies[i].pos.y < -1.6f) {
        bodies[i].pos.y += SIM_RANGE;
        
      }
      while (bodies[i].pos.y > 1.6f) {
        bodies[i].pos.y -= SIM_RANGE;
         
      }
    }
  }
}

void drawOrbitalTrails() {
  
  float t = millis() * 0.002;

  uint8_t colors[3][3] = {
    {255, 100, 100},
    {100, 255, 100},
    {100, 100, 255}
  };

  bool anyPixelDrawn = false; // Debug flag to check if any pixels are drawn

  for (int i = 0; i < NUM_BODIES; i++) {
    // Update trail buffer
    if (trailSizes[i] < maxTrailLength) {
      trails[i][trailSizes[i]] = bodies[i].pos;
      trailSizes[i]++;
    } else {
      for (int j = 1; j < maxTrailLength; j++) {
        trails[i][j - 1] = trails[i][j];
      }
      trails[i][maxTrailLength - 1] = bodies[i].pos;
    }

    // Draw trail with waveform-modulated position
    for (int j = 0; j < trailSizes[i]; j++) {
      float fx = trails[i][j].x * scale + WIDTH / 2;
      float fy = trails[i][j].y * scale + HEIGHT / 2;

      int x = (int)fx;
      if (x < 0 || x >= WIDTH) continue;

      // Get waveform amplitude at this X
      float normX = fx / WIDTH;
      int sampleIndex = (int)(normX * SAMPLES);
      sampleIndex = constrain(sampleIndex, 0, SAMPLES - 1);
      float amplitude = (float)samples[sampleIndex] / 32768.0;

      // Modulate Y position with waveform, further reduced for 64x64
      float yOffset = amplitude * 2.0f + 0.5f * fastSin(t + normX * 10.0);
      int y = (int)(fy + yOffset);
      if (y < 0 || y >= HEIGHT) continue;

      // Modulate color with waveform
      float glow = 0.8 + 0.2 * fabs(amplitude);
      float shimmer = 0.5 + 0.5 * fastSin(t + normX * 12.0);
      uint8_t r = min(255, (int)(colors[i][0] * glow * shimmer));
      uint8_t g = min(255, (int)(colors[i][1] * glow * shimmer));
      uint8_t b = min(255, (int)(colors[i][2] * glow * shimmer));

      dma_display->drawPixelRGB888(x, y, r, g, b);
      anyPixelDrawn = true;
    }

    // Draw body with waveform pulse
    int bx = (int)(bodies[i].pos.x * scale + WIDTH / 2);
    int by = (int)(bodies[i].pos.y * scale + HEIGHT / 2);
    if (bx >= 0 && bx < WIDTH && by >= 0 && by < HEIGHT) {
      float amp = (float)samples[(int)((float)bx / WIDTH * SAMPLES)] / 32768.0;
      uint8_t pulse = 200 + 55 * fabs(amp);
      dma_display->drawPixelRGB888(bx, by, pulse, pulse, pulse);
      anyPixelDrawn = true;
    }
  }

   
  

  dma_display->flipDMABuffer();
}

void body3(int trans) {


  currentShape=trans;

 dma_display->clearScreen();


  updateBodies();
  drawOrbitalTrails();
  transition += transitionSpeed;
  if (transition >= 1.0f) {
    transition = 0.0f;
    currentShape = (currentShape + 1) % 30;
    nextShape = (currentShape + 1) % 30;
   for (int i = 0; i < NUM_BODIES; i++) trailSizes[i] = 0;
    
  }
}




// ============================================================================
// FUNCTION 2: Hyperdimensional Orbital Dance with Audio Particles
// ============================================================================
static uint8_t fb[HEIGHT][WIDTH][3]; // R,G,B values for each pixel
void Hyperdimensional() {
    static float time = 0.0f;
    static float rotation = 0.0f;
    static float rotationVelocity = 0.0f;

    static float smoothBass = 0.0f;
    static float smoothMid  = 0.0f;
    static float smoothHigh = 0.0f;

    static int particleCount = 0;
    static Vec2 particles[200];
    static float particleLife[200];
    static uint8_t particleColors[200][3];
    
    // Fade screen slightly instead of clearing hard (afterglow effect)
    dma_display->fillScreenRGB888(0, 0,0); // low alpha = persistent glow
    float t = millis() * 0.001f;

    // === AUDIO ENERGY CALCULATION ===
    float bassEnergy = 0.0f, midEnergy = 0.0f, highEnergy = 0.0f;
    int third = SAMPLES / 3;

    for (int i = 0; i < third; i++)
        bassEnergy += fabs((float)samples[i] / 32768.0f);
    for (int i = third; i < third * 2; i++)
        midEnergy += fabs((float)samples[i] / 32768.0f);
    for (int i = third * 2; i < SAMPLES; i++)
        highEnergy += fabs((float)samples[i] / 32768.0f);

    bassEnergy /= third;
    midEnergy  /= third;
    highEnergy /= third;

    // === SMOOTHING FILTERS ===
    smoothBass = 0.85f * smoothBass + 0.15f * bassEnergy;
    smoothMid  = 0.85f * smoothMid  + 0.15f * midEnergy;
    smoothHigh = 0.85f * smoothHigh + 0.15f * highEnergy;

    // === PARTICLE SPAWN (AUDIO REACTIVE) ===
    if (smoothBass > 0.25f && particleCount < 200) {
        int spawnCount = min(6, 200 - particleCount);
        for (int i = 0; i < spawnCount; i++) {
            float angle = random(0, 628) / 100.0f;
            float speed = 0.5f + smoothBass * 2.0f;
            particles[particleCount] = { fastCos(angle) * speed, fastSin(angle) * speed};
            particleLife[particleCount] = 1.0f;
 
            // Color shifts with sound energy
            particleColors[particleCount][0] = 180 + 75 * smoothBass; // red = bass
            particleColors[particleCount][1] = 100 + 155 * smoothMid; // green = mids
            particleColors[particleCount][2] = 120 + 135 * smoothHigh; // blue = highs

            particleCount++;
        }
    }

    // === ORBITERS ===
    const int NUM_ORBITERS = 4;
    Vec2 orbiters[NUM_ORBITERS];
    float orbiterPhases[NUM_ORBITERS] = {0.0f, PI/2, PI, 3*PI/2};
    float orbiterSpeeds[NUM_ORBITERS] = {1.0f, 1.5f, 0.8f, 1.2f};

    for (int i = 0; i < NUM_ORBITERS; i++) {
        float phase = orbiterPhases[i] + t * orbiterSpeeds[i];

        float R = 15.0f + smoothBass * 10.0f;
        float r = 8.0f + smoothMid * 6.0f;
        float d = 12.0f + smoothHigh * 8.0f;

        float angle = phase + rotation;
        float x = (R - r) * fastCos(angle) + d * fastCos((R - r) * angle / r);
        float y = (R - r) * fastSin(angle) - d *fastSin((R - r) * angle / r);

        x += 4.0f * fastSin(t * 3.0f + i) * smoothBass;
        y += 4.0f * fastCos(t * 2.5f + i) * smoothMid;

        orbiters[i] = {x + WIDTH / 2, y + HEIGHT / 2};

        // === COLOR REACTIVE GLOW ===
        uint8_t rcol = 180 + 75 * fastSin(t * 2.0f + i) * smoothBass;
        uint8_t gcol = 100 + 155 * fabs(fastSin(t * 1.5f + i)) * smoothMid;
        uint8_t bcol = 150 + 105 * fabs(fastCos(t * 1.3f + i)) * smoothHigh;

        int ox = (int)orbiters[i].x;
        int oy = (int)orbiters[i].y;

        for (int gy = -2; gy <= 2; gy++) {
            for (int gx = -2; gx <= 2; gx++) {
                int px = ox + gx;
                int py = oy + gy;
                if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
                    float dist = sqrt(gx * gx + gy * gy);
                    float intensity = 1.0f - dist / 3.0f;
                    if (intensity > 0) {
                        dma_display->drawPixelRGB888(px, py,
                            std::min(255, (int)(rcol * intensity)),
                            std::min(255, (int)(gcol * intensity)),
                            std::min(255, (int)(bcol * intensity)));
                    }
                }
            }
        }
    }

    // === CONNECTIONS (BEZIER CURVES) ===
    for (int i = 0; i < NUM_ORBITERS; i++) {
        for (int j = i + 1; j < NUM_ORBITERS; j++) {
            Vec2 start = orbiters[i];
            Vec2 end = orbiters[j];
            Vec2 mid = {
                (start.x + end.x) / 2.0f + 5.0f * fastSin(t + i + j),
                (start.y + end.y) / 2.0f + 5.0f * fastCos(t + i + j)
            };

            for (float u = 0.0f; u <= 1.0f; u += 0.05f) {
                float x = (1 - u) * (1 - u) * start.x + 2 * (1 - u) * u * mid.x + u * u * end.x;
                float y = (1 - u) * (1 - u) * start.y + 2 * (1 - u) * u * mid.y + u * u * end.y;
                int px = (int)x;
                int py = (int)y;

                if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
                    float brightness = 0.8f + 0.4f * smoothBass;
                    uint8_t r = 120 * brightness * (1.0f + smoothBass);
                    uint8_t g = 200 * brightness * (1.0f + smoothMid);
                    uint8_t b = 255 * brightness * (1.0f + smoothHigh);
                    dma_display->drawPixelRGB888(px, py,
                        std::min(255, (int)r),
                        std::min(255, (int)g),
                        std::min(255, (int)b));
                }
            }
        }
    }

    // === PARTICLES UPDATE ===
    for (int i = 0; i < particleCount; i++) {
        particleLife[i] -= 0.01f;

        if (particleLife[i] > 0) {
            Vec2 force = {0, 0};
            for (int j = 0; j < NUM_ORBITERS; j++) {
                Vec2 diff = {orbiters[j].x - particles[i].x - WIDTH / 2,
                             orbiters[j].y - particles[i].y - HEIGHT / 2};
                float dist = sqrt(diff.x * diff.x + diff.y * diff.y);
                if (dist > 1.0f) {
                    force.x += diff.x / (dist * dist) * 0.5f;
                    force.y += diff.y / (dist * dist) * 0.5f;
                }
            }

            particles[i].x += force.x;
            particles[i].y += force.y;

            int px = (int)(particles[i].x + WIDTH / 2);
            int py = (int)(particles[i].y + HEIGHT / 2);

            if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
                float life = particleLife[i];

            
                dma_display->drawPixelRGB888(px, py,
                    particleColors[i][0] * life,
                    particleColors[i][1] * life,
                    particleColors[i][2] * life);
            }
        } else {
            particles[i] = particles[particleCount - 1];
            particleLife[i] = particleLife[particleCount - 1];
            for (int c = 0; c < 3; c++)
                particleColors[i][c] = particleColors[particleCount - 1][c];
            particleCount--;
            i--;
        }
    }

    // === ROTATION (SMOOTH AND REACTIVE) ===
    rotationVelocity = rotationVelocity * 0.9f + (smoothBass - 0.1f) * 0.02f;
    rotation += 0.02f + rotationVelocity;

    time += 0.016f;
    dma_display->flipDMABuffer();
}





 












// ============================================================================
// FUNCTION 1: Neon Rain Dance - Cascading Audio Particles with Physics
// ============================================================================
void NeonRainDance(int charge) {
    static float rainDrops[100][6]; // Reduced for performance
    static int dropCount = 0;
    static float beatPulse = 0.0f;
    static float strobePhase = 0.0f;
    static float fractalTime = 0.0f;
    static float lastBass = 0.0f, lastMid = 0.0f, lastHigh = 0.0f;
    
    float t = millis() * 0.001f;
    fractalTime += 0.03f; // Faster animation
    
    // Analyze audio in frequency bands with smoothing
    float subBass = 0.0f, bass = 0.0f, lowMid = 0.0f, highMid = 0.0f, treble = 0.0f;
    int bandSize = SAMPLES / 5;
    
    for (int i = 0; i < bandSize; i++) {
        subBass += fabs((float)samples[i] / 32768.0f);
        bass += fabs((float)samples[i + bandSize] / 32768.0f);
        lowMid += fabs((float)samples[i + bandSize * 2] / 32768.0f);
        highMid += fabs((float)samples[i + bandSize * 3] / 32768.0f);
        treble += fabs((float)samples[i + bandSize * 4] / 32768.0f);
    }
    subBass /= bandSize; bass /= bandSize; lowMid /= bandSize;
    highMid /= bandSize; treble /= bandSize;
    
    // Smooth audio values
    lastBass = lastBass * 0.7f + bass * 0.3f;
    lastMid = lastMid * 0.7f + lowMid * 0.3f;
    lastHigh = lastHigh * 0.7f + treble * 0.3f;
    
    // Beat detection
    beatPulse = beatPulse * 0.8f + (lastBass + bass) * 0.6f;
    
    // --- OPTIMIZED SMOOTH BACKGROUND ---
    for (int y = 0; y < HEIGHT; y++) {
        float fy = (float)y / HEIGHT;
        for (int x = 0; x < WIDTH; x++) {
            float fx = (float)x / WIDTH;
            
            // Fast smooth gradient with audio modulation
            float baseHue = fractalTime * 15.0f; // Slower hue rotation
            float xWave = fastSin(fx * 6.0f + t * 3.0f) * 0.4f;
            float yWave = fastCos(fy * 4.0f - t * 2.0f) * 0.3f;
            
            // Smooth color blending
            float hue1 = baseHue + xWave * 50.0f;
            float hue2 = baseHue + 120.0f + yWave * 40.0f; // Complementary
            float hue3 = baseHue + 240.0f + (xWave + yWave) * 30.0f; // Triadic
            
            // Audio-modulated blend weights
            float blend1 = 0.4f + lastBass * 0.3f;
            float blend2 = 0.3f + lastMid * 0.4f;
            float blend3 = 0.3f + lastHigh * 0.3f;
            float totalBlend = blend1 + blend2 + blend3;
            
            // Normalize and blend hues
            float finalHue = (hue1 * blend1 + hue2 * blend2 + hue3 * blend3) / totalBlend;
            finalHue = fmod(finalHue, 360.0f);
            
            // Smooth saturation and value
            float saturation = 0.7f + lastMid * 0.3f;
            float value = 0.2f + lastBass * 0.4f + beatPulse * 0.3f;
            
            // HSV to RGB with smooth gradients
            float c = value * saturation;
            float h_normalized = finalHue / 60.0f;
            float x_val = c * (1.0f - fabs(fmod(h_normalized, 2.0f) - 1.0f));
            float m = value - c;
            
            float r, g, b;
            int hueSection = (int)h_normalized;
            switch (hueSection) {
                case 0: r = c; g = x_val; b = 0; break;
                case 1: r = x_val; g = c; b = 0; break;
                case 2: r = 0; g = c; b = x_val; break;
                case 3: r = 0; g = x_val; b = c; break;
                case 4: r = x_val; g = 0; b = c; break;
                default: r = c; g = 0; b = x_val; break;
            }
            
            uint8_t red = (r + m) * 255;
            uint8_t green = (g + m) * 255;
            uint8_t blue = (b + m) * 255;
            
            // Add subtle pulsing with beat
            float pulse = 1.0f + beatPulse * 0.2f;
            red = constrain(red * pulse, 0, 255);
            green = constrain(green * pulse, 0, 255);
            blue = constrain(blue * pulse, 0, 255);
            
            dma_display->drawPixelRGB888(x, y, red, green, blue);
        }
    }
    
    // --- FASTER RAINDROP SPAWNING ---
    if ((lastHigh > 0.5f || lastBass > 0.15f) && dropCount < 30) {
        int spawnCount = min((int)((lastHigh + lastBass) * 15), 100 - dropCount);
        for (int i = 0; i < spawnCount; i++) {
            int audioX = random(0, SAMPLES);
            float localAmp = fabs((float)samples[audioX]) / 32768.0f;
            
            rainDrops[dropCount][0] = random(0, WIDTH); // Faster random positioning
            rainDrops[dropCount][1] = -random(0, 100); // Start closer to top
            rainDrops[dropCount][2] = 1.0f + localAmp * 0.3f + lastBass * 0.02f; // Faster velocity
            rainDrops[dropCount][3] = fmod(t * 200.0f + audioX * 5.0f, 45.0f); // Faster hue change
            rainDrops[dropCount][4] = 1; // Shorter trails
            rainDrops[dropCount][5] = 0.8f + localAmp * 0.5f; // Higher intensity
            dropCount++;
        }
    }
    
    // --- OPTIMIZED RAINDROP UPDATE ---
    for (int i = 0; i < dropCount; i++) {
        // Faster gravity
        float gravity = 0.8f + lastBass * 0.8f;
        rainDrops[i][2] += gravity * 0.2f; // Reduced multiplier for speed
        rainDrops[i][1] += rainDrops[i][2] * 0.7f; // Faster movement
        
        // Simpler oscillation
        float xOffset = fastSin(rainDrops[i][1] * 0.1f + t * 4.0f) * lastMid * 3.0f;
        float currentX = rainDrops[i][0] + xOffset;
        
        // Wrap around
        if (currentX < 0) currentX += WIDTH;
        if (currentX >= WIDTH) currentX -= WIDTH;
        
        // Draw optimized trail
        int trailLength = (int)rainDrops[i][4];
        for (int trail = 0; trail < trailLength; trail++) {
            float trailY = rainDrops[i][1] - trail * 0.8f; // Closer trail spacing
            
            if (trailY >= 0 && trailY < HEIGHT) {
                int px = (int)currentX;
                int py = (int)trailY;
                
                if (px >= 0 && px < WIDTH) {
                    float trailFade = 1.0f - (float)trail / trailLength;
                    float intensity = rainDrops[i][5] * trailFade;
                    
                    // Fast color calculation
                    float h = fmod(rainDrops[i][3] + trail * 8.0f, 360.0f);
                    float s = 0.9f;
                    float v = intensity;
                    
                    // Quick HSV to RGB
                    float c = v * s;
                    float h_norm = h / 60.0f;
                    float x_val = c * (1.0f - fabs(fmod(h_norm, 2.0f) - 1.0f));
                    float m = v - c;
                    
                    uint8_t r, g, b;
                    int section = (int)h_norm;
                    switch (section) {
                        case 0: r = (c + m) * 255; g = (x_val + m) * 255; b = m * 255; break;
                        case 1: r = (x_val + m) * 255; g = (c + m) * 255; b = m * 255; break;
                        case 2: r = m * 255; g = (c + m) * 255; b = (x_val + m) * 255; break;
                        case 3: r = m * 255; g = (x_val + m) * 255; b = (c + m) * 255; break;
                        case 4: r = (x_val + m) * 255; g = m * 255; b = (c + m) * 255; break;
                        default: r = (c + m) * 255; g = m * 255; b = (x_val + m) * 255; break;
                    }
                    
                    // Simple glow for head only
                    if (trail == 0) {
                        dma_display->drawPixelRGB888(px, py, 
                            min(255, r + 80), min(255, g + 80), min(255, b + 80));
                        // Draw one extra pixel for glow
                        if (py + 1 < HEIGHT) {
                            dma_display->drawPixelRGB888(px, py + 1, r, g, b);
                        }
                    } else {
                        dma_display->drawPixelRGB888(px, py, r, g, b);
                    }
                }
            }
        }
        
        // Fast splash detection and removal
        if (rainDrops[i][1] >= HEIGHT + 5) { // Allow some overshoot for visual effect
            // Remove drop by swapping with last
            if (i < dropCount - 1) {
                for (int k = 0; k < 6; k++) {
                    rainDrops[i][k] = rainDrops[dropCount - 1][k];
                }
            }
            dropCount--;
            i--; // Recheck current index
        }
    }
    
    strobePhase += 0.12f + beatPulse * 0.8f;
    dma_display->flipDMABuffer();
}



// ============================================================================
//    Kaleidoscope - Rotating Mirror Fractals
// ============================================================================
 // ============================================================================
// FUNCTION 1: Neon Rain Dance - Cascading Audio Particles with Physics
// ============================================================================
void updateSupercharged4(int charge) {
    static float rainDrops[150][6]; // x, y, velocity, hue, trail_length, intensity
    static int dropCount = 0;
    static float beatPulse = 0.0f;
    static float strobePhase = 0.0f;
    static uint8_t backgroundNoise[WIDTH][HEIGHT];
    
    float t = millis() * 0.001f;
    
    // Analyze audio in frequency bands
    float subBass = 0.0f, bass = 0.0f, lowMid = 0.0f, highMid = 0.0f, treble = 0.0f;
    int bandSize = SAMPLES / 5;
    
    for (int i = 0; i < bandSize; i++) {
        subBass += fabs((float)samples[i] / 32768.0f);
        bass += fabs((float)samples[i + bandSize] / 32768.0f);
        lowMid += fabs((float)samples[i + bandSize * 2] / 32768.0f);
        highMid += fabs((float)samples[i + bandSize * 3] / 32768.0f);
        treble += fabs((float)samples[i + bandSize * 4] / 32768.0f);
    }
    subBass /= bandSize; bass /= bandSize; lowMid /= bandSize;
    highMid /= bandSize; treble /= bandSize;
    
    // Beat detection
    beatPulse = beatPulse * 0.9f + (subBass + bass) * 0.5f;
    
    // Update background noise field with strobe effect
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            float nx = (float)x / WIDTH * 6.0f;
            float ny = (float)y / HEIGHT * 6.0f;
            
            // Fast moving interference pattern
            float pattern = sin(nx * 3.0f + t * 5.0f) * cos(ny * 2.5f - t * 4.0f);
            pattern += sin((nx + ny) * 4.0f + strobePhase * 10.0f) * 0.5f;
            
            backgroundNoise[x][y] = constrain(
                15 + pattern * 20.0f + beatPulse * 40.0f,
                0, 80
            );
        }
    }
    
    // Spawn raindrops based on audio
    if (treble > 0.25f && dropCount < 150) {
        int spawnCount = min((int)(treble * 8), 150 - dropCount);
        for (int i = 0; i < spawnCount; i++) {
            int audioX = random(0, SAMPLES);
            float localAmp = fabs((float)samples[audioX] / 32768.0f);
            
            rainDrops[dropCount][0] = (float)audioX / SAMPLES * WIDTH; // x
            rainDrops[dropCount][1] = -2.0f; // y (start above screen)
            rainDrops[dropCount][2] = 0.5f + localAmp * 2.5f; // velocity
            rainDrops[dropCount][3] = fmod(t * 100.0f + audioX * 5.0f, 360.0f); // hue
            rainDrops[dropCount][4] = 3.0f + localAmp * 8.0f; // trail length
            rainDrops[dropCount][5] = 0.6f + localAmp * 0.4f; // intensity
            dropCount++;
        }
    }
    
    // Clear screen with animated background
    dma_display->clearScreen();
    
    // Draw animated background with audio reactivity
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            uint8_t noise = backgroundNoise[x][y];
            
            // Add frequency-based color shifts
            float colorShift = sin(t * 3.0f + x * 0.2f) * lowMid * 30.0f;
            uint8_t r = noise + colorShift;
            uint8_t g = noise * 0.8f;
            uint8_t b = noise * 1.2f + highMid * 50.0f;
            
            dma_display->drawPixelRGB888(x, y, r, g, b);
        }
    }
    
    // Update and draw raindrops with physics
    for (int i = 0; i < dropCount; i++) {
        // Apply gravity and audio modulation
        float gravity = 0.08f + bass * 0.1f;
        rainDrops[i][2] += gravity;
        rainDrops[i][1] += rainDrops[i][2];
        
        // Horizontal oscillation based on mid frequencies
        float xOffset = sin(rainDrops[i][1] * 0.3f + t * 2.0f) * lowMid * 3.0f;
        float currentX = rainDrops[i][0] + xOffset;
        
        // Draw trail
        int trailLength = (int)rainDrops[i][4];
        for (int trail = 0; trail < trailLength; trail++) {
            float trailY = rainDrops[i][1] - trail * 1.5f;
            
            if (trailY >= 0 && trailY < HEIGHT) {
                int px = (int)currentX;
                int py = (int)trailY;
                
                if (px >= 0 && px < WIDTH) {
                    float trailFade = 1.0f - (float)trail / trailLength;
                    float intensity = rainDrops[i][5] * trailFade;
                    
                    // HSV to RGB conversion for rainbow effect
                    float h = rainDrops[i][3] + trail * 10.0f;
                    h = fmod(h, 360.0f);
                    float s = 0.9f;
                    float v = intensity;
                    
                    float c = v * s;
                    float x_val = c * (1.0f - fabs(fmod(h / 60.0f, 2.0f) - 1.0f));
                    float m = v - c;
                    
                    float r, g, b;
                    if (h < 60)       { r = c; g = x_val; b = 0; }
                    else if (h < 120) { r = x_val; g = c; b = 0; }
                    else if (h < 180) { r = 0; g = c; b = x_val; }
                    else if (h < 240) { r = 0; g = x_val; b = c; }
                    else if (h < 300) { r = x_val; g = 0; b = c; }
                    else              { r = c; g = 0; b = x_val; }
                    
                    uint8_t red = (r + m) * 255;
                    uint8_t green = (g + m) * 255;
                    uint8_t blue = (b + m) * 255;
                    
                    // Glow effect for head of drop
                    if (trail < 2) {
                        for (int gy = -1; gy <= 1; gy++) {
                            for (int gx = -1; gx <= 1; gx++) {
                                int glowX = px + gx;
                                int glowY = py + gy;
                                if (glowX >= 0 && glowX < WIDTH && glowY >= 0 && glowY < HEIGHT) {
                                    float glowFade = 1.0f - sqrt(gx*gx + gy*gy) * 0.4f;
                                    if (glowFade > 0) {
                                        dma_display->drawPixelRGB888(glowX, glowY,
                                            min(255, red + (int)(100 * glowFade * treble)),
                                            min(255, green + (int)(100 * glowFade * treble)),
                                            min(255, blue + (int)(100 * glowFade * treble)));
                                    }
                                }
                            }
                        }
                    } else {
                        dma_display->drawPixelRGB888(px, py, red, green, blue);
                    }
                }
            }
        }
        
        // Splash effect when drop hits bottom
        if (rainDrops[i][1] >= HEIGHT) {
            int splashX = (int)currentX;
            for (int s = -3; s <= 3; s++) {
                int sx = splashX + s;
                if (sx >= 0 && sx < WIDTH) {
                    float splashIntensity = 1.0f - fabs(s) / 4.0f;
                    uint8_t white = 200 * splashIntensity;
                    dma_display->drawPixelRGB888(sx, HEIGHT - 1, white, white, white);
                    if (HEIGHT > 1) {
                        dma_display->drawPixelRGB888(sx, HEIGHT - 2, white * 0.5f, white * 0.5f, white * 0.5f);
                    }
                }
            }
            
            // Remove drop
            for (int j = i; j < dropCount - 1; j++) {
                for (int k = 0; k < 6; k++) {
                    rainDrops[j][k] = rainDrops[j + 1][k];
                }
            }
            dropCount--;
            i--;
        }
    }
    
    strobePhase += 0.1f + beatPulse * 0.3f;
    dma_display->flipDMABuffer();
}

// ============================================================================
// FUNCTION 2: Smooth Kaleidoscope with Color Transitions
// ============================================================================
void Kaleidoscope(int trans) {
    static float kaleido_time = 0.0f;
    static float rotation = 0.0f;
    static float zoomPulse = 1.0f;
    static int currentPattern = 0;
    static unsigned long lastPatternChange = 0;
    
    // Smooth color transition variables
    static float colorPhase = 0.0f;
    static float hueOffset = 0.0f;
    static float prevHueOffset = 0.0f;
    static float transitionProgress = 0.0f;
    
    // Use trans to control pattern, or auto-cycle
    int symmetry = 4 + (trans % 5) * 2; // 4, 6, 8, 10, 12 based on trans
    
    float t = millis() * 0.001f;
    
    // Frequency analysis with smoothing buffers
    static float smoothLowFreq = 0.0f;
    static float smoothMidFreq = 0.0f;
    static float smoothHighFreq = 0.0f;
    
    float lowFreq = 0.0f, midFreq = 0.0f, highFreq = 0.0f;
    int third = SAMPLES / 3;
    
    for (int i = 0; i < third; i++) {
        lowFreq += fabs((float)samples[i] / 32768.0f);
    }
    for (int i = third; i < third * 2; i++) {
        midFreq += fabs((float)samples[i] / 32768.0f);
    }
    for (int i = third * 2; i < SAMPLES; i++) {
        highFreq += fabs((float)samples[i] / 32768.0f);
    }
    lowFreq /= third; midFreq /= third; highFreq /= third;
    
    // Apply soft limiting to prevent distortion
    lowFreq = min(lowFreq, 0.8f);
    midFreq = min(midFreq, 0.8f);
    highFreq = min(highFreq, 0.8f);
    
    // Smooth the audio values over time (exponential moving average)
    float smoothing = 0.7f; // Higher = smoother (0.0-1.0)
    smoothLowFreq = smoothLowFreq * smoothing + lowFreq * (1.0f - smoothing);
    smoothMidFreq = smoothMidFreq * smoothing + midFreq * (1.0f - smoothing);
    smoothHighFreq = smoothHighFreq * smoothing + highFreq * (1.0f - smoothing);
    
    // Use smoothed values
    lowFreq = smoothLowFreq;
    midFreq = smoothMidFreq;
    highFreq = smoothHighFreq;
    
    // Auto-cycle pattern every 10 seconds with smooth transition
    if (millis() - lastPatternChange > 10000) {
       // currentPattern = (currentPattern + 1) % 10;
       currentPattern = random(0,10);
        lastPatternChange = millis();
        transitionProgress = 0.0f;
        prevHueOffset = hueOffset;
        hueOffset = fmod(hueOffset + 60.0f + random(-30, 30), 360.0f);
    }
    
    // Smooth transition between color schemes
    if (transitionProgress < 1.0f) {
        transitionProgress += 0.01f; // Adjust speed here (lower = slower)
    }
    float smoothTransition = transitionProgress * transitionProgress * (3.0f - 2.0f * transitionProgress); // Smoothstep
    float currentHueOffset = prevHueOffset + (hueOffset - prevHueOffset) * smoothTransition;
    
    // Gentle color phase rotation
    colorPhase += 0.2f + midFreq * 0.3f; // Slower base speed
    
    // Beat-driven zoom (with limiting)
    zoomPulse = zoomPulse * 0.92f + (lowFreq + midFreq) * 0.4f; // Reduced from 0.5
    float zoom = 1.0f + zoomPulse * 0.6f; // Reduced max zoom from 0.8
    
    int cx = WIDTH / 2;
    int cy = HEIGHT / 2;
    
    // Render kaleidoscope
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            // Convert to polar coordinates
            float dx = (x - cx) / zoom;
            float dy = (y - cy) / zoom;
            float dist = sqrt(dx * dx + dy * dy);
            float angle = atan2(dy, dx);
            
            // Apply rotation
            angle += rotation;
            
            // Kaleidoscope mirror effect
            angle = fmod(angle, TWO_PI / symmetry);
            if ((int)(angle / (TWO_PI / symmetry / 2.0f)) % 2 == 1) {
                angle = TWO_PI / symmetry - angle;
            }
            
            // Audio-modulated pattern coordinates
            float u = dist * 0.3f + kaleido_time * 0.5f;
            float v = angle * 3.0f + kaleido_time * 0.3f;
            
            // Complex wave interference pattern
            float pattern = 0.0f;
            
            // Use trans to select different mathematical patterns
            int patternMode = trans % 10;
            
            switch(patternMode) {
                case 0: // Rotating spirals
                    pattern += fastSin(u * 4.0f - v * 2.0f + kaleido_time * 2.0f) * 0.5f;
                    pattern += fastCos(u * 3.0f + v * 3.0f - kaleido_time * 1.5f) * 0.5f;
                    break;
                    
                case 1: // Concentric rings
                    pattern += fastSin(dist * 6.0f + kaleido_time + lowFreq * 5.0f);
                    pattern += fastCos(angle * symmetry + midFreq * 3.0f) * 0.5f;
                    break;
                    
                case 2: // Radial burst
                    pattern += exp(-dist * 0.3f) * fastSin(angle * symmetry * 3.0f + kaleido_time * 4.0f);
                    pattern += fastCos(dist * 4.0f - kaleido_time * 2.0f);
                    break;
                    
                case 3: // Hyperbolic pattern
                    pattern += fastSin(u * 5.0f / (dist + 0.5f)) * fastCos(v * 4.0f);
                    pattern += fastSin(angle * symmetry + kaleido_time);
                    break;
                    
                case 4: // Grid interference
                    pattern += fastSin(u * 10.0f) * fastCos(v * 8.0f);
                    pattern += fastSin(dist * 5.0f + kaleido_time) * highFreq;
                    break;
                    
                case 5: // Flower petals
                    pattern += fastSin(angle * symmetry * 2.0f + kaleido_time) * fastCos(dist * 3.0f);
                    pattern += fastSin(u * 6.0f + v * 2.0f);
                    break;
                    
                case 6: // Tunnel effect
                    pattern += fastSin(1.0f / (dist + 0.2f) * 10.0f + kaleido_time * 3.0f);
                    pattern += fastCos(angle * symmetry + lowFreq * 4.0f);
                    break;
                    
                case 7: // Star burst
                    pattern += pow(fastSin(angle * symmetry + kaleido_time), 2.0f) * exp(-dist * 0.4f);
                    pattern += fastCos(dist * 8.0f - kaleido_time);
                    break;
                    
                case 8: // Chevron waves
                    pattern += fastSin(fabs(dx) * 4.0f + fabs(dy) * 3.0f - kaleido_time * 2.0f);
                    pattern += fastCos(angle * symmetry) * midFreq;
                    break;
                    
                case 9: // Plasma vortex
                    pattern += fastSin(dx * 5.0f + kaleido_time) * fastCos(dy * 4.0f - kaleido_time);
                    pattern += fastSin((dx + dy) * 6.0f + angle * symmetry);
                    break;

                    default:currentPattern = 7; break;
            }
            
            // Add universal layers
            float rings = fastSin(dist * 2.0f + kaleido_time + lowFreq * 5.0f) * 
                         fastCos(angle * symmetry + midFreq * 3.0f);
            pattern += rings * 0.4f;
            
            // Normalize pattern
            pattern = (pattern + 3.0f) / 6.0f;
            pattern = constrain(pattern, 0.0f, 1.0f);
            
            // Sample audio for this pixel's position with limiting
            int audioIdx = (int)(fmod(angle * symmetry + dist, 1.0f) * SAMPLES);
            audioIdx = constrain(audioIdx, 0, SAMPLES - 1);
            float audioMod = fabs((float)samples[audioIdx] / 32768.0f);
            
            // Limit and smooth individual pixel audio
            audioMod = min(audioMod, 0.7f); // Hard limit to prevent spikes
            audioMod = pow(audioMod, 1.5f); // Compress loud signals more
            
            // SMOOTH color mapping with gradual transitions
            // Base hue rotates slowly and smoothly
            float baseHue = colorPhase * 0.5f + currentHueOffset;
            
            // Add pattern-based variation (smoother, limited)
            float patternHue = pattern * 100.0f; // Further reduced from 120
            
            // Add spatial variation (smoother)
            float spatialHue = dist * 15.0f; // Reduced from 20
            
            // Add gentle audio modulation (with limiting)
            float audioHue = audioMod * 40.0f; // Reduced from 60 and limited
            
            // Combine all hue components smoothly
            float hue = fmod(baseHue + patternHue + spatialHue + audioHue, 360.0f);
            
            // Smooth saturation changes (with audio limiting)
            float baseSaturation = 0.75f + sin(colorPhase * 0.1f) * 0.15f;
            float saturation = baseSaturation + audioMod * 0.1f; // Reduced from 0.15
            saturation = constrain(saturation, 0.6f, 0.95f); // Tighter max range
            
            // Smooth brightness with gentle pulsing (with limiting)
            float baseBrightness = 0.45f + sin(colorPhase * 0.05f) * 0.1f;
            float brightness = baseBrightness + pattern * 0.3f + audioMod * 0.12f; // Reduced audio impact
            
            // Gentle strobe effect on strong beats (heavily limited)
            if (lowFreq > 0.6f) { // Raised threshold from 0.7
                float strobeAmount = (lowFreq - 0.6f) * 0.3f; // Reduced from 0.4
                brightness = min(0.95f, brightness + strobeAmount); // Lower max
            }
            
            brightness = constrain(brightness, 0.25f, 0.95f); // Tighter range, lower max
            
            // HSV to RGB conversion
            float c = brightness * saturation;
            float h_prime = hue / 60.0f;
            float x_val = c * (1.0f - fabs(fmod(h_prime, 2.0f) - 1.0f));
            float m = brightness - c;
            
            float r, g, b;
            if (h_prime < 1.0f)      { r = c; g = x_val; b = 0; }
            else if (h_prime < 2.0f) { r = x_val; g = c; b = 0; }
            else if (h_prime < 3.0f) { r = 0; g = c; b = x_val; }
            else if (h_prime < 4.0f) { r = 0; g = x_val; b = c; }
            else if (h_prime < 5.0f) { r = x_val; g = 0; b = c; }
            else                     { r = c; g = 0; b = x_val; }
            
            uint8_t red = (r + m) * 255;
            uint8_t green = (g + m) * 255;
            uint8_t blue = (b + m) * 255;
            
            dma_display->drawPixelRGB888(x, y, red, green, blue);
        }
    }
    
    // Draw frequency spectrum bars around the edge as bonus effect
    if (trans % 2 == 0) {
        int barWidth = WIDTH / 16;
        for (int i = 0; i < 16; i++) {
            int sampleIdx = (i * SAMPLES) / 16;
            float amp = fabs((float)samples[sampleIdx] / 32768.0f);
            int barHeight = (int)(amp * 8.0f);
            
            // Smooth bar colors matching the kaleidoscope palette
            float barHue = fmod(colorPhase * 0.5f + currentHueOffset + i * 15.0f, 360.0f);
            float barC = amp;
            float barH = barHue / 60.0f;
            float barX = barC * (1.0f - fabs(fmod(barH, 2.0f) - 1.0f));
            
            float barR, barG, barB;
            if (barH < 1.0f)      { barR = barC; barG = barX; barB = 0; }
            else if (barH < 2.0f) { barR = barX; barG = barC; barB = 0; }
            else if (barH < 3.0f) { barR = 0; barG = barC; barB = barX; }
            else if (barH < 4.0f) { barR = 0; barG = barX; barB = barC; }
            else if (barH < 5.0f) { barR = barX; barG = 0; barB = barC; }
            else                  { barR = barC; barG = 0; barB = barX; }
            
            // Bottom edge
            for (int h = 0; h < barHeight && h < 4; h++) {
                for (int w = 0; w < barWidth && i * barWidth + w < WIDTH; w++) {
                    int px = i * barWidth + w;
                    int py = HEIGHT - 1 - h;
                    if (px < WIDTH) {
                        dma_display->drawPixelRGB888(px, py, 
                            barR * 255, barG * 255, barB * 255);
                    }
                }
            }
        }
    }
    
    kaleido_time += 0.04f + midFreq * 0.08f;
    rotation += 0.02f + lowFreq * 0.06f;
    
    dma_display->flipDMABuffer();
}