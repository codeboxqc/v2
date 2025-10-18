#include <Arduino.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h>
#include "bar.h"
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#define NUM_BODIES 3
#define G 10.0f
#define dt 0.0016f  //0.0008f
#define scale 18.0f
#define maxTrailLength 300 //400
#define transitionSpeed 0.001f
#define baseTransitionSpeed 0.004f
 
 #define SIM_RANGE 6.2f 
 

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
  dma_display->clearScreen();
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
      float yOffset = amplitude * 2.0f + 0.5f * sin(t + normX * 10.0);
      int y = (int)(fy + yOffset);
      if (y < 0 || y >= HEIGHT) continue;

      // Modulate color with waveform
      float glow = 0.8 + 0.2 * fabs(amplitude);
      float shimmer = 0.5 + 0.5 * sin(t + normX * 12.0);
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

void body3() {
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