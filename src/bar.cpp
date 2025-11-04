#include <Arduino.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h>
#include <algorithm> // for std::max
#include <cmath>     // for sin, fmax

#include "bar.h"

// HUB75 setup
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>



double bins[nBands];
double barHeight[nBands];
double maxBin = 1.0;
double peakHold[nBands];

// Convert HSV to RGB for vibrant colors
uint16_t hsvToRgb(float h, float s, float v) {
  float r, g, b;
  int i = int(h * 6);
  float f = h * 6 - i;
  float p = v * (1 - s);
  float q = v * (1 - f * s);
  float t = v * (1 - (1 - f) * s);

  switch (i % 6) {
    case 0: r = v, g = t, b = p; break;
    case 1: r = q, g = v, b = p; break;
    case 2: r = p, g = v, b = t; break;
    case 3: r = p, g = q, b = v; break;
    case 4: r = t, g = p, b = v; break;
    case 5: r = v, g = p, b = q; break;
  }

  return dma_display->color565(r * 255, g * 255, b * 255);
}







void drawBars() {
  dma_display->clearScreen();

  int barWidth = PANEL_RES_X / nBands;
  int spacing = 1;

  int peakBand = 0;
  double maxHeight = 0;
  for (int i = 0; i < nBands; i++) {
    if (barHeight[i] > maxHeight) {
      maxHeight = barHeight[i];
      peakBand = i;
    }
  }

  for (int band = 0; band < nBands; band++) {
    int x = band * barWidth;
    int height = (int)barHeight[band];

    // Update peak hold
    if (height > peakHold[band]) {
      peakHold[band] = height;
    } else {
      peakHold[band] = max(peakHold[band] - 0.5, 0.0); // slow decay
    }

    // Draw bar
    for (int y = 0; y < height; y++) {
      float hue = (float)band / nBands;
      float value = (float)y / PANEL_RES_Y;
      float saturation = (band == peakBand) ? 0.5 : 1.0;
      uint16_t color = hsvToRgb(hue, saturation, value);

      for (int barX = x + spacing; barX < x + barWidth - spacing; barX++) {
        if (barX >= 0 && barX < PANEL_RES_X && (PANEL_RES_Y - 1 - y) >= 0) {
          dma_display->drawPixel(barX + 2, PANEL_RES_Y - 1 - y, color);
        }
      }
    }

    // Draw peak cap
    int peakY = PANEL_RES_Y - 1 - (int)peakHold[band];
    if (peakY >= 0 && peakY < PANEL_RES_Y) {
      uint16_t capColor = capColor = dma_display->color565(128+peakY ,64   , 6   );   //cap max
      for (int barX = x + spacing; barX < x + barWidth - spacing; barX++) {
     
        
        dma_display->drawPixel(barX + 2, peakY, capColor);
      }
    }
  }

  // Optional: red border
  dma_display->drawRect(0, 0, PANEL_RES_X, PANEL_RES_Y, dma_display->color565(255, 0, 0));
  dma_display->flipDMABuffer();
}



 void drawBarsArt() {
  dma_display->clearScreen();

  int barWidth = PANEL_RES_X / nBands;
  int spacing = 1;
  float t = millis() * 0.0015f;  // animation time

  // --- Find peak band ---
  int peakBand = 0;
  double maxHeight = 0;
  for (int i = 0; i < nBands; i++) {
    if (barHeight[i] > maxHeight) {
      maxHeight = barHeight[i];
      peakBand = i;
    }
  }

  // --- Draw bars ---
  for (int band = 0; band < nBands; band++) {
    int x = band * barWidth;
    float h = barHeight[band];

    // --- Smooth bar motion ---
    static float smoothHeight[128];
    smoothHeight[band] += (h - smoothHeight[band]) * 0.25f; // easing
    int height = (int)smoothHeight[band];

    // --- Peak hold logic ---
    if (height > peakHold[band])
      peakHold[band] = height;
    else
      peakHold[band] = std::max(peakHold[band] - 0.3, 0.0);

    // --- Dynamic glow + hue motion ---
    float hue = ((float)band / nBands) + 0.1f * sin(t + band * 0.2f);
    float saturation = 0.9f;
    float baseValue = 1.0f;
    float pulse = 0.5f + 0.5f * sin(t * 3.0f + band * 0.8f);

    for (int y = 0; y < height; y++) {
      float value = baseValue * (0.3f + 0.7f * (float)y / PANEL_RES_Y);
      if (band == peakBand) value = 1.0f; // bright peak

      uint16_t color = hsvToRgb(
        fmod(hue + 0.1f * sin(y * 0.05f + t * 2.0f), 1.0f),
        saturation,
        value * (0.8f + 0.2f * pulse)
      );

      int drawY = PANEL_RES_Y - 1 - y;
      for (int barX = x + spacing; barX < x + barWidth - spacing; barX++) {
        if (barX >= 0 && barX < PANEL_RES_X && drawY >= 0)
          dma_display->drawPixel(barX + 1, drawY, color);
      }
    }

    // --- Peak cap with shimmer ---
    int peakY = PANEL_RES_Y - 1 - (int)peakHold[band];
    if (peakY >= 0 && peakY < PANEL_RES_Y) {
      float flicker = 0.7f + 0.3f * sin(t * 10.0f + band);
      uint16_t capColor = dma_display->color565(
        (uint8_t)(255 * flicker),
        (uint8_t)(180 + 50 * flicker),
        (uint8_t)(80 + 40 * flicker)
      );

      for (int barX = x + spacing; barX < x + barWidth - spacing; barX++) {
        dma_display->drawPixel(barX + 1, peakY, capColor);
      }
    }

    // --- Optional glow halo above peak ---
    int glowY = peakY - 1;
    if (glowY >= 0 && glowY < PANEL_RES_Y) {
      uint16_t glowColor = dma_display->color565(255, 200, 150);
      for (int barX = x + spacing; barX < x + barWidth - spacing; barX++) {
        dma_display->drawPixel(barX + 1, glowY, glowColor);
      }
    }
  }

  // --- Optional animated bottom line ---
  for (int x = 0; x < PANEL_RES_X; x++) {
    float glow = 0.5f + 0.5f * sin(t * 4.0f + x * 0.1f);
    uint16_t baseLine = dma_display->color565(
      (uint8_t)(100 + 80 * glow),
      (uint8_t)(40 + 40 * glow),
      (uint8_t)(60 + 80 * glow)
    );
    dma_display->drawPixel(x, PANEL_RES_Y - 1, baseLine);
  }

  dma_display->flipDMABuffer();
}







 void drawFlameBars() {
  dma_display->clearScreen();

  int barWidth = PANEL_RES_X / nBands;
  int spacing = 1;
  float t = millis() * 0.002;

  for (int band = 0; band < nBands; band++) {
    int x = band * barWidth;
    int height = (int)barHeight[band];

    // Animate flame tip with sine wave
    float flameTipOffset = 4.0 * sin(t + band * 0.5);
    int animatedHeight = height + (int)flameTipOffset;

    // Update peak hold
    if (animatedHeight > peakHold[band]) {
      peakHold[band] = animatedHeight;
    } else {
      peakHold[band] = max(peakHold[band] - 0.4, 0.0);
    }

    for (int y = 0; y < animatedHeight; y++) {
      float normY = (float)y / PANEL_RES_Y;
      float flicker = 0.8 + 0.2 * sin(t * 3 + y * 0.3 + band);

      // Anime-style flame palette: white core → yellow → orange → red
      uint8_t r, g, b;
      if (normY < 0.2) {
        r = 255; g = 255; b = 200; // white-hot core
      } else if (normY < 0.5) {
        r = 255; g = 200; b = 50;  // yellow-orange
      } else {
        r = 255; g = 80; b = 30;   // red-orange
      }

      // Add flicker
      r = min(255, (int)(r * flicker));
      g = min(255, (int)(g * flicker));
      b = min(255, (int)(b * flicker));

      uint16_t flameColor = dma_display->color565(r, g, b);

      for (int barX = x + spacing; barX < x + barWidth - spacing; barX++) {
        int yPos = PANEL_RES_Y - 1 - y;
        if (barX >= 0 && barX < PANEL_RES_X && yPos >= 0) {
          dma_display->drawPixel(barX + 2, yPos, flameColor);
        }
      }
    }

    // Peak cap: glowing ember
    int peakY = PANEL_RES_Y - 1 - (int)peakHold[band];
    if (peakY >= 0 && peakY < PANEL_RES_Y) {
      float pulse = 0.5 + 0.5 * sin(t * 4 + band);
      uint16_t capColor = dma_display->color565(255, 160 + 40 * pulse, 40);
      for (int barX = x + spacing; barX < x + barWidth - spacing; barX++) {
        dma_display->drawPixel(barX + 2, peakY, capColor);
      }
    }
  }

 // dma_display->drawRect(0, 0, PANEL_RES_X, PANEL_RES_Y, dma_display->color565(random(0,255), random(0,255), random(0,255)));
  dma_display->flipDMABuffer();
}





 