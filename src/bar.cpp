#include <Arduino.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h>
 

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

  dma_display->drawRect(0, 0, PANEL_RES_X, PANEL_RES_Y, dma_display->color565(255, 80, 0));
  dma_display->flipDMABuffer();
}





 