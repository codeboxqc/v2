#include <Arduino.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h>
 

#include <bar.h>

// HUB75 setup
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>






void drawWaveform() {
  dma_display->clearScreen();
  float t = millis() * 0.002;

  int lastY = PANEL_RES_Y / 2;
  for (int x = 0; x < PANEL_RES_X; x++) {
    float normX = (float)x / PANEL_RES_X;
    int sampleIndex = (int)(normX * SAMPLES);
    sampleIndex = constrain(sampleIndex, 0, SAMPLES - 1);

    float amplitude = (float)samples[sampleIndex] / 32768.0;
    
    //float waveY = 0.5 + 0.2 * amplitude + 0.05 * sin(t + normX * 10.0);   

    float waveformScale = 0.3; // try 0.1 to 0.3
    float shimmer = 0.05;
    float waveY = 0.5 + waveformScale * amplitude + shimmer * sin(t + normX * 10.0);

  

    int y = (int)((1.0 - waveY) * PANEL_RES_Y);

    // Interpolate between lastY and y for smoother curve
    int steps = abs(y - lastY);
    for (int i = 0; i <= steps; i++) {
      int interpY = lastY + (y - lastY) * i / max(1, steps);
      float fade = 1.0 - fabs(i - steps / 2) / (float)(steps + 1);

      uint8_t r = 80 + 100 * fabs(amplitude);
      uint8_t g = 180 + 75 * fade;
      uint8_t b = 255;

      int glow =0;
     // for (int glow = -1; glow <= 1; glow++) {
        int yOffset = interpY + glow;
        if (yOffset >= 0 && yOffset < PANEL_RES_Y) {
          float glowFade = 1.0 - fabs(glow) * 0.4;
          dma_display->drawPixelRGB888(x, yOffset, r * glowFade, g * glowFade, b * glowFade);
      //  }
      }
    }

    lastY = y;
  }

  dma_display->flipDMABuffer();
}






void drawCircularWaveform() {
  dma_display->clearScreen();
  float t = millis() * 0.002;

  int cx = PANEL_RES_X / 2;
  int cy = PANEL_RES_Y / 2;
  float baseRadius = min(PANEL_RES_X, PANEL_RES_Y) * 0.3;

  // FIX: Change to SAMPLES - 5 to avoid accessing samples[i + 4] out of bounds
  for (int i = 0; i < SAMPLES - 5; i += 1) {
    float angle1 = TWO_PI * i / SAMPLES;
    float angle2 = TWO_PI * (i + 4) / SAMPLES;

    float amp1 = (float)samples[i] / 32768.0;
    float amp2 = (float)samples[i + 4] / 32768.0;

    float radius1 = baseRadius + amp1 * 10.0 + 2.0 * sin(t + i * 0.01);
    float radius2 = baseRadius + amp2 * 10.0 + 2.0 * sin(t + (i + 4) * 0.01);

    int x1 = cx + (int)(radius1 * cos(angle1));
    int y1 = cy + (int)(radius1 * sin(angle1));
    int x2 = cx + (int)(radius2 * cos(angle2));
    int y2 = cy + (int)(radius2 * sin(angle2));

    int glow = 1;
    int gx1 = x1 + glow;
    int gy1 = y1 + glow;
    int gx2 = x2 + glow;
    int gy2 = y2 + glow;

    uint8_t r = 255;
    uint8_t g = 100 + 100 * fabs(amp1);
    uint8_t b = 200 + 55 * fabs(amp2);

    if (gx1 >= 0 && gx1 < PANEL_RES_X && gy1 >= 0 && gy1 < PANEL_RES_Y)
      dma_display->drawPixelRGB888(gx1, gy1, r, g, b);
    if (gx2 >= 0 && gx2 < PANEL_RES_X && gy2 >= 0 && gy2 < PANEL_RES_Y)
      dma_display->drawPixelRGB888(gx2, gy2, r, g, b);
  }

  dma_display->flipDMABuffer();
}


