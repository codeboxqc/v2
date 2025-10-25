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



 void drawWaveform2() {
  dma_display->clearScreen();
  float t = millis() * 0.002;

  int lastY = PANEL_RES_Y / 2;
  for (int x = 0; x < PANEL_RES_X; x++) {
    float normX = (float)x / PANEL_RES_X;
    int sampleIndex = (int)(normX * SAMPLES);
    sampleIndex = constrain(sampleIndex, 0, SAMPLES - 1);

    float amplitude = (float)samples[sampleIndex] / 32768.0;

    // Dynamic scaling with smoother amplitude response
    float waveformScale = 0.4 + 0.2 * fabs(amplitude);
    float shimmer = 0.08 * sin(t * 2.0 + normX * 8.0);
    float beatPulse = 0.1 * sin(t * 4.0) * fabs(amplitude);

    // Adaptive damping for strong beats (reduces edge clipping)
    float damping = 1.0 - 0.3 * fabs(amplitude);

    // Compute vertical position
    float waveY = 0.5 + waveformScale * amplitude * damping + shimmer + beatPulse;

    // --- Soft clip (tanh) instead of hard constrain ---
    if (waveY < 0.1)
      waveY = 0.1 + 0.05 * tanh((waveY - 0.1) * 10);
    else if (waveY > 0.9)
      waveY = 0.9 + 0.05 * tanh((waveY - 0.9) * 10);

    // Convert to pixel position
    int y = (int)((1.0 - waveY) * PANEL_RES_Y);
    y = constrain(y, 1, PANEL_RES_Y - 2);

    // Interpolate for smooth line segments
    int steps = abs(y - lastY);
    for (int i = 0; i <= steps; i++) {
      int interpY = lastY + (y - lastY) * i / max(1, steps);
      float fade = 1.0 - fabs(i - steps / 2) / (float)(steps + 1);

      // --- Edge fade (reduce brightness near screen edges) ---
      float edgeFade = 1.0;
      if (interpY < 5)
        edgeFade = interpY / 5.0;
      else if (interpY > PANEL_RES_Y - 6)
        edgeFade = (PANEL_RES_Y - 1 - interpY) / 5.0;

      float totalFade = fade * edgeFade;

      // Neon color scheme
      float colorT = t * 0.8 + normX * 5.0;
      uint8_t r = 128 + 127 * sin(colorT + fabs(amplitude));
      uint8_t g = 128 + 127 * sin(colorT + TWO_PI / 3);
      uint8_t b = 128 + 127 * sin(colorT + 2 * TWO_PI / 3);

      // Beat color boost
      if (fabs(amplitude) > 0.5) {
        r = min(255, r + 60);
        g = min(255, g + 40);
        b = min(255, b + 80);
      }

      // Glow effect
      int glow = 1;
      for (int dy = -glow; dy <= glow; dy++) {
        int yOffset = interpY + dy;
        if (yOffset >= 1 && yOffset < PANEL_RES_Y - 1) {
          float glowFade = 1.0 - fabs(dy) * 0.5;
          dma_display->drawPixelRGB888(
            x, yOffset,
            (uint8_t)(r * glowFade * totalFade),
            (uint8_t)(g * glowFade * totalFade),
            (uint8_t)(b * glowFade * totalFade)
          );
        }
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

  // Increased steps for smoother interpolation
  const int INTERPOLATION_STEPS = 4; // Draw 4 sub-points between each sample
  for (int i = 0; i < SAMPLES - 5; i += 1) {
    float angle1 = TWO_PI * i / SAMPLES;
    float angle2 = TWO_PI * (i + 1) / SAMPLES; // Connect to next point for continuity

    float amp1 = (float)samples[i] / 32768.0;
    float amp2 = (float)samples[i + 1] / 32768.0;

    // Interpolate between points for smoother curves
    for (int step = 0; step < INTERPOLATION_STEPS; step++) {
      float tInterp = (float)step / INTERPOLATION_STEPS;
      float angle = angle1 + (angle2 - angle1) * tInterp;
      float amp = amp1 + (amp2 - amp1) * tInterp;

      // Dynamic radius with time-based oscillation
      float radius = baseRadius + amp * 12.0 + 3.0 * sin(t + i * 0.02);

      int x1 = cx + (int)(radius * cos(angle));
      int y1 = cy + (int)(radius * sin(angle));

      // Plasma-like color palette using sine waves for smooth transitions
      uint8_t r = 128 + 127 * sin(t * 0.5 + i * 0.05);
      uint8_t g = 128 + 127 * sin(t * 0.3 + i * 0.07 + TWO_PI / 3);
      uint8_t b = 128 + 127 * sin(t * 0.4 + i * 0.09 + 2 * TWO_PI / 3);

      // Draw with a slight glow effect by rendering nearby pixels
      int glow = 1;
      for (int dx = -glow; dx <= glow; dx++) {
        for (int dy = -glow; dy <= glow; dy++) {
          int gx = x1 + dx;
          int gy = y1 + dy;
          if (gx >= 0 && gx < PANEL_RES_X && gy >= 0 && gy < PANEL_RES_Y) {
            // Fade intensity for glow effect
            uint8_t intensity = 255 - 80 * (abs(dx) + abs(dy));
            dma_display->drawPixelRGB888(gx, gy, r * intensity / 255, g * intensity / 255, b * intensity / 255);
          }
        }
      }

      // Draw line to the next interpolated point for a continuous curve
      if (step < INTERPOLATION_STEPS - 1 || i < SAMPLES - 6) {
        float nextTInterp = (float)(step + 1) / INTERPOLATION_STEPS;
        float nextAngle = angle1 + (angle2 - angle1) * nextTInterp;
        float nextAmp = amp1 + (amp2 - amp1) * nextTInterp;
        float nextRadius = baseRadius + nextAmp * 12.0 + 3.0 * sin(t + (i + nextTInterp) * 0.02);

        int x2 = cx + (int)(nextRadius * cos(nextAngle));
        int y2 = cy + (int)(nextRadius * sin(nextAngle));

        // Draw line between points for smoother curves
        dma_display->drawLine(x1, y1, x2, y2, dma_display->color565(r, g, b));
      }
    }
  }

  dma_display->flipDMABuffer();
}
 

 
void drawCircularWaveform2() {
  dma_display->clearScreen();
  float t = millis() * 0.002;

  int cx = PANEL_RES_X / 2;
  int cy = PANEL_RES_Y / 2;
  float baseRadius = min(PANEL_RES_X, PANEL_RES_Y) * 0.3;

  // FIX: Change to SAMPLES - 5 to avoid accessing samples[i + 4] out of bounds
  int i = 0;
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
 

