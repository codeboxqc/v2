#include <Arduino.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h>
 

#include <bar.h>

// HUB75 setup
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>


 AudioFilter amplitudeFilter;

// Color Palette structure - ADD THIS
struct ColorPalette {
    uint16_t colors[5];
    const char* name;
};

// Color Palettes array - ADD THIS
static ColorPalette colorPalettes[] = {
    {{0xF800, 0xFC00, 0xFD20, 0xFE60, 0xFFE0}, "FIRE"}, 
    {{0x001F, 0x021F, 0x051F, 0x07FF, 0x05FF}, "OCEAN"},
    {{0x7800, 0xF800, 0xF810, 0xF81F, 0x781F}, "PLASMA"},
    {{0x07E0, 0x05E0, 0x04E0, 0x03E0, 0x02E0}, "MATRIX"},
    {{0xFD20, 0xFDC0, 0xFFE0, 0x87E0, 0x07FF}, "NEON"},
    {{0x8000, 0xF800, 0xFE60, 0x07E0, 0x001F}, "RAINBOW"},
    {{0x9810, 0xF810, 0xF800, 0xD000, 0x8000}, "VOLCANO"},
    {{0x001F, 0x021F, 0x051F, 0x07FF, 0xFFFF}, "ARCTIC"},
    {{0x0400, 0x0600, 0x07E0, 0x07F0, 0x07FF}, "FOREST"}
};

// Number of palettes - ADD THIS
static const int numPalettes = sizeof(colorPalettes) / sizeof(ColorPalette);


WaveData calculateWaveData(int x, float time) {
  WaveData data;
  
  float normalizedX = (float)x / PANEL_RES_X;
  int sampleIndex = (int)(normalizedX * SAMPLES);
  sampleIndex = constrain(sampleIndex, 0, SAMPLES - 1);
  
  data.amplitude = (float)samples[sampleIndex] / 32768.0f;
  data.filteredAmplitude = max(amplitudeFilter.process(fabs(data.amplitude)), 0.05f);
  
  // Color calculation (simplified from reusable.cpp)
  float colorIntensity = 0.5f + data.filteredAmplitude * 0.9f;
  colorIntensity = min(colorIntensity, 1.0f);
  
  int colorIndex = (x + (int)(time * 20)) % 5;
  uint16_t baseColor = 0xF800 + colorIndex * 0x0841;  // Example palette
  
  data.r = ((baseColor >> 11) & 0x1F) << 3;
  data.g = ((baseColor >> 5) & 0x3F) << 2;
  data.b = (baseColor & 0x1F) << 3;
  
  data.r = constrain(data.r * colorIntensity, 0, 255);
  data.g = constrain(data.g * colorIntensity, 0, 255);
  data.b = constrain(data.b * colorIntensity, 0, 255);
  
  data.y = (int)((1.0f - (0.5f + 0.3f * data.filteredAmplitude)) * PANEL_RES_Y);
  
  return data;
}

// Corrected calculateWaveData function using your existing ColorPalette
WaveData calculateWaveData2(int x, float time) {
    WaveData data;





    
    // Convert screen position to sample index
    data.normalizedX = (float)x / WIDTH;
    int sampleIndex = (int)(data.normalizedX * SAMPLES);
    sampleIndex = constrain(sampleIndex, 0, SAMPLES - 1);
    
    // Get raw amplitude
    float rawAmplitude = (float)samples[sampleIndex] / 32768.0f;
    data.amplitude = rawAmplitude;

 
    // Use envelope smoothing, then apply a minimum value
    float filtered = amplitudeFilter.process(fabs(rawAmplitude));
    data.filteredAmplitude = max(filtered, 0.05f);  // never z
    
    // Apply smoothing for less reactive visuals
    //data.filteredAmplitude = amplitudeFilter.process(fabs(rawAmplitude));
    
    // ADD THIS: Calculate Y position for waveform
    float waveformScale = 0.1f;
    float shimmer = 0.05f;
    float waveY = 0.5f + waveformScale * data.filteredAmplitude + shimmer * sin(time + data.normalizedX * 10.0f);
    data.y = (int)((1.0f - waveY) * PANEL_RES_Y);
    
    // Use your existing color palettes
    int paletteIndex = ((int)(time * 10) + x) % numPalettes;
    ColorPalette palette = colorPalettes[paletteIndex];

    float colorIntensity = 0.5f + data.filteredAmplitude * 0.9f; // stronger base brightness
    colorIntensity = min(colorIntensity, 1.0f);

    
    //float colorIntensity = 0.3f + data.filteredAmplitude * 0.7f;
    int colorIndex = (x + (int)(time * 20)) % 5;
    
    uint16_t baseColor = palette.colors[colorIndex];
    
    // Extract RGB components (convert from RGB565 to RGB888)
    data.r = ((baseColor >> 11) & 0x1F) << 3;
    data.g = ((baseColor >> 5) & 0x3F) << 2;
    data.b = (baseColor & 0x1F) << 3;
    
    // Modulate by amplitude
    data.r = constrain(data.r * colorIntensity, 0, 255);
    data.g = constrain(data.g * colorIntensity, 0, 255);
    data.b = constrain(data.b * colorIntensity, 0, 255);
    
    return data;
}




 

 

// NEW: Different visualization using the same wave data
void DRB() {
    dma_display->clearScreen();
    float t = millis() * 0.001; // Slower animation

    for (int x = 0; x < PANEL_RES_X; x++) {
        WaveData data = calculateWaveData(x, t);
        
        // Draw vertical bars instead of waveform
       // int barHeight = (int)(data.amplitude * PANEL_RES_Y * 0.2);
        int barHeight = max(2, (int)(fabs(data.amplitude) * PANEL_RES_Y * 0.2f));
     


        int barY = PANEL_RES_Y / 2 - barHeight / 2;
        
        for (int y = barY; y < barY + barHeight; y++) {
            if (y >= 0 && y < PANEL_RES_Y) {
               // float brightness = (float)(y - barY) / barHeight;
                float brightness = 0.6f + 0.4f * ((float)(y - barY) / barHeight);

                dma_display->drawPixelRGB888(x, y, 
                    data.r * brightness, 
                    data.g * brightness, 
                    data.b * brightness);
            }
        }
    }

    dma_display->flipDMABuffer();
}




// NEW: Particle visualization
void drawParticles() {
    dma_display->clearScreen();
    float t = millis() * 0.002;

    for (int x = 0; x < PANEL_RES_X; x++) {
        WaveData data = calculateWaveData(x, t);
        
        int py =0;
        // Draw particles that follow the waveform
       // for (int py = -1; py <= 0; py++) {
            int particleY = data.y + py;
            if (particleY >= 0 && particleY < PANEL_RES_Y) {
                float particleFade = 1.0 - abs(py) * 0.3;
                dma_display->drawPixelRGB888(x, particleY, 
                    data.r * particleFade, 
                    data.g * particleFade, 
                    data.b * particleFade);
          //  }
        }
    }

    dma_display->flipDMABuffer();
}






// Helper function for drawing interpolated lines
void drawLine(int x1, int y1, int x2, int y2, 
             uint8_t r1, uint8_t g1, uint8_t b1,
             uint8_t r2, uint8_t g2, uint8_t b2) {
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int steps = max(dx, dy);

    for (int i = 0; i <= steps; i++) {
        float ratio = (float)i / steps;
        int x = x1 + (x2 - x1) * ratio;
        int y = y1 + (y2 - y1) * ratio;
        
        uint8_t r = r1 + (r2 - r1) * ratio;
        uint8_t g = g1 + (g2 - g1) * ratio;
        uint8_t b = b1 + (b2 - b1) * ratio;

        // Glow effect
        int glow =0;
       // for (int glow = -1; glow <= 1; glow++) {
            int gx = x + glow;
            int gy = y + glow;
            if (gx >= 0 && gx < PANEL_RES_X && gy >= 0 && gy < PANEL_RES_Y) {
                float glowFade = 1.0 - fabs(glow) * 0.3;
                dma_display->drawPixelRGB888(gx, gy, r * glowFade, g * glowFade, b * glowFade);
          //  }
        }
    }
}


/////////////////////////////////////////
void drawCircularWaveformPure() {
    dma_display->clearScreen();
    float t = millis() * 0.002;

    int cx = PANEL_RES_X / 2;
    int cy = PANEL_RES_Y / 2;
    float baseRadius = min(PANEL_RES_X, PANEL_RES_Y) * 0.3;

    for (int i = 0; i < SAMPLES - 1; i += 2) {
        float angle1 = TWO_PI * i / SAMPLES;
        float angle2 = TWO_PI * (i + 2) / SAMPLES;

        // Use calculateWaveData for everything
        int virtualX1 = (int)((float)i / SAMPLES * PANEL_RES_X);
        int virtualX2 = (int)((float)(i + 2) / SAMPLES * PANEL_RES_X);
        WaveData data1 = calculateWaveData(virtualX1, t);
        WaveData data2 = calculateWaveData(virtualX2, t);

        // Simpler radius using only wave data
        float radius1 = baseRadius + data1.amplitude * 3;//15.0;
        float radius2 = baseRadius + data2.amplitude * 3; //15.0;

        int x1 = cx + (int)(radius1 * cos(angle1));
        int y1 = cy + (int)(radius1 * sin(angle1));
        int x2 = cx + (int)(radius2 * cos(angle2));
        int y2 = cy + (int)(radius2 * sin(angle2));

        // Draw line between points using wave data colors
        drawLine(x1, y1, x2, y2, data1.r, data1.g, data1.b, data2.r, data2.g, data2.b);
    }

    dma_display->flipDMABuffer();
}



 