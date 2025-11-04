#include <Arduino.h>
#include <cmath>
#include <algorithm>
#include <string.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
 
#include "bar.h" 
// Display dimensions - adjust for your LED matrix

#define CENTER_X (WIDTH/2)
#define CENTER_Y (HEIGHT/2)

// Animation parameters
static float t = 0.0f;
static int currentCurve = random(0, 26) ;
static int nextCurve = random(0, 26) ;
static float transitionProgress = 0.0f;
static unsigned long lastChangeTime = 0;
static const float animationSpeed = 0.07f;
static const int maxRadius = min(WIDTH, HEIGHT) * 0.40f;


 
struct ColorPalette {
    uint16_t colors[5];
    const char* name;
};

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

 

 uint16_t fastRGB565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static const int numPalettes = sizeof(colorPalettes) / sizeof(ColorPalette);

  
 

 float fastSin(float x) {
    // Wrap x to [0, 2*PI] - much faster than fmod
    x = x - 6.283185307f * floor(x / 6.283185307f);
    
    // Cubic approximation for sine
    if (x < 3.141592654f) {
        return (4.0f * x * (3.141592654f - x)) / (9.869604401f);
    } else {
        x = 6.283185307f - x;
        return -(4.0f * x * (3.141592654f - x)) / (9.869604401f);
    }
}


 float fastCos(float x) {
    return fastSin(x + PI/2);
}

// HSV to RGB565 conversion
uint16_t hsvToRgb565(float h, float s, float v) {
    h = fmod(h, 360.0f);
    if (h < 0) h += 360.0f;
    
    float c = v * s;
    float x = c * (1.0f - fabs(fmod(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    
    float r, g, b;
    
    if (h < 60) { r = c; g = x; b = 0; }
    else if (h < 120) { r = x; g = c; b = 0; }
    else if (h < 180) { r = 0; g = c; b = x; }
    else if (h < 240) { r = 0; g = x; b = c; }
    else if (h < 300) { r = x; g = 0; b = c; }
    else { r = c; g = 0; b = x; }
    
    r = (r + m) * 255;
    g = (g + m) * 255;
    b = (b + m) * 255;
    
    return fastRGB565((uint8_t)r, (uint8_t)g, (uint8_t)b);
}



// Simple map function
float maper(float value, float inMin, float inMax, float outMin, float outMax) {
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// Constrain function
float constraint(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Plasma color generation
uint16_t getPlasmaColor(float x, float y, float time, int effectType) {
    float scale = 0.1f;
    x *= scale;
    y *= scale;
    time *= 0.1f;
    
    float val = 0.0f;
    
    switch(effectType % 6) {
    case 0: { // Classic plasma
        val = fastSin(x + time) + fastSin(y + time) + 
              fastSin((x + y + time) * 0.5f) + 
              fastSin(sqrt(x*x + y*y) + time);
        val = (val + 4.0f) / 8.0f;
        return hsvToRgb565(val * 360.0f + time * 50.0f, 0.9f, 0.9f);
    }

    case 1: { // Fire effect
        val = fastSin(x * 2.0f + time) + fastSin(y * 1.5f - time) + 
              fastSin((x - y) * 0.8f + time * 1.2f);
        val = (val + 3.0f) / 6.0f;
        float fireHue = 15.0f + val * 30.0f;
        return hsvToRgb565(fireHue, 0.9f, val * 0.8f + 0.2f);
    }

    case 2: { // Electric/blue plasma
        val = fastSin(x * 3.0f + time) * fastCos(y * 2.0f - time) + 
              fastSin((x + y) * 1.2f + time * 0.5f);
        val = (val + 2.0f) / 4.0f;
        return hsvToRgb565(200.0f + val * 100.0f, 0.8f, 0.9f);
    }

    case 3: { // Green matrix
        val = fastSin(x * 4.0f + time * 0.3f) + fastCos(y * 3.0f + time * 0.2f) +
              fastSin((x - y) * 2.0f + time * 0.4f);
        val = fabs(val) * 0.7f;
        return hsvToRgb565(120.0f, 0.9f, val);
    }

    case 4: { // Rainbow spiral
        float angle = atan2(y - CENTER_Y * scale, x - CENTER_X * scale);
        float dist = sqrt((x - CENTER_X * scale) * (x - CENTER_X * scale) + 
                         (y - CENTER_Y * scale) * (y - CENTER_Y * scale));
        val = fastSin(angle * 5.0f + dist * 10.0f + time);
        return hsvToRgb565(fmod(angle * 57.3f + time * 50.0f + dist * 100.0f, 360.0f), 0.9f, 0.9f);
    }

    case 5: { // Purple cosmic
        val = fastSin(x * 1.5f + time) + cos(y * 1.3f - time) +
              fastSin(sqrt(x*x + y*y) * 2.0f + time * 0.7f);
        val = (val + 3.0f) / 6.0f;
        return hsvToRgb565(280.0f + val * 50.0f, 0.8f, 0.8f);
    }
}

    return hsvToRgb565(val * 360.0f, 0.8f, 0.9f);
}


// Calculate parametric curve point
void calculateCurvePoint(float theta, float radius, int curveType, float *outX, float *outY) {
    float x = 0, y = 0;
    float tFactor = t * 0.8f;
    float r=0.0f;
    
    switch(curveType % 24) {
        case 0: // Rose curve
            x = radius * fastCos(3 * theta + t) * fastCos(theta);
            y = radius * fastSin(3 * theta + t) * fastSin(theta);
            break;
            
        case 1: // Spiral
            {
                float spiral = radius * (0.6f + 0.3f * sin(theta/8 + t));
                x = spiral * fastCos(theta) + radius/5 * fastCos(9 * theta + t);
                y = spiral * fastSin(theta) - radius/5 * fastSin(7 * theta - t);
            }
            break;
            
        case 2: // Lissajous
            x = radius * fastSin(theta * 2 + t) * fastCos(theta * 3 - t/2);
            y = radius * fastCos(theta * 3 - t) * fastSin(theta * 2 + t/3);
            break;
            
        case 3: // Modulated circle
            x = radius * fastCos(theta) * (1 + 0.25f * fastSin(7 * theta + t));
            y = radius * fastSin(theta) * (1 + 0.25f * fastCos(5 * theta - t));
            break;
            
        case 4: // Vortex
            {
                float vortex = radius * (0.3f + 0.5f * pow(fastSin(theta/2 + t), 2));
                x = vortex * fastCos(theta + 5 * fastSin(theta/3 + t/4));
                y = vortex * fastSin(theta + 5 * fastCos(theta/4 - t/5));
            }
            break;
            
        case 5: // Complex wave
            x = radius * fastSin(3 * theta + t) * fastCos(2 * theta - t);
            y = radius * fastCos(4 * theta - t) * fastSin(5 * theta + t);
            break;
            
        case 6: // Branching pattern
            {
                float branch = radius * 0.6f;
                x = branch * fastCos(theta) * (1 + 0.3f * fastSin(13 * theta + t));
                y = branch * fastSin(theta) * (1 + 0.3f * fastCos(11 * theta - t));
            }
            break;
            
        case 7: // Orbital motion
            {
                float orbital = radius * (0.7f + 0.2f * fastSin(theta * 2 + t));
                x = orbital * fastCos(theta + fastSin(theta * 7 + t));
                y = orbital * fastSin(theta + fastCos(theta * 6 - t));
            }
            break;
            
        case 8: // Hypocycloid
            {
                float a = radius * 0.6f, b = radius * 0.2f;
                x = (a + b) * fastCos(theta) - b * fastCos((a/b + 1) * theta + t);
                y = (a + b) * fastSin(theta) - b * fastSin((a/b + 1) * theta + t);
            }
            break;
            
        case 9: // Epicycloid
            {
                float c = radius * 0.7f, d = radius * 0.175f;
                x = (c - d) * fastCos(theta) + d * fastCos((c/d - 1) * theta - t);
                y = (c - d) * fastSin(theta) - d * fastSin((c/d - 1) * theta - t);
            }
            break;
            
        case 10: // Butterfly curve
            {
                float r = radius * 0.5f * exp(fastCos(theta + t)) - 2 * fastCos(4 * theta) - pow(fastSin(theta/12), 5);
                x = r * fastSin(theta);
                y = r * fastCos(theta);
            }
            break;
            
        case 11: // Cardioid
            {
                float card = radius * (1 + fastCos(theta + t));
                x = card * fastCos(theta);
                y = card * fastSin(theta);
            }
            break;
            
        case 12: // Limacon
            {
                float lem = radius * (0.5f + fastCos(theta + t));
                x = lem * fastCos(theta);
                y = lem * fastSin(theta);
            }
            break;
            
        case 13: // Astroid
            x = radius * pow(fastCos(theta + t), 3);
            y = radius * pow(fastSin(theta + t), 3);
            break;
            
        case 14: // Cycloid
            {
                float a = radius * 0.5f;
                x = a * (theta - fastSin(theta + t));
                y = a * (1 - fastCos(theta + t));
            }
            break;
            
        case 15: // Trochoid
            {
                float a = radius * 0.6f, b = radius * 0.4f;
                x = a * theta - b * fastSin(theta + t);
                y = a - b * fastCos(theta + t);
            }
            break;
            
        case 16: // Folium
            {
                float denom = 1 + pow(tan(theta + t), 3);
                if (abs(denom) > 0.01f) {
                    x = radius * 3 * fastCos(theta) / denom;
                    y = radius * 3 * fastSin(theta) * fastCos(theta + t) / denom;
                }
            }
            break;
            
        case 17: // Logarithmic spiral
            {
                float r = radius * 0.3f * exp(0.1f * theta);
                x = r * fastCos(theta + t);
                y = r * fastSin(theta + t);
            }
            break;
            
        case 18: // Archimedean spiral
            {
                float r = radius * 0.02f * theta;
                x = r * fastCos(theta + t);
                y = r * fastSin(theta + t);
            }
            break;

            case 19: // Nephroid
            {
                float a = radius * 0.3f;
                x = a * 3 * fastCos(theta) - a * fastCos(3 * theta + t);
                y = a * 3 * fastSin(theta) - a * fastSin(3 * theta + t);
            }
            break;
            
        case 20: // Deltoid
            {
                float a = radius * 0.4f;
                x = a * 2 * fastCos(theta) + a * fastCos(2 * theta + t);
                y = a * 2 * fastSin(theta) - a * fastSin(2 * theta + t);
            }
            break;
            
        case 21: // Epitrochoid
            {
                float R = radius * 0.4f, r = radius * 0.2f, d = radius * 0.3f;
                x = (R + r) * fastCos(theta) - d * fastCos((R + r) * theta / r + t);
                y = (R + r) * fastSin(theta) - d * fastSin((R + r) * theta / r + t);
            }
            break;
            
        case 22: // Hypotrochoid
            {
                float R = radius * 0.5f, r = radius * 0.15f, d = radius * 0.25f;
                x = (R - r) * fastCos(theta) + d * fastCos((R - r) * theta / r - t);
                y = (R - r) * fastSin(theta) - d * fastSin((R - r) * theta / r - t);
            }
            break;
            
        case 23: // Cissoid
            {
                float a = radius * 0.4f;
                float denom = 1 - fastCos(theta + t);
                if (abs(denom) > 0.01f) {
                    x = a * 2 * fastSin(theta) * fastSin(theta + t) / denom;
                    y = a * 2 * fastSin(theta) * fastSin(theta) * fastSin(theta + t) / denom;
                }
            }
            break;
            
        case 24: // Conchoid
            {
                float a = radius * 0.3f, b = radius * 0.4f;
                x = a + b * fastCos(theta + t);
                y = a * tan(theta + t) + b * fastSin(theta + t);
            }
            break;
            
        case 25: // Kappa curve
            {
                float a = radius * 0.5f;
                x = a * fastCos(theta + t) * tan(theta);
                y = a * fastSin(theta + t);
            }
            break;
      
            
       
            
         
            
        default:
            r = radius * (0.6f + 0.4f * sin(t * 0.9f + theta * 1.7f));
            x = r * fastCos(theta) * (1.0f + 0.3f * fastSin(t * 1.2f + theta * 3.0f));
            y = r * fastSin(theta) * (1.0f + 0.3f * fastCos(t * 1.2f + theta * 3.0f));
            break;
    }
    
    *outX = x;
    *outY = y;
}


///////////////////////////////////////////////////////
 





///////////////////////////////////////////////////////
 
/*
void drawCurve(int curveType, float alpha) {
    const int resolution = 180;
    const float radius = maxRadius * 1.0f;
    
    // Select palette based on curve type and time
    ColorPalette palette = colorPalettes[(curveType + (int)(t * 0.5f)) % numPalettes];
    int effectType = (curveType + (int)t) % 6;
    
    float prevX = -1, prevY = -1;
    
    for (int i = 0; i <= resolution; i++) {
        float theta = maper(i, 0, resolution, 0, TWO_PI);
        
        float x, y;
        calculateCurvePoint(theta, radius, curveType, &x, &y);
        
        int screenX = (int)(CENTER_X + x);
        int screenY = (int)(CENTER_Y + y);
        
        if (screenX >= 0 && screenX < WIDTH && screenY >= 0 && screenY < HEIGHT) {
            // Choose color based on effect type
            uint16_t color;
            if (effectType == 0) {
                // Use palette colors with cycling
                int colorIndex = (i + (int)(t * 20)) % 5;
                color = palette.colors[colorIndex];
            } else {
                // Use plasma/fire effects
                color = getPlasmaColor(screenX, screenY, t, effectType);
            }
            
            // Apply alpha blending
            uint8_t r = ((color >> 11) & 0x1F) * alpha;
            uint8_t g = ((color >> 5) & 0x3F) * alpha;
            uint8_t b = (color & 0x1F) * alpha;
            color = (r << 11) | (g << 5) | b;
            
            // Draw with different styles
            if (prevX >= 0 && prevY >= 0) {
                // Main curve line
                dma_display->drawLine((int)prevX, (int)prevY, screenX, screenY, color);
                
                // Add glow effect for certain curves
                if (curveType % 4 == 0 && alpha > 0.7f) {
                    uint16_t glowColor = hsvToRgb565(fmod(t * 100.0f, 360.0f), 0.5f, alpha * 0.3f);
                    dma_display->drawPixel((int)prevX + 1, (int)prevY, glowColor);
                    dma_display->drawPixel((int)prevX - 1, (int)prevY, glowColor);
                    dma_display->drawPixel((int)prevX, (int)prevY + 1, glowColor);
                    dma_display->drawPixel((int)prevX, (int)prevY - 1, glowColor);
                }
            }
            
            // Add particles along the curve for high-alpha curves
            if (alpha > 0.8f && i % 15 == 0) {
                float particleSize = 1.0f + sin(t * 3.0f + i) * 0.5f;
                uint16_t particleColor = hsvToRgb565(fmod(t * 80.0f + i * 10.0f, 360.0f), 0.9f, alpha);
                dma_display->fillCircle(screenX, screenY, (int)particleSize, particleColor);
            }
            
            prevX = screenX;
            prevY = screenY;
        }
    }
}
  
*/
 

 

//////////////////////////////////////////////////////////////////
void drawCurve(int curveType, float alpha) {
    const int resolution = 90;
    const float radius = maxRadius * 1.0f;
    
    // Select palette based on curve type and time
    ColorPalette palette = colorPalettes[(curveType + (int)(t * 0.5f)) % numPalettes];
    int effectType = (curveType + (int)t) % 6;
    
    float prevX = -1, prevY = -1;
    
    for (int i = 0; i <= resolution; i++) {
        float theta = maper(i, 0, resolution, 0, TWO_PI);

        // Better audio mapping - use screen position for audio sampling
        int audioX = (int)((float)i / resolution * WIDTH);
        WaveData audioData = calculateWaveData(audioX, t);
        
        // Modulate radius with audio
        float audioRadius = radius * (1.0f + audioData.filteredAmplitude * 0.3f);
        
        float x, y;
        calculateCurvePoint(theta, audioRadius, curveType, &x, &y);
        
        // METHOD 1: Direct position displacement based on audio
         /*
        float audioDisplacementX = (audioData.filteredAmplitude - 0.5f) * 10.0f; // -5 to +5 pixels
        float audioDisplacementY = (audioData.r - 128.0f) / 25.0f; // Use color channels for variation
        
        x += audioDisplacementX;
        y += audioDisplacementY;
        */
        
        /*
        // METHOD 2: Angular displacement (pushes points along their normal)
        float angle = atan2(y, x); // Direction from center
        float radialPush = audioData.filteredAmplitude * 8.0f; // Push outward
        x += radialPush * fastCos(angle);
        y += radialPush * fastSin(angle);
        */

        //*
        // METHOD 3: Wave-like distortion
        float waveDistortionX = audioData.filteredAmplitude * fastSin(theta * 8.0f + t) * 6.0f;
        float waveDistortionY = audioData.filteredAmplitude * fastCos(theta * 6.0f - t) * 6.0f;
         x += waveDistortionX;
         y += waveDistortionY;
         //*/
        
        // METHOD 4: Pulse effect at specific frequencies
        /*
        if (audioData.filteredAmplitude > 0.7f) {
            float pulse = (audioData.filteredAmplitude - 0.7f) * 20.0f;
            float pulseAngle = t * 2.0f + i * 0.1f;
            x += pulse * fastCos(pulseAngle);
            y += pulse * fastSin(pulseAngle);
        }
          */
        

         /*
        // METHOD 5: Smooth noise-like displacement
        float noiseX = fastSin(theta * 12.0f + t * 3.0f) * audioData.filteredAmplitude * 4.0f;
        float noiseY = fastCos(theta * 10.0f - t * 2.0f) * audioData.filteredAmplitude * 4.0f;
        x += noiseX;
        y += noiseY;
         */


        /*
        // METHOD 6: Color-channel based displacement
        float colorDisplacementX = (audioData.r - 128.0f) / 20.0f;
        float colorDisplacementY = (audioData.g - 128.0f) / 20.0f;
        x += colorDisplacementX;
        y += colorDisplacementY;
        */

        int screenX = (int)(CENTER_X + x);
        int screenY = (int)(CENTER_Y + y);
        
        if (screenX >= 0 && screenX < WIDTH && screenY >= 0 && screenY < HEIGHT) {
            // Choose color based on effect type
            uint16_t color;
            if (effectType == 0) {
                // Use palette colors with cycling
                int colorIndex = (i + (int)(t * 20)) % 5;
                color = palette.colors[colorIndex];
            } else {
                // Use plasma/fire effects
                color = getPlasmaColor(screenX, screenY, t, effectType);
            }
            
            // Apply alpha blending
            uint8_t r = ((color >> 11) & 0x1F) * alpha;
            uint8_t g = ((color >> 5) & 0x3F) * alpha;
            uint8_t b = (color & 0x1F) * alpha;
            color = (r << 11) | (g << 5) | b;
            
            // Draw with different styles
            if (prevX >= 0 && prevY >= 0) {
                // Main curve line
                dma_display->drawLine((int)prevX, (int)prevY, screenX, screenY, color);
                
                // Add glow effect for certain curves (audio enhanced)
                if (curveType % 4 == 0 && alpha > 0.7f && audioData.filteredAmplitude > 0.3f) {
                    uint16_t glowColor = hsvToRgb565(
                        fmod(t * 100.0f + audioData.filteredAmplitude * 200.0f, 360.0f), 
                        0.5f, 
                        alpha * audioData.filteredAmplitude * 0.5f
                    );
                  //  dma_display->drawPixel((int)prevX + 1, (int)prevY, glowColor);
                   // dma_display->drawPixel((int)prevX - 1, (int)prevY, glowColor);
                    dma_display->drawPixel((int)prevX, (int)prevY + 1, glowColor);
                   // dma_display->drawPixel((int)prevX, (int)prevY - 1, glowColor);
                }
            }
            
            
            prevX = screenX;
            prevY = screenY;
        }
    }
}
/////////////////////////////////////////////////////////////////// 
  

 
// Main animation update
void updateSupercharged(int charge) {
    unsigned long currentTime = millis();
    
    // Clear buffer
    dma_display->clearScreen();
    
  
    
    // Handle curve transitions
    if (currentTime - lastChangeTime > 12000) {
        transitionProgress += 0.015f;
        if (transitionProgress >= 1.0f) {
            transitionProgress = 0.0f;
            currentCurve = nextCurve;
            nextCurve =  random(0, 26) ;
            lastChangeTime = currentTime;
        }
    }
    
    // Draw curves with smooth transition
    if (transitionProgress > 0.0f) {
       // drawCurve(currentCurve, 1.0f - transitionProgress);
        drawCurve(nextCurve, transitionProgress);
    } else {
        drawCurve(currentCurve, 1.0f);
    }
    
    // Draw secondary curve layers for depth
    for (int layer = 1; layer <= 2; layer++) {
        int layerCurve = (currentCurve + layer * 3) % 25;
        float layerAlpha = 0.4f / layer;
        drawCurve(layerCurve, layerAlpha);
    }
    
    // Update animation time
    t += animationSpeed;
    if (t > 1000.0f) t = 0.0f;
}





// ============================================================================
// FUNCTION 1: Quantum Field Visualization with Audio Morphing
// ============================================================================
void Quantum() {
 
    static float t = 0.0f;
    static float hueShift = 0.0f;

   // dma_display->clearScreen();

    // --- 1. Compute smoothed energy (overall + bass emphasis) ---
    float globalEnergy = 0.0f;
    float bassEnergy = 0.0f;

    for (int i = 0; i < SAMPLES; i += 4) {
        float s = fabs((float)samples[i]) / 32768.0f;
        globalEnergy += s;
        if (i < SAMPLES / 8) bassEnergy += s;  // low frequencies
    }

    globalEnergy /= (SAMPLES / 4);
    bassEnergy /= (SAMPLES / 8);

    static float smoothGlobal = 0.0f, smoothBass = 0.0f;
    smoothGlobal = 0.85f * smoothGlobal + 0.15f * globalEnergy;
    smoothBass = 0.85f * smoothBass + 0.15f * bassEnergy;

    // --- 2. Animate based on sound energy ---
    hueShift += 1.5f + smoothGlobal * 50.0f;          // faster hue spin with sound
    float waveSpeed = 1.0f + smoothBass * 3.0f;       // faster wave animation on bass
    float fieldIntensity = 0.8f + smoothGlobal * 1.5f; // stronger field amplitude on peaks

    // --- 3. Precompute sin/cos across X for efficiency ---
    float sinTable[WIDTH];
    float cosTable[WIDTH];
    for (int x = 0; x < WIDTH; x++) {
        float v = (x - WIDTH / 2) * 0.12f;
        sinTable[x] = sinf(v + t * waveSpeed);
        cosTable[x] = cosf(v - t * 0.8f * waveSpeed);
    }

    // --- 4. Draw field pattern ---
    for (int y = 0; y < HEIGHT; y++) {
        float ny = (y - HEIGHT / 2) * 0.12f;
        float sinY = sinf(ny + t * 0.8f * waveSpeed);
        float cosY = cosf(ny - t * 1.1f * waveSpeed);

        for (int x = 0; x < WIDTH; x++) {
            float field = (sinTable[x] * cosY + cosTable[x] * sinY) * fieldIntensity;
            field = (field + 1.0f) * 0.5f; // normalize 0..1

            // --- Audio-reactive brightness ---
            float brightness = field * (0.5f + smoothGlobal * 1.0f);

            // --- Audio-reactive hue shift ---
            float hue = fmod(hueShift + field * 240.0f + smoothBass * 120.0f, 360.0f);

            // --- Simplified HSV→RGB ---
            uint8_t r, g, b;
            if (hue < 120) {
                r = (uint8_t)(255 * (1.0f - hue / 120.0f));
                g = (uint8_t)(255 * (hue / 120.0f));
                b = 0;
            } else if (hue < 240) {
                hue -= 120;
                r = 0;
                g = (uint8_t)(255 * (1.0f - hue / 120.0f));
                b = (uint8_t)(255 * (hue / 120.0f));
            } else {
                hue -= 240;
                r = (uint8_t)(255 * (hue / 120.0f));
                g = 0;
                b = (uint8_t)(255 * (1.0f - hue / 120.0f));
            }

            r = (uint8_t)(r * brightness);
            g = (uint8_t)(g * brightness);
            b = (uint8_t)(b * brightness);

            dma_display->drawPixelRGB888(x, y, r, g, b);
        }
    }

    t += 0.05f * waveSpeed;
    dma_display->flipDMABuffer();
 
}


void QuantumPlasmaReactive() {
    static float t = 0.0f;
    static float hueShift = 0.0f;

   // dma_display->clearScreen();

    // --- 1. Audio energy analysis ---
    float bassEnergy = 0.0f;
    float midEnergy = 0.0f;
    float highEnergy = 0.0f;

    int third = SAMPLES / 3;
    for (int i = 0; i < third; i++) bassEnergy += fabs((float)samples[i] / 32768.0f);
    for (int i = third; i < 2 * third; i++) midEnergy += fabs((float)samples[i] / 32768.0f);
    for (int i = 2 * third; i < SAMPLES; i++) highEnergy += fabs((float)samples[i] / 32768.0f);

    bassEnergy /= third;
    midEnergy /= third;
    highEnergy /= third;

    static float smoothBass = 0, smoothMid = 0, smoothHigh = 0;
    smoothBass = 0.8f * smoothBass + 0.2f * bassEnergy;
    smoothMid = 0.85f * smoothMid + 0.15f * midEnergy;
    smoothHigh = 0.9f * smoothHigh + 0.1f * highEnergy;

    // --- 2. Global animation parameters ---
    hueShift += 0.8f + smoothHigh * 25.0f;
    float plasmaSpeed = 0.5f + smoothBass * 3.0f;
    float intensity = 0.7f + (smoothBass + smoothMid) * 1.3f;

    // --- 3. Fast plasma field ---
    for (int y = 0; y < HEIGHT; y++) {
        float fy = (y - HEIGHT / 2) * 0.15f;
        for (int x = 0; x < WIDTH; x++) {
            float fx = (x - WIDTH / 2) * 0.15f;

            float v = 0.0f;
            v += sinf(fx * 3.1f + t * plasmaSpeed);
            v += sinf((fy + t * 0.7f) * 4.3f);
            v += sinf((fx + fy) * 2.5f - t * 1.3f);
            v += cosf(sqrtf(fx * fx + fy * fy) * 4.0f - t * plasmaSpeed * 1.2f);

            // local reaction — pixel flicker by amplitude in this zone
            int idx = ((x + y * WIDTH) % SAMPLES);
            float localAmp = fabs((float)samples[idx]) / 32768.0f;
            v += localAmp * 3.0f;

            v *= intensity;

            // Normalize field 0..1
            float field = (v + 4.0f) / 8.0f;
            field = constrain(field, 0.0f, 1.0f);

            // --- 4. Fire/Ice color mix ---
            float fireHue = 30.0f + field * 60.0f;   // orange/yellow
            float iceHue = 200.0f + field * 80.0f;   // cyan/blue
            float mix = smoothMid * 1.5f;            // mid frequencies switch color temperature
            mix = constrain(mix, 0.0f, 1.0f);
            float hue = fireHue * (1.0f - mix) + iceHue * mix;

            float sat = 0.8f + smoothHigh * 0.3f;
            float val = field * (0.6f + smoothBass * 0.8f);

            // HSV → RGB (fast inline)
            float c = val * sat;
            float h = fmodf(hue, 360.0f) / 60.0f;
            float x_val = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));
            float m = val - c;

            float r, g, b;
            if (h < 1) { r = c; g = x_val; b = 0; }
            else if (h < 2) { r = x_val; g = c; b = 0; }
            else if (h < 3) { r = 0; g = c; b = x_val; }
            else if (h < 4) { r = 0; g = x_val; b = c; }
            else if (h < 5) { r = x_val; g = 0; b = c; }
            else { r = c; g = 0; b = x_val; }

            uint8_t R = (uint8_t)((r + m) * 255);
            uint8_t G = (uint8_t)((g + m) * 255);
            uint8_t B = (uint8_t)((b + m) * 255);

            dma_display->drawPixelRGB888(x, y, R, G, B);
        }
    }

    t += 0.04f;
    dma_display->flipDMABuffer();
}


void QuantumPlasmaReactive2() {
    static float t = 0.0f;
    static float hueShift = 0.0f;

   // dma_display->clearScreen();

    // --- 1. Audio energy analysis ---
    float bassEnergy = 0.0f;
    float midEnergy = 0.0f;
    float highEnergy = 0.0f;

    int third = SAMPLES / 3;
    for (int i = 0; i < third; i++) bassEnergy += fabs((float)samples[i] / 32768.0f);
    for (int i = third; i < 2 * third; i++) midEnergy += fabs((float)samples[i] / 32768.0f);
    for (int i = 2 * third; i < SAMPLES; i++) highEnergy += fabs((float)samples[i] / 32768.0f);

    bassEnergy /= third;
    midEnergy /= third;
    highEnergy /= third;

    static float smoothBass = 0, smoothMid = 0, smoothHigh = 0;
    smoothBass = 0.8f * smoothBass + 0.2f * bassEnergy;
    smoothMid = 0.85f * smoothMid + 0.15f * midEnergy;
    smoothHigh = 0.9f * smoothHigh + 0.1f * highEnergy;

    // --- 2. Global animation parameters ---
    hueShift += 0.8f + smoothHigh * 25.0f;
    float plasmaSpeed = 0.5f + smoothBass * 3.0f;
    float intensity = 0.7f + (smoothBass + smoothMid) * 1.3f;

    // --- 3. Multi-octave plasma field ---
    for (int y = 0; y < HEIGHT; y++) {
        float fy = (y - HEIGHT / 2) * 0.15f;
        for (int x = 0; x < WIDTH; x++) {
            float fx = (x - WIDTH / 2) * 0.15f;

            // Multi-frequency plasma for more detail
            float v = 0.0f;
            v += sinf(fx * 3.1f + t * plasmaSpeed) * 1.0f;
            v += sinf((fy + t * 0.7f) * 4.3f) * 0.8f;
            v += sinf((fx + fy) * 2.5f - t * 1.3f) * 0.6f;
            v += cosf(sqrtf(fx * fx + fy * fy) * 4.0f - t * plasmaSpeed * 1.2f) * 1.2f;
            v += sinf(fx * 8.2f + t * 2.1f) * 0.3f;  // High frequency detail
            v += cosf(fy * 7.3f - t * 1.7f) * 0.3f;  // High frequency detail

            // Local audio reaction
            int idx = ((x + y * WIDTH) % SAMPLES);
            float localAmp = fabs((float)samples[idx]) / 32768.0f;
            v += localAmp * 3.0f * smoothBass;

            v *= intensity;

            // Normalize field 0..1
            float field = (v + 6.0f) / 12.0f;
            field = constrain(field, 0.0f, 1.0f);

            // --- 4. Sophisticated color gradients ---
            float hue, sat, val;
            
            // Choose between different color palettes based on audio energy
            float paletteSelector = fmodf(hueShift / 360.0f + smoothMid * 2.0f, 1.0f);
            
            if (paletteSelector < 0.25f) {
                // Deep Ocean to Electric Cyan gradient
                hue = 220.0f + field * 60.0f + smoothHigh * 40.0f;
                sat = 0.9f - field * 0.3f + smoothBass * 0.4f;
                val = 0.3f + field * 0.7f + smoothBass * 0.5f;
            } else if (paletteSelector < 0.5f) {
                // Magenta Dream to Gold Fire gradient
                hue = 300.0f - field * 120.0f + smoothMid * 50.0f;
                sat = 0.8f + field * 0.2f;
                val = 0.4f + powf(field, 1.5f) * 0.6f + smoothBass * 0.3f;
            } else if (paletteSelector < 0.75f) {
                // Forest Green to Sunset Orange gradient
                hue = 120.0f + field * 90.0f + smoothHigh * 30.0f;
                sat = 0.7f + field * 0.3f;
                val = 0.5f + field * 0.5f + smoothMid * 0.4f;
            } else {
                // Purple Rain to Pink Neon gradient
                hue = 270.0f + field * 70.0f + smoothBass * 60.0f;
                sat = 0.9f - field * 0.2f + smoothHigh * 0.3f;
                val = 0.6f + field * 0.4f;
            }

            // Add overall hue cycling
            hue = fmodf(hue + hueShift, 360.0f);
            
            // Enhance saturation with audio
            sat = constrain(sat + smoothHigh * 0.4f, 0.3f, 1.0f);
            val = constrain(val, 0.1f, 1.0f);

            // HSV → RGB with gamma correction for better colors
            float c = val * sat;
            float h = fmodf(hue, 360.0f) / 60.0f;
            float x_val = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));
            float m = val - c;

            float r, g, b;
            if (h < 1) { r = c; g = x_val; b = 0; }
            else if (h < 2) { r = x_val; g = c; b = 0; }
            else if (h < 3) { r = 0; g = c; b = x_val; }
            else if (h < 4) { r = 0; g = x_val; b = c; }
            else if (h < 5) { r = x_val; g = 0; b = c; }
            else { r = c; g = 0; b = x_val; }

            // Gamma correction for more vibrant colors
            r = powf(r + m, 1.2f);
            g = powf(g + m, 1.2f);
            b = powf(b + m, 1.2f);

            uint8_t R = (uint8_t)(constrain(r * 255, 0, 255));
            uint8_t G = (uint8_t)(constrain(g * 255, 0, 255));
            uint8_t B = (uint8_t)(constrain(b * 255, 0, 255));

            dma_display->drawPixelRGB888(x, y, R, G, B);
        }
    }

    t += 0.04f;
    dma_display->flipDMABuffer();
}




// ============================================================================
// FUNCTION 1: Inferno - Fire Simulation with Perlin Noise & Audio
// ============================================================================
void Inferno() {
    static float fireTime = 0.0f;
    static float smokeField[WIDTH][HEIGHT];
    static float heatField[WIDTH][HEIGHT];
    static unsigned long lastUpdate = 0;
    
    unsigned long now = millis();
    float deltaTime = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;
    
    //dma_display->clearScreen();
    float t = now * 0.001f;
    
    // Calculate audio energy distribution
    float bassHeat = 0.0f, midFlame = 0.0f, highSpark = 0.0f;
    int third = SAMPLES / 3;
    
    for (int i = 0; i < third; i++) {
        bassHeat += fabs((float)samples[i] / 32768.0f);
    }
    for (int i = third; i < third * 2; i++) {
        midFlame += fabs((float)samples[i] / 32768.0f);
    }
    for (int i = third * 2; i < SAMPLES; i++) {
        highSpark += fabs((float)samples[i] / 32768.0f);
    }
    bassHeat /= third;
    midFlame /= third;
    highSpark /= third;
    
    // Update heat field with Perlin-like noise
    for (int x = 0; x < WIDTH; x++) {
        for (int y = 0; y < HEIGHT; y++) {
            float nx = (float)x / WIDTH * 4.0f;
            float ny = (float)y / HEIGHT * 4.0f;
            
            // Multi-octave noise for realistic fire texture
            float noise = 0.0f;
            float amplitude = 1.0f;
            float frequency = 1.0f;
            
            for (int octave = 0; octave < 4; octave++) {
                float sx = nx * frequency + fireTime * 0.5f;
                float sy = ny * frequency - fireTime * 1.2f;
                
                noise += fastSin(sx * 3.14159f) * cos(sy * 3.14159f) * amplitude;
                noise += fastSin(sx * 2.5f + sy * 1.7f) * amplitude * 0.5f;
                
                frequency *= 2.0f;
                amplitude *= 0.5f;
            }
            
            // Audio modulation - bass creates heat sources at bottom
            float audioHeat = 0.0f;
            if (y > HEIGHT - 12) {
                int audioX = (int)((float)x / WIDTH * SAMPLES);
                audioX = constrain(audioX, 0, SAMPLES - 1);
                float localAmp = fabs((float)samples[audioX] / 32768.0f);
                audioHeat = localAmp * 2.0f * (1.0f - (HEIGHT - y) / 12.0f);
            }
            
            // Heat rises - use exponential falloff
            float risingHeat = exp(-ny * 0.8f) * (1.0f + bassHeat * 2.0f);
            
            // Combine noise with heat propagation
            heatField[x][y] = noise * 0.3f + risingHeat + audioHeat;
            
            // Add turbulence from mid frequencies
            float turbulence = fastSin(nx * 8.0f + t * 3.0f) * fastCos(ny * 6.0f - t * 2.0f);
            heatField[x][y] += turbulence * midFlame * 0.5f;
            
            // Clamp heat values
            heatField[x][y] = constrain(heatField[x][y], 0.0f, 2.0f);
        }
    }
    
    // Smoke propagation using cellular automata
    for (int x = 1; x < WIDTH - 1; x++) {
        for (int y = 1; y < HEIGHT - 1; y++) {
            // Smoke rises and diffuses
            float diffusion = (
                heatField[x-1][y] + heatField[x+1][y] + 
                heatField[x][y-1] + heatField[x][y+1]
            ) * 0.25f;
            
            smokeField[x][y] = (heatField[x][y] + diffusion) * 0.5f;
            
            // Smoke rises (shift upward)
            if (y > 0) {
                smokeField[x][y] = smokeField[x][y] * 0.7f + heatField[x][y+1] * 0.3f;
            }
        }
    }
    
    // Render fire and smoke
    for (int x = 0; x < WIDTH; x++) {
        for (int y = 0; y < HEIGHT; y++) {
            float heat = heatField[x][y];
            float smoke = smokeField[x][y];
            
            // Fire color gradient based on heat
            uint8_t r = 0, g = 0, b = 0;
            
            if (heat > 1.5f) {
                // White hot core
                r = 255;
                g = 240 + 15 * highSpark;
                b = 200 + 55 * highSpark;
            } else if (heat > 1.0f) {
                // Yellow-white flames
                float t = (heat - 1.0f) * 2.0f;
                r = 255;
                g = 180 + 75 * t;
                b = 50 + 150 * t * highSpark;
            } else if (heat > 0.5f) {
                // Orange flames
                float t = (heat - 0.5f) * 2.0f;
                r = 200 + 55 * t;
                g = 80 + 100 * t * midFlame;
                b = 10 + 40 * t;
            } else if (heat > 0.2f) {
                // Red embers
                float t = (heat - 0.2f) * 3.33f;
                r = 120 + 80 * t;
                g = 20 + 60 * t;
                b = 5;
            } else {
                // Dark smoke
                float smokeIntensity = smoke * 60.0f;
                r = smokeIntensity;
                g = smokeIntensity * 0.9f;
                b = smokeIntensity * 1.1f;
            }
            
            // Add sparks on high frequencies
            if (highSpark > 0.6f && random(0, 100) < 2) {
                r = min(255, r + 100);
                g = min(255, g + 80);
                b = min(255, b + 60);
            }
            
            dma_display->drawPixelRGB888(x, y, r, g, b);
        }
    }
    
    fireTime += deltaTime * (1.0f + bassHeat * 2.0f);
    dma_display->flipDMABuffer();
}