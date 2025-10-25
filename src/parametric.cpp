#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
 
#include "bar.h" 
// Display dimensions - adjust for your LED matrix

#define CENTER_X (WIDTH/2)
#define CENTER_Y (HEIGHT/2)

// Animation parameters
static float t = 0.0f;
static int currentCurve = 0;
static int nextCurve = 1;
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


static float fastCos(float x) {
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
            nextCurve = (nextCurve + random(1, 8)) % 25;
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