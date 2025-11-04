/********************************************************************
 *  ESP32 HUB75 64x64 – Advanced Generative Portrait Art Engine
 *  ------------------------------------------------------------
 *   Divination (I Ching), Cellular Automata 
 *  (Game of Life), Quantum (Grover sim), Randomness fields, 
 *  Synchronicity motion, Fluid density sim, Wave Function Collapse,
 *  Recursive Tiling, and more. Switches randomly every 20-60s.
 *  All styles evoke low-res portrait art on 64x64 grid.
 ********************************************************************/
#include <Arduino.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h>
#include <cmath>
#include <algorithm>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <math.h>

 #include "bar.h" 


inline float grad(int hash, float x, float y) {
  int h = hash & 15;
  float g1 = (h & 1) ? x : -x;
  float g2 = (h & 2) ? y : -y;
  return ((h & 8) ? -g1 : g1) + ((h & 4) ? -g2 : g2);
}

inline float lerp(float a, float b, float t) {
  return a + t * (b - a);
}
 
inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r>>3)<<11) | ((g>>2)<<5) | (b>>3);
}

uint16_t hslToRgb565(float h, float s, float l) {
  float c = (1 - fabsf(2*l-1))*s;
  float x = c*(1 - fabsf(fmodf(h/60,2)-1));
  float m = l - c/2;
  float r,g,b;
  if      (h<60)  {r=c; g=x; b=0;}
  else if (h<120) {r=x; g=c; b=0;}
  else if (h<180) {r=0; g=c; b=x;}
  else if (h<240) {r=0; g=x; b=c;}
  else if (h<300) {r=x; g=0; b=c;}
  else            {r=c; g=0; b=x;}
  uint8_t R = (uint8_t)roundf((r+m)*255);
  uint8_t G = (uint8_t)roundf((g+m)*255);
  uint8_t B = (uint8_t)roundf((b+m)*255);
  return rgb565(R,G,B);
}

 


// Perlin noise (integer-based)
static uint8_t p[512];
static void noiseInit() {
  for (int i=0;i<256;i++) p[i]=i;
  for (int i=255;i>0;i--) {
    int j=random(i+1);
    uint8_t t=p[i]; p[i]=p[j]; p[j]=t;
  }
  for (int i=0;i<256;i++) p[256+i]=p[i];
}
static float noise(float x, float y) {
  int X = (int)floorf(x) & 255;
  int Y = (int)floorf(y) & 255;
  x -= floorf(x); y -= floorf(y);
  float u = x*x*(3-2*x);
  float v = y*y*(3-2*y);
  int A = p[X]+Y,   AA = p[A], AB = p[A+1];
  int B = p[X+1]+Y, BA = p[B], BB = p[B+1];
  return lerp(lerp(grad(p[AA],x  ,y  ),
                   grad(p[BA],x-1,y  ),u),
              lerp(grad(p[AB],x  ,y-1),
                   grad(p[BB],x-1,y-1),u),v);
}




void fastFillScreen(uint16_t color) {
  static uint16_t lastColor = 0xFFFF;
  if (color == lastColor) return;  // Skip if same
  
  // Use drawFastHLine - hardware optimized
  for(int y = 0; y < 64; y++) {
    dma_display->drawFastHLine(0, y, 64, color);
  }
  
  lastColor = color;
}


struct FaceParams {
  float eyeX1, eyeY, eyeX2, eyeSize;
  float noseX, noseY, noseH;
  float mouthX, mouthY, mouthW;
  float hueBase;
  void randomize() {
    eyeX1 = random(15,25)/1.0f; eyeX2 = PANEL_RES_X - eyeX1 - random(0,5);
    eyeY = random(20,30)/1.0f;
    eyeSize = random(5,10)/1.0f;
    noseX = PANEL_RES_X/2.0f + random(-5,6);
    noseY = eyeY + random(10,15);
    noseH = random(10,20)/1.0f;
    mouthX = PANEL_RES_X/2.0f + random(-5,6);
    mouthY = noseY + random(10,15);
    mouthW = random(20,40)/1.0f;
    hueBase = random(360);
  }
};
FaceParams face;
FaceParams prevFace;



void smoothFace(float alpha = 0.7f) {  // alpha = 0.0: instant, 1.0: no change
    /*
  prevFace.eyeX1 = lerp(prevFace.eyeX1, face.eyeX1, alpha);
  prevFace.eyeX2 = lerp(prevFace.eyeX2, face.eyeX2, alpha);
  prevFace.eyeY = lerp(prevFace.eyeY, face.eyeY, alpha);
  prevFace.eyeSize = lerp(prevFace.eyeSize, face.eyeSize, alpha);
  prevFace.noseX = lerp(prevFace.noseX, face.noseX, alpha);
  prevFace.noseY = lerp(prevFace.noseY, face.noseY, alpha);
  prevFace.noseH = lerp(prevFace.noseH, face.noseH, alpha);
  prevFace.mouthX = lerp(prevFace.mouthX, face.mouthX, alpha);
  prevFace.mouthY = lerp(prevFace.mouthY, face.mouthY, alpha);
  prevFace.mouthW = lerp(prevFace.mouthW, face.mouthW, alpha);
  prevFace.hueBase = lerp(prevFace.hueBase, face.hueBase, alpha);
  */

    float t=millis()*0.003;
    WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
 
  float beat = globalAudio.filteredAmplitude;
  float rawAmp = globalAudio.amplitude; 
  
  prevFace.eyeX1 = lerp(prevFace.eyeX1, face.eyeX1 + rawAmp * 5.0f, alpha);
  prevFace.eyeX2 = lerp(prevFace.eyeX2, face.eyeX2 - rawAmp * 5.0f, alpha);
  prevFace.eyeY = lerp(prevFace.eyeY, face.eyeY + beat * 6.0f, alpha);
  prevFace.eyeSize = lerp(prevFace.eyeSize, face.eyeSize * (1.0f + beat * 0.8f), alpha);
  prevFace.noseX = lerp(prevFace.noseX, face.noseX + rawAmp * 3.0f, alpha);
  prevFace.noseY = lerp(prevFace.noseY, face.noseY + beat * 8.0f, alpha);
  prevFace.noseH = lerp(prevFace.noseH, face.noseH * (1.0f + beat * 1.2f), alpha);
  prevFace.mouthX = lerp(prevFace.mouthX, face.mouthX + rawAmp * 4.0f, alpha);
  prevFace.mouthY = lerp(prevFace.mouthY, face.mouthY + sinf(t * 10.0f + rawAmp * 8.0f) * 3.0f, alpha);
  prevFace.mouthW = lerp(prevFace.mouthW, face.mouthW * (1.0f + beat * 0.9f), alpha);
  prevFace.hueBase = fmodf(prevFace.hueBase + beat * 120.0f, 360.0f);
}

// ---------------------------------------------------------------
// 3. Style Enum & Switcher (expanded)
// ---------------------------------------------------------------
enum ArtStyle {
  ABSTRACT_CUBIST,     // 0
  NOISE_FACE,          // 1
  FRACTAL_EYES,        // 2
  POINTILLIST,         // 4
  SURREAL_MELT,        // 5
  GEOMETRIC_MINIMAL,   // 6
  VAPORWAVE_GLITCH,    // 7
  EXPRESSIONIST_BRUSH, // 8
  NEO_POP_COLLAGE,     // 9
  DIVINATION_ICHING,   // 10
  CELLULAR_LIFE,       // 11
  QUANTUM_GROVER,      // 12
  RANDOMNESS_FIELD,    // 13
  SYNCHRONICITY_MOTION,// 14
 
  RECURSIVE_TILE,      // 17
  QUANTUM_SUPERPOS,    // 18
  DIVINATION_RUNES     // 19
};


#define NUM_STYLES 17
ArtStyle currentStyle = ABSTRACT_CUBIST;
uint32_t nextSwitch = 0;
uint32_t switchInterval = 0;

// ---------------------------------------------------------------
// 4. Global Buffers for New FX
// ---------------------------------------------------------------
uint8_t lifeGrid[PANEL_RES_X][PANEL_RES_Y];  // For CA
uint8_t lifeNext[PANEL_RES_X][PANEL_RES_Y];
float density[PANEL_RES_X][PANEL_RES_Y];     // For fluid
float vx[PANEL_RES_X][PANEL_RES_Y], vy[PANEL_RES_X][PANEL_RES_Y];

// Simple WFC structs
#define WFC_TILES 4  // 0=bg, 1=skin, 2=eye, 3=shadow
int wfcGrid[PANEL_RES_X][PANEL_RES_Y];
bool wfcCollapsed = false;

// Particles for synchronicity
#define NUM_SYNC_PARTS 32  //100
struct Particle { float x,y,vx,vy; float phase; };
Particle parts[NUM_SYNC_PARTS];

// ---------------------------------------------------------------
// 5. Init Functions for New FX
// ---------------------------------------------------------------
void initLife() {
  memset(lifeGrid, 0, sizeof(lifeGrid));
  // Seed with face outline
  for (int y=0; y<PANEL_RES_Y; y++) {
    for (int x=0; x<PANEL_RES_X; x++) {
      float distHead = hypotf(x - PANEL_RES_X/2, y - PANEL_RES_Y/2 + 5);
      if (distHead < 25 && random(100) < 50) lifeGrid[x][y] = 1;
      if (hypotf(x-prevFace.eyeX1, y-prevFace.eyeY) < prevFace.eyeSize && random(100) < 70) lifeGrid[x][y] = 1;
      // Similar for other features
    }
  }
}

void initFluid() {
  memset(density, 0, sizeof(density));
  memset(vx, 0, sizeof(vx));
  memset(vy, 0, sizeof(vy));
  // Seed density at mouth/nose
  for (int i=0; i<10; i++) {
    density[(int)prevFace.mouthX][(int)prevFace.mouthY + i] = 1.0f;
  }
}

void initWFC() {
  for (int y=0; y<PANEL_RES_Y; y++) for (int x=0; x<PANEL_RES_X; x++) wfcGrid[x][y] = -1;
  wfcCollapsed = false;
}

void initRecursive() {
  // No init needed
}

void initSyncMotion() {
  for (int i=0; i<NUM_SYNC_PARTS; i++) {
    parts[i].x = PANEL_RES_X/2;
    parts[i].y = PANEL_RES_Y/2;
    parts[i].vx = cosf(i*0.06f) * 0.5f;
    parts[i].vy = sinf(i*0.06f) * 0.5f;
    parts[i].phase = random(360)/360.0f * 6.28f;
  }
}


void initQuantum() {
  // No init
}


void drawThickLine(int x0, int y0, int x1, int y1, int thick, uint16_t col)
{
  if (thick <= 1)
  {
    dma_display->drawLine(x0, y0, x1, y1, col);
    return;
  }

  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, e2;

  for (int t = -(thick/2); t <= thick/2; ++t)
  {
    int cx = x0, cy = y0;
    int offset = t;
    if (abs(dx) > abs(dy))
      offset = (t * dy) / dx;  // approximate perpendicular offset
    else if (dy != 0)
      offset = (t * dx) / dy;

    int px = cx + offset * sy;  // rough perpendicular
    int py = cy - offset * sx;

    int x = px, y = py;
    int ex = x1 + offset * sy;
    int ey = y1 - offset * sx;

    dma_display->drawLine(x, y, ex, ey, col);
  }
}


 /********************************************************************
 *  ABSTRACT CUBIST – Audio-Reactive Sprite-Art
 *  ------------------------------------------------
 *  • smoothFace()   → no jitter between frames
 *  • per-pixel WaveData → hue, saturation, size, line-count,
 *    line-thickness and mouth-wiggle all follow the beat
 *  • all geometry is still drawn with the HUB75 primitives
 *    (fillEllipse, fillRect, fillTriangle, drawLine)
 ********************************************************************/

 void drawAbstractCubist(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.12f);               // 0 = instant, 1 = frozen
  // prevFace now contains the *smoothed* values we will draw

  /*---------------------------------------------------------
    2. GLOBAL AUDIO SAMPLE (center of the panel)
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  
  int   mouthY   = (int)prevFace.mouthY;
  int   mouthW   = (int)(prevFace.mouthW * (1.0f + beat * 0.4f));
  float mouthOsc = sinf(t * 8.0f + rawAmp * 12.0f) * 4.0f;   // ±4 px

   float baseSaturation = 0.7f + beat * 0.3f;
    float baseBrightness = 0.4f + beat * 0.4f;
    float saturation = baseSaturation + (beat * 0.4f);
    float brightness = baseBrightness + (beat * 0.3f);

  dma_display->drawLine(
      (int)(prevFace.mouthX - mouthW / 2) ,
      mouthY,
      (int)(prevFace.mouthX + mouthW / 2) ,
      (int)(mouthY + mouthOsc),
      hslToRgb565(prevFace.hueBase + 90.0f, 0.9f, 0.3f + saturation * 0.3f));

  

  /*---------------------------------------------------------
    8. CUBIST RANDOM LINES – count, thickness & hue follow beat
    ---------------------------------------------------------*/
  int   lineCount   = 12 + (int)(beat * 20);               // 12-32 lines
  float lineThick   = 1.0f + beat * 2.0f;                  // 1-3 px
  for (int i = 0; i < lineCount; ++i)
  {
    int x1 = random(PANEL_RES_X);
    int y1 = random(PANEL_RES_Y);
    int x2 = random(PANEL_RES_X);
    int y2 = random(PANEL_RES_Y);

    // hue rotates with time + a little beat modulation
    float lineHue = fmodf(t * 30.0f + i * 7.0f + beat * 60.0f,saturation* 360.0f);

    

    uint16_t lineCol = hslToRgb565(lineHue, 0.6f + beat * 0.3f, 0.5f);
    

    // draw a thick line by overdrawing a few pixels
    for (int thick = 0; thick < (int)lineThick; ++thick)
    {
      int ox = thick / 2, oy = thick / 2;
      dma_display->drawLine(x1 + ox, y1 + oy, x2 + ox, y2 + oy, lineCol);
    }
  }
}


void drawFractalEyes(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE (anti-flicker) – we draw the eyes later
    ---------------------------------------------------------*/
  smoothFace(0.15f);   // 0 = instant, 1 = frozen

  /*---------------------------------------------------------
    2. GLOBAL AUDIO SAMPLE (center of the panel)
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  /*---------------------------------------------------------
    3. FRACTAL PARAMETERS driven by audio
    ---------------------------------------------------------*/
  // Base zoom oscillates slowly, beat adds extra pulse
  float baseZoom = 1.0f + sinf(t * 0.8f) * 0.4f;          // 0.6-1.4
  float zoom     = baseZoom + beat * 1.2f;               // up to ~2.6

  // More iterations on loud beats → richer detail
  int maxIter = 30 + (int)(beat * 45);                   // 30-75

  // Hue base follows face hue, beat shifts it
  float hueOffset = beat * 120.0f;                       // 0-120°

  // Brightness boost on strong hits
  float brightBoost = beat * 0.4f;                       // 0-0.4

  /*---------------------------------------------------------
    4. RENDER MANDELBROT (64×64 – still fast)
    ---------------------------------------------------------*/
  for (int y = 0; y < PANEL_RES_Y; ++y)
  {
    for (int x = 0; x < PANEL_RES_X; ++x)
    {
      // ---- map pixel → complex plane (centered, zoom) ----
      float cx = (x - PANEL_RES_X/2.0f) / (PANEL_RES_X * zoom) - 0.7f;
      float cy = (y - PANEL_RES_Y/2.0f) / (PANEL_RES_X * zoom);

      // ---- classic Mandelbrot iteration ----
      float zx = cx, zy = cy;
      int   iter = 0;

      while (iter < maxIter && (zx*zx + zy*zy) < 4.0f)
      {
        float xt = zx*zx - zy*zy + cx;
        zy = 2.0f*zx*zy + cy;
        zx = xt;
        ++iter;
      }

      // ---- colour from iteration count ----
      if (iter < maxIter)
      {
        // Smooth colouring (optional – looks nicer)
        float smooth = iter + 1.0f - log2f(log2f(zx*zx + zy*zy));
        float hue = fmodf(prevFace.hueBase + hueOffset + smooth * 3.0f, 360.0f);

        // Optional per-pixel audio wiggle (subtle)
        WaveData localAudio = calculateWaveData(x, t);
        hue = fmodf(hue + localAudio.amplitude * 20.0f, 360.0f);

        float sat = 0.9f + beat * 0.1f;                 // 0.9-1.0
        float val = 0.4f + brightBoost + smooth / maxIter * 0.4f;

        dma_display->drawPixel(x, y,
          hslToRgb565(hue, sat, val));
      }
      else
      {
        // inside set → dark background
        dma_display->drawPixel(x, y,
          hslToRgb565(prevFace.hueBase, 0.2f, 0.05f));
      }
    }
  }

  /*---------------------------------------------------------
    5. OVERLAY FACE FEATURES (eyes stay on top)
    ---------------------------------------------------------*/
  // Eye size pulses with beat
  float eyePulse = 1.0f + beat * 0.5f;                     // 1-1.5×
  int   eyeR     = (int)(prevFace.eyeSize * eyePulse);

  uint16_t eyeCol = hslToRgb565(
      fmodf(prevFace.hueBase + 120.0f + rawAmp * 30.0f, 360.0f),
      1.0f,
      0.8f + beat * 0.2f);

  dma_display->fillCircle((int)prevFace.eyeX1, (int)prevFace.eyeY, eyeR, eyeCol);
  dma_display->fillCircle((int)prevFace.eyeX2, (int)prevFace.eyeY, eyeR, eyeCol);

  /*---------------------------------------------------------
    6. OPTIONAL: tiny pupil that follows the raw sample
    ---------------------------------------------------------*/
  int pupilR = max(1, eyeR / 3);
  int pupilX1 = (int)(prevFace.eyeX1 + rawAmp * 2.0f);
  int pupilX2 = (int)(prevFace.eyeX2 + rawAmp * 2.0f);
  dma_display->fillCircle(pupilX1, (int)prevFace.eyeY, pupilR,
                         hslToRgb565(prevFace.hueBase + 180.0f, 1.0f, 0.2f));
  dma_display->fillCircle(pupilX2, (int)prevFace.eyeY, pupilR,
                         hslToRgb565(prevFace.hueBase + 180.0f, 1.0f, 0.2f));
}
 

void drawNoiseFace(float t) {
  smoothFace();
  float scale = 0.05f + sinf(t*0.1f)*0.02f;
  for (int y=0; y<PANEL_RES_Y; y++) {
    for (int x=0; x<PANEL_RES_X; x++) {
      WaveData audio = calculateWaveData(x, t);  // Per-pixel audio for reactivity
      float n = noise(x*scale, y*scale + t) * 0.5f + 0.5f;
      // Modulate for face: darker around eyes/nose + audio
      float distEye1 = hypotf(x-prevFace.eyeX1, y-prevFace.eyeY);
      float distEye2 = hypotf(x-prevFace.eyeX2, y-prevFace.eyeY);
      float distNose = hypotf(x-prevFace.noseX, y-prevFace.noseY);
      float distMouth = hypotf(x-prevFace.mouthX, y-prevFace.mouthY);
      if (distEye1 < prevFace.eyeSize || distEye2 < prevFace.eyeSize) n *= 0.3f + audio.amplitude*0.2f;
      if (distNose < 5) n *= 0.5f + audio.filteredAmplitude*0.3f;
      if (distMouth < 3 && abs(x-prevFace.mouthX) < prevFace.mouthW/2) n *= 0.4f + audio.amplitude*0.1f;
      uint16_t col = hslToRgb565(prevFace.hueBase + n*60 + audio.filteredAmplitude*90, 0.8f, n + audio.amplitude*0.2f);
      dma_display->drawPixel(x, y, col);
    }
  }
}

 

/********************************************************************
 *  COLOR-FIELD SILHOUETTE – Audio-Reactive Minimal Portrait
 *  -------------------------------------------------------
 *  • smoothFace()        → no jitter when face.randomize() runs
 *  • globalAudio         → drives hue rotation, saturation,
 *                           size-pulse, field-count
 *  • per-feature audio   → eyes, nose, mouth get unique hue
 *                           modulation from left / centre / right
 *  • background pulses gently with bass
 ********************************************************************/
int frameCounter = 0;
void drawColorFieldSilhouette(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.18f);               // 0 = instant, 1 = frozen
  // prevFace now holds the smoothly-interpolated values

  /*---------------------------------------------------------
    2. GLOBAL AUDIO SAMPLE (center of panel)
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData2(PANEL_RES_X / 2, t);
  
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  /*---------------------------------------------------------
    3. BACKGROUND – hue rotates slowly, brightness pulses
    ---------------------------------------------------------*/
  float bgHue   = fmodf(prevFace.hueBase + sinf(t * 0.5f) * 30.0f + beat * 60.0f, 360.0f);
  float bgLight = 0.25f + beat * 0.15f;                     // 0.25-0.40
  uint16_t bg   = hslToRgb565(bgHue, 0.4f, bgLight);

  //dma_display->fillScreen(bg);
  // fastFillScreen(bg) ;

  

frameCounter++; // increment every loop

// Every 5 frames, call fastFillScreen
if (frameCounter== 100) {
    fastFillScreen(bg) ;
    frameCounter=0;
}

  /*---------------------------------------------------------
    4. HEAD SILHOUETTE – size pulses with beat
    ---------------------------------------------------------*/
  float headPulse = 1.0f + beat * 0.4f;                     // 1-1.4×
  int   headW     = (int)(25 * headPulse);
  int   headH     = (int)(30 * headPulse);
  uint16_t headCol = hslToRgb565(
      fmodf(prevFace.hueBase + 180.0f + rawAmp * 20.0f, 360.0f),
      0.12f,
      0.08f + beat * 0.08f);                       // darker on quiet
  dma_display->fillEllipse(PANEL_RES_X/2, PANEL_RES_Y/2, headW, headH, headCol);

  /*---------------------------------------------------------
    5. FEATURE AUDIO SAMPLES (left / centre / right)
    ---------------------------------------------------------*/
  WaveData leftAudio  = calculateWaveData(PANEL_RES_X / 4, t);
  WaveData rightAudio = calculateWaveData(PANEL_RES_X * 3 / 4, t);
  WaveData midAudio   = calculateWaveData(PANEL_RES_X / 2, t);

  /*---------------------------------------------------------
    6. EYES – rectangles that grow & change hue per side
    ---------------------------------------------------------*/
  float eyeScale = 1.0f + beat * 0.6f;                      // 1-1.6×
  int   eyeW     = (int)(10 * eyeScale);
  int   eyeH     = (int)(10 * eyeScale);

  // Left eye – hue from left-audio
  float leftHue = fmodf(prevFace.hueBase + leftAudio.amplitude * 40.0f, 360.0f);
  uint16_t leftEyeCol = hslToRgb565(leftHue, 0.7f + beat * 0.2f, 0.6f + beat * 0.2f);
  dma_display->fillRect(
      (int)(prevFace.eyeX1 - 5), (int)(prevFace.eyeY - 5),
      eyeW, eyeH, leftEyeCol);

  // Right eye – hue from right-audio
  float rightHue = fmodf(prevFace.hueBase + rightAudio.amplitude * 40.0f, 360.0f);
  uint16_t rightEyeCol = hslToRgb565(rightHue, 0.7f + beat * 0.2f, 0.6f + beat * 0.2f);
  dma_display->fillRect(
      (int)(prevFace.eyeX2 - 5), (int)(prevFace.eyeY - 5),
      eyeW, eyeH, rightEyeCol);

  /*---------------------------------------------------------
    7. NOSE – triangle height wiggles with mid-freq
    ---------------------------------------------------------*/
  float noseWiggle = midAudio.filteredAmplitude * 8.0f;     // 0-8 px
  int   noseH      = (int)(prevFace.noseH + noseWiggle);
  float noseHue    = fmodf(prevFace.hueBase + 120.0f + beat * 80.0f, 360.0f);
  uint16_t noseCol = hslToRgb565(noseHue, 0.7f, 0.55f + beat * 0.25f);

  dma_display->fillTriangle(
      (int)(prevFace.noseX - 5), (int)prevFace.noseY,
      (int)(prevFace.noseX + 5), (int)prevFace.noseY,
      (int)prevFace.noseX,       (int)(prevFace.noseY + noseH),
      noseCol);

  /*---------------------------------------------------------
    8. MOUTH – width & vertical offset follow beat
    ---------------------------------------------------------*/
  float mouthScale = 1.0f + beat * 0.5f;                    // 1-1.5×
  int   mouthW     = (int)(prevFace.mouthW * mouthScale);
  int   mouthY     = (int)(prevFace.mouthY + sinf(t * 6.0f + rawAmp * 10.0f) * 3.0f);

  float mouthHue = fmodf(prevFace.hueBase + 240.0f + beat * 100.0f, 360.0f);
  uint16_t mouthCol = hslToRgb565(mouthHue, 0.8f, 0.5f + beat * 0.3f);

  dma_display->fillRect(
      (int)(prevFace.mouthX - mouthW/2), mouthY - 2,
      mouthW, 4, mouthCol);

  /*---------------------------------------------------------
    9. OPTIONAL: subtle “glow” outline on loud hits
    ---------------------------------------------------------*/
  if (beat > 0.65f)   // strong beat
  {
    uint16_t glowCol = hslToRgb565(bgHue, 0.9f, 0.9f);
    dma_display->drawEllipse(PANEL_RES_X/2, PANEL_RES_Y/2, headW+2, headH+2, glowCol);
  }
}





 /********************************************************************
 *  POINTILLIST – Audio-Reactive Dot-Painting Portrait
 *  -------------------------------------------------
 *  • smoothFace()        → no jitter when face.randomize()
 *  • globalAudio         → drives dot density, size, hue shift
 *  • per-region audio    → eyes (left/right), nose, mouth get
 *                           unique hue modulation
 *  • all dots are drawn with drawPixel() → pure pointillism
 ********************************************************************/

void drawDot(int cx, int cy, int radius, uint16_t col)
{
  if (radius <= 1)
  {
    if (cx >= 0 && cx < PANEL_RES_X && cy >= 0 && cy < PANEL_RES_Y)
      dma_display->drawPixel(cx, cy, col);
    return;
  }

  int r2 = radius * radius;
  for (int dy = -radius; dy <= radius; ++dy)
  {
    for (int dx = -radius; dx <= radius; ++dx)
    {
      if (dx*dx + dy*dy <= r2)
      {
        int x = cx + dx;
        int y = cy + dy;
        if (x >= 0 && x < PANEL_RES_X && y >= 0 && y < PANEL_RES_Y)
        {
          // Fade edges
          float dist = sqrtf(dx*dx + dy*dy) / radius;
          float alpha = 1.0f - dist;
          // Simple alpha blend with black background (approx)
          uint8_t r = ((col >> 11) & 0x1F) * alpha;
          uint8_t g = ((col >> 5)  & 0x3F) * alpha;
          uint8_t b = (col & 0x1F) * alpha;
          dma_display->drawPixel(x, y, (r<<11) | (g<<5) | b);
        }
      }
    }
  }
}

void drawPointillist(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.14f);               // 0 = instant, 1 = frozen
  // prevFace now holds the smoothly-interpolated values

  /*---------------------------------------------------------
    2. GLOBAL AUDIO SAMPLE (center of panel)
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  /*---------------------------------------------------------
    3. BACKGROUND – dark canvas, subtle hue drift
    ---------------------------------------------------------*/
  float bgHue   = fmodf(prevFace.hueBase + sinf(t * 0.3f) * 20.0f + beat * 50.0f, 360.0f);
  float bgLight = 0.15f + beat * 0.12f;                     // 0.15-0.27

//  dma_display->fillScreen(hslToRgb565(bgHue, 0.25f, bgLight));
 fastFillScreen(hslToRgb565(bgHue, 0.25f, bgLight)) ;

  /*---------------------------------------------------------
    4. AUDIO SAMPLES FOR STEREO REGIONS
    ---------------------------------------------------------*/
  WaveData leftAudio  = calculateWaveData(PANEL_RES_X / 4, t);         // left side
  WaveData rightAudio = calculateWaveData(PANEL_RES_X * 3 / 4, t);     // right side
  WaveData midAudio   = calculateWaveData(PANEL_RES_X / 2, t);         // centre

  /*---------------------------------------------------------
    5. DOT PARAMETERS driven by audio
    ---------------------------------------------------------*/
  int   baseDots   = 1400 + (int)(beat * 1200);        // 1400-2600 dots
  float dotSize    = 1.0f + beat * 1.5f;               // 1-2.5 px (we draw multiple pixels)
  float hueSpread  = 30.0f + beat * 40.0f;             // colour variation
  float brightBase = 0.45f + beat * 0.25f;             // 0.45-0.70

  /*---------------------------------------------------------
    6. POINTILLIST LOOP – 3 passes for layering
    ---------------------------------------------------------*/
  // Pass 1: HEAD AREA (large soft dots)
  for (int i = 0; i < baseDots / 3; ++i)
  {
    int x = random(PANEL_RES_X);
    int y = random(PANEL_RES_Y);
    float distCenter = hypotf(x - PANEL_RES_X/2, y - PANEL_RES_Y/2);

    if (distCenter < 30 + beat * 8)   // head grows slightly with beat
    {
      float hue = fmodf(prevFace.hueBase + random(-30, 30) + beat * 60.0f, 360.0f);
      uint16_t col = hslToRgb565(hue, 0.7f, brightBase + sinf(t + i * 0.1f) * 0.15f);
      drawDot(x, y, (int)dotSize, col);
    }
  }

  // Pass 2: EYES – stereo-aware colour + size pulse
  for (int i = 0; i < baseDots / 5; ++i)
  {
    // Left eye cluster
    if (random(2) == 0)
    {
      int x = (int)prevFace.eyeX1 + random(-8, 9);
      int y = (int)prevFace.eyeY  + random(-8, 9);
      if (hypotf(x - prevFace.eyeX1, y - prevFace.eyeY) < prevFace.eyeSize * 2.2f)
      {
        float hue = fmodf(prevFace.hueBase + 180.0f + leftAudio.amplitude * 50.0f, 360.0f);
        float eyeBright = 0.65f + beat * 0.25f;
        uint16_t col = hslToRgb565(hue, 0.9f, eyeBright);
        drawDot(x, y, (int)(dotSize * (1.0f + leftAudio.filteredAmplitude)), col);
      }
    }
    // Right eye cluster
    else
    {
      int x = (int)prevFace.eyeX2 + random(-8, 9);
      int y = (int)prevFace.eyeY  + random(-8, 9);
      if (hypotf(x - prevFace.eyeX2, y - prevFace.eyeY) < prevFace.eyeSize * 2.2f)
      {
        float hue = fmodf(prevFace.hueBase + 180.0f + rightAudio.amplitude * 50.0f, 360.0f);
        float eyeBright = 0.65f + beat * 0.25f;
        uint16_t col = hslToRgb565(hue, 0.9f, eyeBright);
        drawDot(x, y, (int)(dotSize * (1.0f + rightAudio.filteredAmplitude)), col);
      }
    }
  }

  // Pass 3: NOSE & MOUTH – mid-freq energy
  for (int i = 0; i < baseDots / 6; ++i)
  {
    // Nose cluster
    if (random(2) == 0)
    {
      int x = (int)prevFace.noseX + random(-6, 7);
      int y = (int)prevFace.noseY + random(-3, (int)(prevFace.noseH + midAudio.filteredAmplitude * 10));
      if (hypotf(x - prevFace.noseX, y - prevFace.noseY) < 10)
      {
        float hue = fmodf(prevFace.hueBase + 60.0f + midAudio.amplitude * 40.0f, 360.0f);
        uint16_t col = hslToRgb565(hue, 0.8f, 0.55f + beat * 0.2f);
        drawDot(x, y, (int)dotSize, col);
      }
    }
    // Mouth cluster
    else
    {
      float mouthWiggle = sinf(t * 7.0f + rawAmp * 8.0f) * 4.0f;
      int x = (int)prevFace.mouthX + random(-(int)prevFace.mouthW/2, (int)prevFace.mouthW/2 + 1);
      int y = (int)(prevFace.mouthY + mouthWiggle + random(-3, 4));
      if (abs(x - prevFace.mouthX) < prevFace.mouthW/2 + 3)
      {
        float hue = fmodf(prevFace.hueBase + 240.0f + beat * 80.0f, 360.0f);
        uint16_t col = hslToRgb565(hue, 0.85f, 0.5f + beat * 0.3f);
        drawDot(x, y, (int)(dotSize * 0.8f), col);
      }
    }
  }

  /*---------------------------------------------------------
    7. OPTIONAL: “sparkle” on strong beats
    ---------------------------------------------------------*/
  if (beat > 0.6f)
  {
    for (int i = 0; i < 30; ++i)
    {
      int x = random(PANEL_RES_X);
      int y = random(PANEL_RES_Y);
      uint16_t spark = hslToRgb565(random(360), 1.0f, 1.0f);
      dma_display->drawPixel(x, y, spark);
    }
  }
}

 


/********************************************************************
 *  SURREAL MELT – Audio-Reactive Dripping Portrait
 *  ------------------------------------------------
 *  • smoothFace()        → jitter-free face overlay
 *  • globalAudio         → drives melt intensity, speed, hue
 *  • per-pixel audio     → adds high-frequency shimmer
 *  • eyes are warped but stay bright and readable
 ********************************************************************/

void drawSurrealMelt(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.16f);               // 0 = instant, 1 = frozen
  // prevFace holds the smoothly-interpolated values

  /*---------------------------------------------------------
    2. GLOBAL AUDIO SAMPLE (center of panel)
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  /*---------------------------------------------------------
    3. MELT PARAMETERS driven by audio
    ---------------------------------------------------------*/
  // Base melt intensity + beat pulse
  float baseMelt   = 4.0f + beat * 10.0f;               // 4-14 px
  float speedX     = 2.0f + beat * 3.0f;                // horizontal wave speed
  float speedY     = 1.0f + beat * 2.0f;                // vertical wave speed
  float noiseScale = 0.04f + beat * 0.03f;              // texture detail

  // Hue drift with time + beat
  float hueDrift   = sinf(t * 0.4f) * 30.0f + beat * 80.0f;

  /*---------------------------------------------------------
    4. RENDER WARPED PIXELS
    ---------------------------------------------------------*/
  for (int y = 0; y < PANEL_RES_Y; ++y)
  {
    for (int x = 0; x < PANEL_RES_X; ++x)
    {
      // ---- MELT OFFSET (X and Y waves) ----
      float meltX = sinf((x + t * speedX) / 10.0f) * baseMelt * 0.6f;
      float meltY = sinf((y + t * speedY) / 8.0f)  * baseMelt;
      float totalMelt = meltY + sinf((x + y + t * 3.0f) / 12.0f) * baseMelt * 0.3f;

      int srcY = (int)(y + totalMelt);
      if (srcY < 0 || srcY >= PANEL_RES_Y) {
        // Out of bounds → dark drip
        dma_display->drawPixel(x, y, hslToRgb565(prevFace.hueBase, 0.3f, 0.05f));
        continue;
      }

      // ---- NOISE TEXTURE + AUDIO SHIMMER ----
      float n = noise(x * noiseScale, srcY * noiseScale + t * 0.5f);
      float localAudio = calculateWaveData(x, t).amplitude;
      float hue = fmodf(
          prevFace.hueBase + hueDrift + n * 90.0f + localAudio * 30.0f,
          360.0f);

      float sat = 0.75f + beat * 0.2f;
      float val = 0.45f + beat * 0.25f + n * 0.2f;

      uint16_t col = hslToRgb565(hue, sat, val);

      // ---- EYE ACCENTS (warped but bright) ----
      float eyeDist1 = hypotf(x - prevFace.eyeX1, (float)srcY - prevFace.eyeY);
      float eyeDist2 = hypotf(x - prevFace.eyeX2, (float)srcY - prevFace.eyeY);
      float eyeSize  = prevFace.eyeSize * (1.0f + beat * 0.4f);

      if (eyeDist1 < eyeSize || eyeDist2 < eyeSize)
      {
        float eyeHue = fmodf(prevFace.hueBase + 120.0f + rawAmp * 40.0f, 360.0f);
        col = hslToRgb565(eyeHue, 1.0f, 0.8f + beat * 0.2f);
      }
      // ---- NOSE & MOUTH (optional subtle highlight) ----
      else if (abs(x - prevFace.noseX) < 6 && abs(srcY - (prevFace.noseY + prevFace.noseH/2)) < 8)
      {
        col = hslToRgb565(prevFace.hueBase + 60.0f, 0.9f, 0.6f);
      }
      else if (abs(x - prevFace.mouthX) < prevFace.mouthW/2 + 2 && abs(srcY - prevFace.mouthY) < 4)
      {
        float mouthOsc = sinf(t * 10.0f + rawAmp * 15.0f) * 2.0f;
        col = hslToRgb565(prevFace.hueBase + 240.0f, 0.9f, 0.5f + beat * 0.3f);
      }

      dma_display->drawPixel(x, y, col);
    }
  }

  /*---------------------------------------------------------
    5. OPTIONAL: "drip trails" on strong beats
    ---------------------------------------------------------*/
  if (beat > 0.75f)
  {
    for (int i = 0; i < 8; ++i)
    {
      int x = random(PANEL_RES_X);
      int y = PANEL_RES_Y - 1;
      uint16_t dripCol = hslToRgb565(random(360), 1.0f, 0.9f);
      for (int len = 0; len < 6 + (int)(beat * 8); ++len)
      {
        if (y - len >= 0)
          dma_display->drawPixel(x + random(-1,2), y - len, dripCol);
      }
    }
  }
}





/********************************************************************
 *  GEOMETRIC MINIMAL – Audio-Reactive Wireframe Portrait
 *  ----------------------------------------------------
 *  • smoothFace()        → jitter-free geometry
 *  • globalAudio         → drives scale, rotation, hue, line count
 *  • left/right audio    → eyes move & change hue independently
 *  • all primitives: drawCircle, drawLine, drawTriangle, drawRect
 ********************************************************************/

void drawGeometricMinimal(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.15f);               // 0 = instant, 1 = frozen
  // prevFace holds smoothly-interpolated values

  /*---------------------------------------------------------
    2. GLOBAL AUDIO SAMPLE (center)
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  /*---------------------------------------------------------
    3. STEREO AUDIO SAMPLES (left / right)
    ---------------------------------------------------------*/
  WaveData leftAudio  = calculateWaveData(PANEL_RES_X / 4, t);
  WaveData rightAudio = calculateWaveData(PANEL_RES_X * 3 / 4, t);

  /*---------------------------------------------------------
    4. BACKGROUND – black with subtle hue pulse
    ---------------------------------------------------------*/
  float bgLight = 0.0f + beat * 0.08f;  // 0-0.08 (almost black)
  //dma_display->fillScreen(hslToRgb565(prevFace.hueBase, 0.1f, bgLight));
  fastFillScreen(hslToRgb565(prevFace.hueBase, 0.1f, bgLight));

  /*---------------------------------------------------------
    5. HEAD CIRCLE – size & hue pulse
    ---------------------------------------------------------*/
  float headScale = 1.0f + beat * 0.4f;                     // 1-1.4×
  int   headR     = (int)(25 * headScale);
  float headRot   = beat * 15.0f;                          // subtle spin on beat
  uint16_t headCol = hslToRgb565(
      fmodf(prevFace.hueBase + sinf(t * 0.6f) * 20.0f + beat * 60.0f, 360.0f),
      0.5f + beat * 0.2f,
      0.5f + beat * 0.3f);

  // Draw rotated circle using multiple lines
  for (int a = 0; a < 360; a += 12)
  {
    float rad1 = radians(a + headRot);
    float rad2 = radians(a + 12 + headRot);
    int x1 = PANEL_RES_X/2 + (int)(cosf(rad1) * headR);
    int y1 = PANEL_RES_Y/2 + (int)(sinf(rad1) * headR);
    int x2 = PANEL_RES_X/2 + (int)(cosf(rad2) * headR);
    int y2 = PANEL_RES_Y/2 + (int)(sinf(rad2) * headR);
    dma_display->drawLine(x1, y1, x2, y2, headCol);
  }

  /*---------------------------------------------------------
    6. EYES – horizontal lines that move & change hue
    ---------------------------------------------------------*/
  float eyeScale = 1.0f + beat * 0.6f;
  int   eyeLen   = (int)(prevFace.eyeSize * eyeScale);

  // Left eye – reacts to left audio
  int leftX  = (int)(prevFace.eyeX1 + leftAudio.amplitude * 3.0f);
  int leftY  = (int)(prevFace.eyeY  + sinf(t * 8.0f) * beat * 2.0f);
  float leftHue = fmodf(prevFace.hueBase + 60.0f + leftAudio.filteredAmplitude * 80.0f, 360.0f);
  uint16_t leftCol = hslToRgb565(leftHue, 0.7f + beat * 0.2f, 0.6f + beat * 0.2f);
  dma_display->drawLine(leftX, leftY, leftX + eyeLen, leftY, leftCol);

  // Right eye – reacts to right audio
  int rightX = (int)(prevFace.eyeX2 + rightAudio.amplitude * 3.0f);
  int rightY = (int)(prevFace.eyeY  + sinf(t * 8.0f + 3.14f) * beat * 2.0f);
  float rightHue = fmodf(prevFace.hueBase + 60.0f + rightAudio.filteredAmplitude * 80.0f, 360.0f);
  uint16_t rightCol = hslToRgb565(rightHue, 0.7f + beat * 0.2f, 0.6f + beat * 0.2f);
  dma_display->drawLine(rightX, rightY, rightX - eyeLen, rightY, rightCol);

  /*---------------------------------------------------------
    7. NOSE – triangle that stretches with mid-freq
    ---------------------------------------------------------*/
  WaveData midAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float noseStretch = 1.0f + midAudio.filteredAmplitude * 1.2f;
  int   noseH       = (int)(prevFace.noseH * noseStretch);
  float noseHue     = fmodf(prevFace.hueBase + 120.0f + beat * 90.0f, 360.0f);
  uint16_t noseCol  = hslToRgb565(noseHue, 0.8f, 0.4f + beat * 0.3f);

  dma_display->drawTriangle(
      (int)prevFace.noseX - 3, (int)prevFace.noseY,
      (int)prevFace.noseX + 3, (int)prevFace.noseY,
      (int)prevFace.noseX,     (int)(prevFace.noseY + noseH),
      noseCol);

  /*---------------------------------------------------------
    8. MOUTH – rectangle that widens & shifts vertically
    ---------------------------------------------------------*/
  float mouthScale = 1.0f + beat * 0.7f;
  int   mouthW     = (int)(prevFace.mouthW * mouthScale);
  int   mouthY     = (int)(prevFace.mouthY + sinf(t * 10.0f + rawAmp * 12.0f) * 2.5f);
  float mouthHue   = fmodf(prevFace.hueBase + 240.0f + beat * 100.0f, 360.0f);
  uint16_t mouthCol = hslToRgb565(mouthHue, 0.9f, 0.3f + beat * 0.4f);

  dma_display->drawRect(
      (int)(prevFace.mouthX - mouthW/2), mouthY,
      mouthW, 3, mouthCol);

  /*---------------------------------------------------------
    9. RANDOM GEO LINES – count, thickness, hue follow beat
    ---------------------------------------------------------*/
  int   lineCount = 8 + (int)(beat * 14);                  // 8-22 lines
  float lineThick = 1.0f + beat * 1.5f;                    // 1-2.5 px
  for (int i = 0; i < lineCount; ++i)
  {
    int x1 = random(PANEL_RES_X);
    int y1 = random(PANEL_RES_Y);
    int x2 = random(PANEL_RES_X);
    int y2 = random(PANEL_RES_Y);

    float lineHue = fmodf(t * 25.0f + i * 11.0f + beat * 70.0f, 360.0f);
    uint16_t lineCol = hslToRgb565(lineHue, 0.4f + beat * 0.3f, 0.5f + beat * 0.2f);

    // Thick line by overdrawing
    for (int thick = 0; thick < (int)lineThick; ++thick)
    {
      int ox = (thick % 2) ? thick/2 : -thick/2;
      int oy = (thick % 2) ? -thick/2 : thick/2;
      dma_display->drawLine(x1 + ox, y1 + oy, x2 + ox, y2 + oy, lineCol);
    }
  }

  /*---------------------------------------------------------
   10. OPTIONAL: "pulse ring" on strong beat
   ---------------------------------------------------------*/
  if (beat > 0.8f)
  {
    int pulseR = (int)(30 + beat * 20);
    uint16_t pulseCol = hslToRgb565(prevFace.hueBase, 1.0f, 0.9f);
    dma_display->drawCircle(PANEL_RES_X/2, PANEL_RES_Y/2, pulseR, pulseCol);
  }
}



/********************************************************************
 *  VAPORWAVE GLITCH – Audio-Reactive Retro Scanline Portrait
 *  --------------------------------------------------------
 *  • smoothFace()        → stable head position
 *  • globalAudio         → drives glitch intensity, speed, hue
 *  • left/right audio    → stereo head pulse
 *  • scanlines + glitch  → beat-synced bursts
 ********************************************************************/

void drawVaporwaveGlitch(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.17f);               // 0 = instant, 1 = frozen
  // prevFace holds smoothly-interpolated values

  /*---------------------------------------------------------
    2. GLOBAL & STEREO AUDIO SAMPLES
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  WaveData leftAudio  = calculateWaveData(PANEL_RES_X / 4, t);
  WaveData rightAudio = calculateWaveData(PANEL_RES_X * 3 / 4, t);

  /*---------------------------------------------------------
    3. BACKGROUND – retro cyan/magenta pulse
    ---------------------------------------------------------*/
  float bgHue   = 200.0f + sinf(t * 0.5f) * 20.0f + beat * 60.0f;
  float bgLight = 0.18f + beat * 0.12f;                     // 0.18-0.30
  //dma_display->fillScreen(hslToRgb565(bgHue, 0.8f, bgLight));
  fastFillScreen(hslToRgb565(bgHue, 0.8f, bgLight));

  /*---------------------------------------------------------
    4. GLITCH PARAMETERS driven by audio
    ---------------------------------------------------------*/
  float glitchIntensity = 3.0f + beat * 12.0f;              // 3-15 px
  float glitchSpeed     = 1.0f + beat * 3.0f;               // wave speed
  int   scanlineCount   = 4 + (int)(beat * 8);              // 4-12 lines
  float hueShift        = beat * 120.0f;                    // neon shift on beat

  /*---------------------------------------------------------
    5. RENDER GLITCHED FRAME
    ---------------------------------------------------------*/
  for (int y = 0; y < PANEL_RES_Y; ++y)
  {
    // ---- GLITCH OFFSET (per scanline) ----
    float wave = sinf((y + t * glitchSpeed) * 0.15f) * glitchIntensity;
    int offset = (int)wave + (int)(rawAmp * 3.0f);  // raw jitter

    // ---- SCANLINE COLOR STRIPE (every 8px) ----
    bool isScanline = (y % 8 == 0);
    float baseHue = prevFace.hueBase + (isScanline ? hueShift : 0.0f);

    for (int x = 0; x < PANEL_RES_X; ++x)
    {
      int srcX = x + offset;
      if (srcX < 0 || srcX >= PANEL_RES_X) {
        dma_display->drawPixel(x, y, hslToRgb565(280, 1.0f, 0.1f)); // dark purple void
        continue;
      }

      // ---- HEAD GLOW (pulsing with stereo) ----
      float headDist = hypotf(srcX - PANEL_RES_X/2, y - PANEL_RES_Y/2);
      float headPulse = 1.0f + (leftAudio.filteredAmplitude + rightAudio.filteredAmplitude) * 0.3f;
      float headRadius = 25.0f * headPulse;

      uint16_t col;
      if (headDist < headRadius)
      {
        float glowHue = fmodf(baseHue + 60.0f + beat * 80.0f, 360.0f);
        float glowVal = 0.65f + beat * 0.25f;
        col = hslToRgb565(glowHue, 1.0f, glowVal);
      }
      else
      {
        float sat = 0.85f + beat * 0.1f;
        float val = 0.55f + beat * 0.15f;
        col = hslToRgb565(baseHue, sat, val);
      }

      // ---- EYE HIGHLIGHTS (stereo-reactive) ----
      float eyeDist1 = hypotf(srcX - prevFace.eyeX1, y - prevFace.eyeY);
      float eyeDist2 = hypotf(srcX - prevFace.eyeX2, y - prevFace.eyeY);
      float eyeSize  = prevFace.eyeSize * (1.0f + beat * 0.5f);

      if (eyeDist1 < eyeSize)
      {
        float eyeHue = fmodf(prevFace.hueBase + 120.0f + leftAudio.amplitude * 50.0f, 360.0f);
        col = hslToRgb565(eyeHue, 1.0f, 0.9f);
      }
      else if (eyeDist2 < eyeSize)
      {
        float eyeHue = fmodf(prevFace.hueBase + 120.0f + rightAudio.amplitude * 50.0f, 360.0f);
        col = hslToRgb565(eyeHue, 1.0f, 0.9f);
      }

      dma_display->drawPixel(x, y, col);
    }
  }

  /*---------------------------------------------------------
    6. GLITCH LINES – count & color follow beat
    ---------------------------------------------------------*/
  for (int i = 0; i < scanlineCount; ++i)
  {
    int y = random(PANEL_RES_Y);
    float lineHue = fmodf(t * 40.0f + i * 15.0f + beat * 100.0f, 360.0f);
    uint16_t lineCol = hslToRgb565(lineHue, 1.0f, 0.8f + beat * 0.2f);

    // Thick glitch line
    int thick = 1 + (int)(beat * 3);
    for (int dy = -thick/2; dy <= thick/2; ++dy)
    {
      int ly = y + dy;
      if (ly >= 0 && ly < PANEL_RES_Y)
        dma_display->drawFastHLine(0, ly, PANEL_RES_X, lineCol);
    }
  }

  /*---------------------------------------------------------
    7. BURST ON STRONG BEATS
    ---------------------------------------------------------*/
  if (beat > 0.75f)
  {
    for (int i = 0; i < 6; ++i)
    {
      int x = random(PANEL_RES_X);
      int y = random(PANEL_RES_Y);
      uint16_t burst = hslToRgb565(random(360), 1.0f, 1.0f);
      dma_display->drawPixel(x, y, burst);
      if (random(3) == 0)
        dma_display->drawFastHLine(x-3, y, 7, burst);
    }
  }
}



/********************************************************************
 *  EXPRESSIONIST BRUSH – Audio-Reactive Impasto Portrait
 *  ----------------------------------------------------
 *  • smoothFace()        → stable face anchor
 *  • globalAudio         → drives stroke count, energy, hue
 *  • left/right audio    → eyes get stereo color + size
 *  • mid audio           → mouth wiggles with raw waveform
 ********************************************************************/

void drawExpressionistBrush(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.16f);               // 0 = instant, 1 = frozen
  // prevFace holds smoothly-interpolated values

  /*---------------------------------------------------------
    2. AUDIO SAMPLES – global, stereo, mid
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  WaveData leftAudio  = calculateWaveData(PANEL_RES_X / 4, t);
  WaveData rightAudio = calculateWaveData(PANEL_RES_X * 3 / 4, t);
  WaveData midAudio   = calculateWaveData(PANEL_RES_X / 2, t);

  /*---------------------------------------------------------
    3. BACKGROUND – dark canvas with subtle hue drift
    ---------------------------------------------------------*/
  float bgHue   = fmodf(prevFace.hueBase + sinf(t * 0.4f) * 25.0f + beat * 70.0f, 360.0f);
  float bgLight = 0.08f + beat * 0.12f;                     // 0.08-0.20
  //dma_display->fillScreen(hslToRgb565(bgHue, 0.3f, bgLight));
   fastFillScreen(hslToRgb565(bgHue, 0.3f, bgLight));

  /*---------------------------------------------------------
    4. STROKE PARAMETERS driven by audio
    ---------------------------------------------------------*/
  int   baseStrokes   = 350 + (int)(beat * 400);             // 350-750 strokes
  float strokeScale   = 1.0f + beat * 1.2f;                  // length multiplier
  float thickness     = 1.0f + beat * 2.5f;                  // 1-3.5 px
  float hueSpread     = 45.0f + beat * 60.0f;                // color variation
  float energy        = beat * 0.6f + 0.4f;                  // brightness base

  /*---------------------------------------------------------
    5. MAIN BRUSH STROKES – chaotic face mass
    ---------------------------------------------------------*/
  for (int i = 0; i < baseStrokes; ++i)
  {
    // Anchor around face center, spread grows with beat
    float spreadX = 25.0f + beat * 15.0f;
    float spreadY = 30.0f + beat * 18.0f;
    int x = (int)(PANEL_RES_X/2 + random(-spreadX, spreadX + 1));
    int y = (int)(PANEL_RES_Y/2 + random(-spreadY, spreadY + 1));

    // Stroke length & direction
    int len = (int)(random(5, 16) * strokeScale);
    float angle = t * (0.5f + beat * 2.0f) + i * 0.018f + rawAmp * 3.0f; // swirling motion
    float rad = angle;
    int dx = (int)(cosf(rad) * len);
    int dy = (int)(sinf(rad) * len);

    // Color: hue from face + audio, brightness wiggles
    float hue = fmodf(prevFace.hueBase + random(-hueSpread, hueSpread + 1) + beat * 80.0f, 360.0f);
    float val = energy + random(-15, 16) / 100.0f;
    uint16_t col = hslToRgb565(hue, 0.8f + beat * 0.15f, val);

    // Draw thick stroke
    drawThickLine(x, y, x + dx, y + dy, (int)thickness, col);
  }

  /*---------------------------------------------------------
    6. EYES – bold, stereo-reactive strokes
    ---------------------------------------------------------*/
  int eyeStrokes = 40 + (int)(beat * 50);  // 40-90
  for (int i = 0; i < eyeStrokes; ++i)
  {
    // Left eye
    if (i % 2 == 0)
    {
      int ex = (int)prevFace.eyeX1 + random(-10, 11);
      int ey = (int)prevFace.eyeY  + random(-6, 7);
      float hue = fmodf(prevFace.hueBase + 180.0f + leftAudio.amplitude * 60.0f, 360.0f);
      uint16_t col = hslToRgb565(hue, 1.0f, 0.8f + beat * 0.2f);
      drawThickLine(ex, ey, ex + random(-8,9), ey + random(-8,9), 2 + (int)(beat * 3), col);
    }
    // Right eye
    else
    {
      int ex = (int)prevFace.eyeX2 + random(-10, 11);
      int ey = (int)prevFace.eyeY  + random(-6, 7);
      float hue = fmodf(prevFace.hueBase + 180.0f + rightAudio.amplitude * 60.0f, 360.0f);
      uint16_t col = hslToRgb565(hue, 1.0f, 0.8f + beat * 0.2f);
      drawThickLine(ex, ey, ex + random(-8,9), ey + random(-8,9), 2 + (int)(beat * 3), col);
    }
  }

  /*---------------------------------------------------------
    7. MOUTH – expressive, waveform-driven strokes
    ---------------------------------------------------------*/
  int mouthStrokes = 30 + (int)(beat * 40);
  for (int i = 0; i < mouthStrokes; ++i)
  {
    float mouthOsc = sinf(t * 12.0f + rawAmp * 15.0f + i * 0.3f) * 4.0f;
    int mx = (int)(prevFace.mouthX + random(-prevFace.mouthW/2, prevFace.mouthW/2 + 1));
    int my = (int)(prevFace.mouthY + mouthOsc + random(-4, 5));

    float hue = fmodf(prevFace.hueBase + 240.0f + midAudio.amplitude * 70.0f, 360.0f);
    uint16_t col = hslToRgb565(hue, 0.9f, 0.5f + beat * 0.4f);
    int len = 6 + (int)(beat * 8);
    drawThickLine(mx, my, mx + random(-len, len+1), my + random(-3,4), 1 + (int)(beat * 2), col);
  }

  /*---------------------------------------------------------
    8. SPLATTER ON LOUD HITS
    ---------------------------------------------------------*/
  if (beat > 0.7f)
  {
    for (int i = 0; i < 15; ++i)
    {
      int sx = random(PANEL_RES_X);
      int sy = random(PANEL_RES_Y);
      uint16_t splat = hslToRgb565(random(360), 1.0f, 0.9f);
      for (int j = 0; j < 5; ++j)
      {
        int dx = random(-6,7), dy = random(-6,7);
        dma_display->drawPixel(sx + dx, sy + dy, splat);
      }
    }
  }
}

 



/********************************************************************
 *  NEO-POP COLLAGE – Audio-Reactive Pop-Art Explosion
 *  -------------------------------------------------
 *  • smoothFace()        → stable face anchor
 *  • globalAudio         → drives grid speed, hue, pop count
 *  • left/right audio    → eyes pulse & shift independently
 *  • all shapes: fillRect, fillEllipse, fillCircle, drawRect
 ********************************************************************/

void drawNeoPopCollage(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.18f);               // 0 = instant, 1 = frozen
  // prevFace holds smoothly-interpolated values

  /*---------------------------------------------------------
    2. AUDIO SAMPLES – global + stereo
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  WaveData leftAudio  = calculateWaveData(PANEL_RES_X / 4, t);
  WaveData rightAudio = calculateWaveData(PANEL_RES_X * 3 / 4, t);

  /*---------------------------------------------------------
    3. BACKGROUND GRID – animated, beat-driven hue
    ---------------------------------------------------------*/
  float gridHueBase = fmodf(t * 30.0f + beat * 120.0f, 360.0f);
  float gridSpeed   = 1.0f + beat * 2.0f;
  int   cellSize    = 8;

  for (int y = 0; y < PANEL_RES_Y; y += cellSize)
  {
    for (int x = 0; x < PANEL_RES_X; x += cellSize)
    {
      float phase = (x + y + t * gridSpeed * 50.0f) * 0.02f;
      float hue = fmodf(gridHueBase + sinf(phase) * 60.0f + beat * 80.0f, 360.0f);
      float sat = 0.5f + beat * 0.3f;
      float val = 0.5f + beat * 0.2f;
      dma_display->fillRect(x, y, cellSize, cellSize, hslToRgb565(hue, sat, val));
    }
  }

  /*---------------------------------------------------------
    4. HEAD ELLIPSE – wobbles, scales, hue pulses
    ---------------------------------------------------------*/
  float headScale = 1.0f + beat * 0.5f;                     // 1-1.5×
  int   headW     = (int)(25 * headScale);
  int   headH     = (int)(30 * headScale);
  float headX     = PANEL_RES_X/2 + sinf(t * 6.0f + rawAmp * 8.0f) * (5.0f + beat * 8.0f);
  float headY     = PANEL_RES_Y/2 + cosf(t * 4.0f) * 2.0f;

  float headHue = fmodf(prevFace.hueBase + beat * 100.0f, 360.0f);
  uint16_t headCol = hslToRgb565(headHue, 0.9f, 0.6f + beat * 0.3f);
  dma_display->fillEllipse((int)headX, (int)headY, headW, headH, headCol);

  /*---------------------------------------------------------
    5. EYES – stereo pulse + color shift
    ---------------------------------------------------------*/
  float eyeScale = 1.0f + beat * 0.7f;
  int   eyeR     = (int)(prevFace.eyeSize * eyeScale);

  // Left eye – left audio
  int leftX = (int)(prevFace.eyeX1 + leftAudio.amplitude * 4.0f);
  int leftY = (int)(prevFace.eyeY  + sinf(t * 10.0f) * beat * 2.0f);
  float leftHue = fmodf(prevFace.hueBase + 120.0f + leftAudio.filteredAmplitude * 90.0f, 360.0f);
  uint16_t leftCol = hslToRgb565(leftHue, 1.0f, 0.8f + beat * 0.2f);
  dma_display->fillCircle(leftX, leftY, eyeR, leftCol);

  // Right eye – right audio
  int rightX = (int)(prevFace.eyeX2 + rightAudio.amplitude * 4.0f);
  int rightY = (int)(prevFace.eyeY  + sinf(t * 10.0f + 3.14f) * beat * 2.0f);
  float rightHue = fmodf(prevFace.hueBase + 120.0f + rightAudio.filteredAmplitude * 90.0f, 360.0f);
  uint16_t rightCol = hslToRgb565(rightHue, 1.0f, 0.8f + beat * 0.2f);
  dma_display->fillCircle(rightX, rightY, eyeR, rightCol);

  /*---------------------------------------------------------
    6. RANDOM POP ELEMENTS – count, size, hue follow beat
    ---------------------------------------------------------*/
  int popCount = 8 + (int)(beat * 15);                     // 8-23
  float popScale = 1.0f + beat * 1.2f;
  int   popSize  = (int)(10 * popScale);

  for (int i = 0; i < popCount; ++i)
  {
    int x = random(PANEL_RES_X);
    int y = random(PANEL_RES_Y);

    float popHue = fmodf(t * 40.0f + i * 17.0f + beat * 100.0f, 360.0f);
    uint16_t popCol = hslToRgb565(popHue, 1.0f, 0.7f + beat * 0.3f);

    // Random shape: rect, circle, or triangle
    int shape = random(3);
    if (shape == 0)
      dma_display->drawRect(x - popSize/2, y - popSize/2, popSize, popSize, popCol);
    else if (shape == 1)
      dma_display->fillCircle(x, y, popSize/2, popCol);
    else
      dma_display->fillTriangle(x, y - popSize/2, x - popSize/2, y + popSize/2, x + popSize/2, y + popSize/2, popCol);
  }

  /*---------------------------------------------------------
    7. POP BURST ON STRONG BEATS
    ---------------------------------------------------------*/
  if (beat > 0.75f)
  {
    for (int i = 0; i < 12; ++i)
    {
      int cx = random(PANEL_RES_X);
      int cy = random(PANEL_RES_Y);
      uint16_t burstCol = hslToRgb565(random(360), 1.0f, 1.0f);
      for (int r = 1; r <= 4; ++r)
      {
        dma_display->drawCircle(cx, cy, r, burstCol);
      }
    }
  }
}



/********************************************************************
 *  DIVINATION I-CHING – Audio-Reactive Oracle Portrait
 *  --------------------------------------------------
 *  • smoothFace()        → stable face overlay
 *  • globalAudio         → drives line count, yin/yang ratio, hue
 *  • left/right audio    → eyes pulse & shift independently
 *  • seeded random       → flicker-free hexagrams
 ********************************************************************/

void drawDivinationIChing(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.19f);               // 0 = instant, 1 = frozen
  // prevFace holds smoothly-interpolated values

  /*---------------------------------------------------------
    2. AUDIO SAMPLES – global + stereo
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  WaveData leftAudio  = calculateWaveData(PANEL_RES_X / 4, t);
  WaveData rightAudio = calculateWaveData(PANEL_RES_X * 3 / 4, t);

  /*---------------------------------------------------------
    3. BACKGROUND – dark void with subtle oracle glow
    ---------------------------------------------------------*/
  float bgHue   = fmodf(prevFace.hueBase + sinf(t * 0.3f) * 30.0f + beat * 90.0f, 360.0f);
  float bgLight = 0.08f + beat * 0.12f;                     // 0.08-0.20
  //dma_display->fillScreen(hslToRgb565(bgHue, 0.3f, bgLight));
  fastFillScreen(hslToRgb565(bgHue, 0.3f, bgLight));

  /*---------------------------------------------------------
    4. ORACLE PARAMETERS driven by audio
    ---------------------------------------------------------*/
  int   lineCount   = 6 + (int)(beat * 3);                  // 6-9 lines
  float yinBias     = 0.5f + rawAmp * 0.3f;                 // -0.3 to +0.3 (yin/yang balance)
  float lineSpacing = 5.0f + beat * 2.0f;                   // 5-7 px
  float lineHue     = fmodf(prevFace.hueBase + beat * 120.0f, 360.0f);

  // Seeded random for flicker-free hexagrams
  uint32_t seed = (uint32_t)(t * 1000.0f) + (uint32_t)(beat * 1000.0f);
  randomSeed(seed);

  /*---------------------------------------------------------
    5. DRAW HEXAGRAM – centered, audio-reactive
    ---------------------------------------------------------*/
  int centerY = PANEL_RES_Y / 2;
  int startY  = centerY - (int)((lineCount - 1) * lineSpacing / 2);

  for (int line = 0; line < lineCount; ++line)
  {
    int y = startY + (int)(line * lineSpacing);

    // Yin (broken) vs Yang (solid) — biased by audio
    float randVal = (float)random(1000) / 1000.0f;
    bool isBroken = randVal < (0.5f + yinBias);

    uint16_t lineCol = hslToRgb565(
        lineHue + (isBroken ? 30.0f : 0.0f),
        0.8f + beat * 0.15f,
        0.6f + beat * 0.3f);

    int leftX1  = 10;
    int leftX2  = 25;
    int rightX1 = 39;
    int rightX2 = 54;

    if (isBroken)
    {
      // Two short lines
      dma_display->drawLine(leftX1,  y, leftX2,  y, lineCol);
      dma_display->drawLine(rightX1, y, rightX2, y, lineCol);
    }
    else
    {
      // One solid line
      dma_display->drawLine(leftX1, y, rightX2, y, lineCol);
    }

    // Subtle glow on strong beats
    if (beat > 0.7f && line % 2 == 0)
    {
      uint16_t glow = hslToRgb565(lineHue, 1.0f, 0.9f);
      dma_display->drawLine(leftX1 - 1, y, rightX2 + 1, y, glow);
    }
  }

  /*---------------------------------------------------------
    6. EYES – stereo pulse + oracle glow
    ---------------------------------------------------------*/
  float eyeScale = 1.0f + beat * 0.6f;
  int   eyeR     = (int)(prevFace.eyeSize * eyeScale);

  // Left eye – left audio
  int leftX = (int)(prevFace.eyeX1 + leftAudio.amplitude * 3.0f);
  int leftY = (int)(prevFace.eyeY  + sinf(t * 9.0f) * beat * 2.0f);
  float leftHue = fmodf(prevFace.hueBase + 120.0f + leftAudio.filteredAmplitude * 80.0f, 360.0f);
  uint16_t leftCol = hslToRgb565(leftHue, 1.0f, 0.8f + beat * 0.2f);
  dma_display->fillCircle(leftX, leftY, eyeR, leftCol);

  // Right eye – right audio
  int rightX = (int)(prevFace.eyeX2 + rightAudio.amplitude * 3.0f);
  int rightY = (int)(prevFace.eyeY  + sinf(t * 9.0f + 3.14f) * beat * 2.0f);
  float rightHue = fmodf(prevFace.hueBase + 120.0f + rightAudio.filteredAmplitude * 80.0f, 360.0f);
  uint16_t rightCol = hslToRgb565(rightHue, 1.0f, 0.8f + beat * 0.2f);
  dma_display->fillCircle(rightX, rightY, eyeR, rightCol);

  /*---------------------------------------------------------
    7. NOSE & MOUTH – minimal oracle symbols
    ---------------------------------------------------------*/
  // Nose: small changing/yang triangle
  WaveData midAudio = calculateWaveData(PANEL_RES_X / 4, t);
  int noseH = (int)(prevFace.noseH * (1.0f + midAudio.filteredAmplitude * 0.8f));
  uint16_t noseCol = hslToRgb565(prevFace.hueBase + 60.0f, 0.9f, 0.5f + beat * 0.3f);
  dma_display->fillTriangle(
      (int)prevFace.noseX - 3, (int)prevFace.noseY,
      (int)prevFace.noseX + 3, (int)prevFace.noseY,
      (int)prevFace.noseX,     (int)(prevFace.noseY + noseH),
      noseCol);

  // Mouth: yin/yang line that breathes
  float mouthOsc = sinf(t * 11.0f + rawAmp * 14.0f) * 3.0f;
  int mouthY = (int)(prevFace.mouthY + mouthOsc);
  uint16_t mouthCol = hslToRgb565(prevFace.hueBase + 240.0f, 0.9f, 0.4f + beat * 0.4f);
  dma_display->drawLine(
      (int)(prevFace.mouthX - prevFace.mouthW/2),
      mouthY,
      (int)(prevFace.mouthX + prevFace.mouthW/2),
      mouthY,
      mouthCol);

  /*---------------------------------------------------------
    8. ORACLE FLASH ON PEAKS
    ---------------------------------------------------------*/
  if (beat > 0.8f)
  {
    uint16_t flash = hslToRgb565(bgHue, 1.0f, 1.0f);
    for (int i = 0; i < 8; ++i)
    {
      int x = random(PANEL_RES_X);
      int y = random(PANEL_RES_Y);
      dma_display->drawPixel(x, y, flash);
      if (random(2) == 0)
        dma_display->drawLine(x, y, x + random(-3,4), y + random(-3,4), flash);
    }
  }

  /*---------------------------------------------------------
    9. RESTORE RANDOM SEED (for other effects)
    ---------------------------------------------------------*/
  randomSeed(millis());  // or your preferred seed
}




void updateLife() {
  memcpy(lifeNext, lifeGrid, sizeof(lifeGrid));
  for (int y=1; y<PANEL_RES_Y-1; y++) {
    for (int x=1; x<PANEL_RES_X-1; x++) {
      int n = 0;
      for (int dy=-1; dy<=1; dy++) for (int dx=-1; dx<=1; dx++) if (dx||dy) n += lifeGrid[x+dx][y+dy];
      if (lifeGrid[x][y]) lifeNext[x][y] = (n==2 || n==3) ? 1 : 0;
      else lifeNext[x][y] = (n==3) ? 1 : 0;
    }
  }
  memcpy(lifeGrid, lifeNext, sizeof(lifeGrid));
}

 

/********************************************************************
 *  CELLULAR LIFE – Audio-Reactive Game of Life Portrait
 *  ---------------------------------------------------
 *  • smoothFace()        → stable face overlay
 *  • globalAudio         → drives birth/death thresholds, hue, speed
 *  • left/right audio    → eyes pulse & shift over living cells
 *  • deterministic Life  → no flicker, same state for same audio
 ********************************************************************/
void updateLifeCustom(int b1, int b2, int d1, int d2)
{
  uint8_t nextGrid[PANEL_RES_X][PANEL_RES_Y] = {0};

  for (int y = 1; y < PANEL_RES_Y - 1; ++y)
  {
    for (int x = 1; x < PANEL_RES_X - 1; ++x)
    {
      int neighbors = 0;
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
          if (dx || dy)
            neighbors += lifeGrid[x + dx][y + dy];

      bool alive = lifeGrid[x][y];
      if (alive && (neighbors < d1 || neighbors > d2))
        nextGrid[x][y] = 0;
      else if (!alive && neighbors >= b1 && neighbors <= b2)
        nextGrid[x][y] = 1;
      else
        nextGrid[x][y] = alive;
    }
  }

  // Copy back
  memcpy(lifeGrid, nextGrid, sizeof(lifeGrid));
}

void drawCellularLife(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.20f);               // 0 = instant, 1 = frozen
  // prevFace holds smoothly-interpolated values

  /*---------------------------------------------------------
    2. AUDIO SAMPLES – global + stereo
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  WaveData leftAudio  = calculateWaveData(PANEL_RES_X / 4, t);
  WaveData rightAudio = calculateWaveData(PANEL_RES_X * 3 / 4, t);

  /*---------------------------------------------------------
    3. LIFE PARAMETERS driven by audio
    ---------------------------------------------------------*/
  // Birth/death thresholds shift with beat
  int birthLow  = 3;
  int birthHigh = 3;
  int deathLow  = 2;
  int deathHigh = 3;

  if (beat > 0.4f) {
    birthHigh = 4;  // more births on loud
    if (beat > 0.7f) deathLow = 3;  // harsher survival
  }

  // Update speed: faster on beat
  static float lifeTimer = 0.0f;
  float lifeInterval = 0.08f - beat * 0.06f;  // 0.02–0.08 sec
  lifeTimer += 1.0f / 60.0f;  // assume ~60 FPS
  bool updateNow = false;
  if (lifeTimer >= lifeInterval) {
    lifeTimer = 0.0f;
    updateNow = true;
  }

  // Seeded random for flicker-free initial patterns
  uint32_t seed = (uint32_t)(t * 1000.0f) + (uint32_t)(beat * 1000.0f);
  randomSeed(seed);

  /*---------------------------------------------------------
    4. UPDATE LIFE (only when needed)
    ---------------------------------------------------------*/
  if (updateNow) {
    updateLifeCustom(birthLow, birthHigh, deathLow, deathHigh);
  }

  /*---------------------------------------------------------
    5. RENDER LIFE GRID – color from audio
    ---------------------------------------------------------*/
  float baseHue = fmodf(prevFace.hueBase + sinf(t * 0.5f) * 30.0f + beat * 90.0f, 360.0f);
  float cellScale = 1.0f + beat * 0.5f;  // 1-1.5x cell size

  for (int y = 0; y < PANEL_RES_Y; ++y)
  {
    for (int x = 0; x < PANEL_RES_X; ++x)
    {
      if (lifeGrid[x][y])
      {
        // Local audio shimmer
        WaveData local = calculateWaveData(x, t);
        float hue = fmodf(baseHue + local.amplitude * 40.0f, 360.0f);
        float sat = 0.85f + beat * 0.1f;
        float val = 0.55f + beat * 0.3f + local.filteredAmplitude * 0.2f;

        uint16_t col = hslToRgb565(hue, sat, val);

        // Draw cell as soft circle (anti-aliased)
        int r = (int)(cellScale);
        for (int dy = -r; dy <= r; ++dy)
        {
          for (int dx = -r; dx <= r; ++dx)
          {
            if (dx*dx + dy*dy <= r*r)
            {
              int px = x + dx, py = y + dy;
              if (px >= 0 && px < PANEL_RES_X && py >= 0 && py < PANEL_RES_Y)
              {
                float dist = sqrtf(dx*dx + dy*dy) / r;
                float alpha = 1.0f - dist;
                uint8_t cr = ((col >> 11) & 0x1F) * alpha;
                uint8_t cg = ((col >> 5)  & 0x3F) * alpha;
                uint8_t cb = (col & 0x1F) * alpha;
                dma_display->drawPixel(px, py, (cr<<11) | (cg<<5) | cb);
              }
            }
          }
        }
      }
      else
      {
        // Dark background with faint grid
        float grid = ((x % 4 == 0) || (y % 4 == 0)) ? 0.05f : 0.03f;
        dma_display->drawPixel(x, y, hslToRgb565(baseHue, 0.2f, grid + beat * 0.05f));
      }
    }
  }

  /*---------------------------------------------------------
    6. EYES – living over Life, stereo-reactive
    ---------------------------------------------------------*/
  float eyeScale = 1.0f + beat * 0.7f;
  int   eyeR     = (int)(prevFace.eyeSize * eyeScale);

  // Left eye – follows living cell density near eye
  int leftX = (int)(prevFace.eyeX1 + leftAudio.amplitude * 3.0f);
  int leftY = (int)(prevFace.eyeY  + sinf(t * 10.0f) * beat * 2.0f);
  float leftHue = fmodf(prevFace.hueBase + 120.0f + leftAudio.filteredAmplitude * 100.0f, 360.0f);
  uint16_t leftCol = hslToRgb565(leftHue, 1.0f, 0.8f + beat * 0.2f);
  dma_display->fillCircle(leftX, leftY, eyeR, leftCol);

  // Right eye
  int rightX = (int)(prevFace.eyeX2 + rightAudio.amplitude * 3.0f);
  int rightY = (int)(prevFace.eyeY  + sinf(t * 10.0f + 3.14f) * beat * 2.0f);
  float rightHue = fmodf(prevFace.hueBase + 120.0f + rightAudio.filteredAmplitude * 100.0f, 360.0f);
  uint16_t rightCol = hslToRgb565(rightHue, 1.0f, 0.8f + beat * 0.2f);
  dma_display->fillCircle(rightX, rightY, eyeR, rightCol);

  /*---------------------------------------------------------
    7. LIFE PULSE ON STRONG BEATS
    ---------------------------------------------------------*/
  if (beat > 0.8f)
  {
    for (int i = 0; i < 20; ++i)
    {
      int x = random(PANEL_RES_X);
      int y = random(PANEL_RES_Y);
      if (lifeGrid[x][y])
      {
        uint16_t pulse = hslToRgb565(random(360), 1.0f, 1.0f);
        dma_display->fillCircle(x, y, 2, pulse);
      }
    }
  }

  /*---------------------------------------------------------
    8. RESTORE RANDOM SEED
    ---------------------------------------------------------*/
  randomSeed(millis());
}

  



/********************************************************************
 *  QUANTUM GROVER – Audio-Reactive Quantum Search Portrait
 *  ------------------------------------------------------
 *  • smoothFace()        → stable face overlay
 *  • globalAudio         → drives search phase, amplitude, hue
 *  • left/right audio    → eyes pulse & shift over search space
 *  • seeded random       → flicker-free target selection
 ********************************************************************/



void drawQuantumGrover(float t)
{
    //--- 1. Audio sampling ---
    WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
    float beat   = globalAudio.filteredAmplitude;   // 0-1 smooth envelope

    //--- 2. Background update every 5 frames ---
    static int frameCounter = 0;
    frameCounter++;
    if (frameCounter % 5 == 0)
    {
        float bgHue   = fmodf(prevFace.hueBase + sinf(t * 0.4f) * 40.0f + beat * 100.0f, 360.0f);
        float bgLight = 0.02f + beat * 0.08f;
        fastFillScreen( hslToRgb565(bgHue, 0.2f, bgLight));
    }

    //--- 3. Full-screen cube grid ---
    int cubeSize = 8; // 8x8 pixels
    int gridX = PANEL_RES_X / cubeSize;
    int gridY = PANEL_RES_Y / cubeSize;

    float hueBase = fmodf(prevFace.hueBase + t * 50.0f, 360.0f);

    for (int y = 0; y < gridY; ++y)
    {
        for (int x = 0; x < gridX; ++x)
        {
            // Sample audio per horizontal slice for variation
            int xSample = x * cubeSize + cubeSize / 2;
            WaveData wd = calculateWaveData(xSample, t);
            float amp = wd.filteredAmplitude;

            // Hue changes per cube and vertically
            float cubeHue = fmodf(hueBase + x * 12.0f + y * 15.0f + amp * 80.0f, 360.0f);
            float cubeSat = 0.8f + amp * 0.2f;
            float cubeVal = 0.3f + amp * 0.5f;

            uint16_t color = hslToRgb565(cubeHue, cubeSat, cubeVal);

            // Draw the 8x8 cube
            dma_display->fillRect(x * cubeSize, y * cubeSize, cubeSize, cubeSize, color);
        }
    }
}




 


/********************************************************************
 *  RANDOMNESS FIELD – Audio-Reactive Entropy Portrait
 *  -------------------------------------------------
 *  • smoothFace()        → stable face anchor
 *  • globalAudio         → drives noise density, hue, entropy
 *  • left/right audio    → eyes pulse & distort the field
 *  • seeded random       → flicker-free chaos
 ********************************************************************/

void drawRandomnessField(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.17f);               // 0 = instant, 1 = frozen
  // prevFace holds smoothly-interpolated values

  /*---------------------------------------------------------
    2. AUDIO SAMPLES – global + stereo
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  WaveData leftAudio  = calculateWaveData(PANEL_RES_X / 4, t);
  WaveData rightAudio = calculateWaveData(PANEL_RES_X * 3 / 4, t);

  /*---------------------------------------------------------
    3. RANDOMNESS PARAMETERS driven by audio
    ---------------------------------------------------------*/
  float entropy     = 0.6f + beat * 0.4f;                    // 0.6-1.0 (noise density)
  float hueDrift    = sinf(t * 0.5f) * 40.0f + beat * 100.0f;
  float brightBase  = 0.3f + beat * 0.4f;                    // 0.3-0.7
  float faceRadius  = 32.0f + beat * 12.0f;                  // 32-44 px

  // Seeded random for flicker-free noise field
  uint32_t seed = (uint32_t)(t * 1000.0f) + (uint32_t)(beat * 1000.0f);
  randomSeed(seed);

  /*---------------------------------------------------------
    4. RENDER RANDOMNESS FIELD
    ---------------------------------------------------------*/
  for (int y = 0; y < PANEL_RES_Y; ++y)
  {
    for (int x = 0; x < PANEL_RES_X; ++x)
    {
      // ---- ENTROPY THRESHOLD (sparsity) ----
      if ((float)random(1000) / 1000.0f > entropy)
      {
        dma_display->drawPixel(x, y, 0);  // dark void
        continue;
      }

      // ---- FACE MODULATION (stronger near eyes/nose/mouth) ----
      float distEye1 = hypotf(x - prevFace.eyeX1, y - prevFace.eyeY);
      float distEye2 = hypotf(x - prevFace.eyeX2, y - prevFace.eyeY);
      float distNose = hypotf(x - prevFace.noseX, y - (prevFace.noseY + prevFace.noseH/2));
      float distMouth = abs(x - prevFace.mouthX) < prevFace.mouthW/2 ? abs(y - prevFace.mouthY) : 1000;

      float minDist = fminf(fminf(distEye1, distEye2), fminf(distNose, distMouth));
      float faceMod = 1.0f - (minDist / faceRadius);
      faceMod = fmaxf(0.0f, faceMod);

      // ---- NOISE VALUE + AUDIO SHIMMER ----
      uint8_t rnd = random(256);
      float localAudio = calculateWaveData(x, t).amplitude;
      float hue = fmodf(
          prevFace.hueBase + hueDrift + rnd % 60 + localAudio * 50.0f,
          360.0f);

      float sat = 0.7f + beat * 0.2f;
      float val = brightBase + faceMod * 0.4f + (rnd / 255.0f) * 0.3f;

      uint16_t col = hslToRgb565(hue, sat, val);
      dma_display->drawPixel(x, y, col);
    }
  }

  /*---------------------------------------------------------
    6. EYES – bright, stereo-reactive beacons in the chaos
    ---------------------------------------------------------*/
  float eyeScale = 1.0f + beat * 0.8f;
  int   eyeR     = (int)(prevFace.eyeSize * eyeScale);

  // Left eye – left audio
  int leftX = (int)(prevFace.eyeX1 + leftAudio.amplitude * 4.0f);
  int leftY = (int)(prevFace.eyeY  + sinf(t * 11.0f) * beat * 3.0f);
  float leftHue = fmodf(prevFace.hueBase + 120.0f + leftAudio.filteredAmplitude * 90.0f, 360.0f);
  uint16_t leftCol = hslToRgb565(leftHue, 1.0f, 0.9f + beat * 0.1f);
  dma_display->fillCircle(leftX, leftY, eyeR, leftCol);

  // Right eye
  int rightX = (int)(prevFace.eyeX2 + rightAudio.amplitude * 4.0f);
  int rightY = (int)(prevFace.eyeY  + sinf(t * 11.0f + 3.14f) * beat * 3.0f);
  float rightHue = fmodf(prevFace.hueBase + 120.0f + rightAudio.filteredAmplitude * 90.0f, 360.0f);
  uint16_t rightCol = hslToRgb565(rightHue, 1.0f, 0.9f + beat * 0.1f);
  dma_display->fillCircle(rightX, rightY, eyeR, rightCol);

  /*---------------------------------------------------------
    7. ENTROPY BURST ON STRONG BEATS
    ---------------------------------------------------------*/
  if (beat > 0.75f)
  {
    for (int i = 0; i < 25; ++i)
    {
      int x = random(PANEL_RES_X);
      int y = random(PANEL_RES_Y);
      uint16_t burst = hslToRgb565(random(360), 1.0f, 1.0f);
      dma_display->drawPixel(x, y, burst);
      if (random(3) == 0)
      {
        dma_display->drawLine(x, y, x + random(-4,5), y + random(-4,5), burst);
      }
    }
  }

  /*---------------------------------------------------------
    8. RESTORE RANDOM SEED
    ---------------------------------------------------------*/
  randomSeed(millis());
}



void updateSyncMotion(float t) {
  float syncFreq = sinf(t*0.1f) * 2 + 3;  // Varying sync
  for (int i=0; i<NUM_SYNC_PARTS; i++) {
    Particle &p = parts[i];
    p.x += p.vx + sinf(t + p.phase * syncFreq) * 0.2f;
    p.y += p.vy + cosf(t + p.phase * syncFreq) * 0.2f;
    if (p.x < 0 || p.x >= PANEL_RES_X) p.vx = -p.vx;
    if (p.y < 0 || p.y >= PANEL_RES_Y) p.vy = -p.vy;
  }
}



/********************************************************************
 *  SYNCHRONICITY MOTION – Audio-Reactive Particle Harmony
 *  -----------------------------------------------------
 *  • smoothFace()        → stable face anchor
 *  • globalAudio         → drives sync strength, speed, hue
 *  • left/right audio    → eyes pulse & influence nearby particles
 *  • deterministic motion → flicker-free
 ********************************************************************/

 

/*---------------------------------------------------------
   Custom update with sync strength
   ---------------------------------------------------------*/
void updateSyncMotionCustom(float t, float strength)
{
  for (int i = 0; i < NUM_SYNC_PARTS; i += 2)
  {
    float angle1 = t + i * 0.3f;
    float angle2 = t + (i + 1) * 0.3f + PI;

    // Base orbit
    float cx1 = PANEL_RES_X / 2 + cosf(angle1) * 20.0f;
    float cy1 = PANEL_RES_Y / 2 + sinf(angle1) * 25.0f;
    float cx2 = PANEL_RES_X / 2 + cosf(angle2) * 20.0f;
    float cy2 = PANEL_RES_Y / 2 + sinf(angle2) * 25.0f;

    // Sync pull
    float dx = cx2 - cx1;
    float dy = cy2 - cy1;
    float dist = hypotf(dx, dy);
    if (dist > 0.1f)
    {
      float pull = strength * 0.8f;
      cx1 += dx * pull / dist;
      cy1 += dy * pull / dist;
      cx2 -= dx * pull / dist;
      cy2 -= dy * pull / dist;
    }

    parts[i].x = cx1;
    parts[i].y = cy1;
    parts[i+1].x = cx2;
    parts[i+1].y = cy2;
  }
}


void drawSynchronicityMotion(float t)
{
  /*---------------------------------------------------------
    1. SMOOTH FACE PARAMETERS (anti-flicker)
    ---------------------------------------------------------*/
  smoothFace(0.16f);               // 0 = instant, 1 = frozen
  // prevFace holds smoothly-interpolated values

  /*---------------------------------------------------------
    2. AUDIO SAMPLES – global + stereo
    ---------------------------------------------------------*/
  WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;           // -1 … +1, raw

  WaveData leftAudio  = calculateWaveData(PANEL_RES_X / 4, t);
  WaveData rightAudio = calculateWaveData(PANEL_RES_X * 3 / 4, t);

  /*---------------------------------------------------------
    3. BACKGROUND – soft void with energy pulse
    ---------------------------------------------------------*/
  float bgHue   = fmodf(prevFace.hueBase + sinf(t * 0.3f) * 30.0f + beat * 80.0f, 360.0f);
  float bgLight = 0.08f + beat * 0.12f;
  //dma_display->fillScreen(hslToRgb565(bgHue, 0.2f, bgLight));

   fastFillScreen(hslToRgb565(bgHue, 0.2f, bgLight));
  /*---------------------------------------------------------
    4. MOTION PARAMETERS driven by audio
    ---------------------------------------------------------*/
  float syncStrength = 0.4f + beat * 0.5f;                  // 0.4-0.9 (tighter sync on beat)
  float speed        = 1.0f + beat * 2.5f;                  // orbit speed
  float hueBase      = fmodf(prevFace.hueBase + beat * 100.0f, 360.0f);
  float particleSize = 1.0f + beat * 1.5f;                  // 1-2.5 px

  /*---------------------------------------------------------
    5. UPDATE & DRAW PARTICLES
    ---------------------------------------------------------*/
  updateSyncMotionCustom(t * speed, syncStrength);

  for (int i = 0; i < NUM_SYNC_PARTS; ++i)
  {
    int x = (int)parts[i].x;
    int y = (int)parts[i].y;

    // Hue per particle + audio shimmer
    float localAudio = calculateWaveData(x, t).amplitude;
    float hue = fmodf(hueBase + i * (360.0f / NUM_SYNC_PARTS) + localAudio * 40.0f, 360.0f);
    float sat = 0.9f + beat * 0.1f;
    float val = 0.6f + beat * 0.3f;

    uint16_t col = hslToRgb565(hue, sat, val);

    // Draw particle with size
    int r = (int)particleSize;
    for (int dy = -r; dy <= r; ++dy)
      for (int dx = -r; dx <= r; ++dx)
        if (dx*dx + dy*dy <= r*r)
        {
          int px = x + dx, py = y + dy;
          if (px >= 0 && px < PANEL_RES_X && py >= 0 && py < PANEL_RES_Y)
            dma_display->drawPixel(px, py, col);
        }
  }

  /*---------------------------------------------------------
    6. CONNECT SYNCED PAIRS – line thickness & color
    ---------------------------------------------------------*/
  uint16_t lineCol = hslToRgb565(hueBase, 0.5f + beat * 0.3f, 0.4f + beat * 0.3f);
  int lineThick = 1 + (int)(beat * 2);

  for (int i = 0; i < NUM_SYNC_PARTS; i += 2)
  {
    int x1 = (int)parts[i].x,     y1 = (int)parts[i].y;
    int x2 = (int)parts[i+1].x,   y2 = (int)parts[i+1].y;

    for (int thick = 0; thick < lineThick; ++thick)
    {
      int ox = (thick % 2) ? thick/2 : -thick/2;
      int oy = (thick % 2) ? -thick/2 : thick/2;
      dma_display->drawLine(x1 + ox, y1 + oy, x2 + ox, y2 + oy, lineCol);
    }
  }

  /*---------------------------------------------------------
    7. EYES – quantum observers in the sync field
    ---------------------------------------------------------*/
  float eyeScale = 1.0f + beat * 0.7f;
  int   eyeR     = (int)(prevFace.eyeSize * eyeScale);

  // Left eye – left audio
  int leftX = (int)(prevFace.eyeX1 + leftAudio.amplitude * 3.5f);
  int leftY = (int)(prevFace.eyeY  + sinf(t * 10.0f) * beat * 2.5f);
  float leftHue = fmodf(prevFace.hueBase + 120.0f + leftAudio.filteredAmplitude * 90.0f, 360.0f);
  uint16_t leftCol = hslToRgb565(leftHue, 1.0f, 0.9f + beat * 0.1f);
  dma_display->fillCircle(leftX, leftY, eyeR, leftCol);

  // Right eye
  int rightX = (int)(prevFace.eyeX2 + rightAudio.amplitude * 3.5f);
  int rightY = (int)(prevFace.eyeY  + sinf(t * 10.0f + 3.14f) * beat * 2.5f);
  float rightHue = fmodf(prevFace.hueBase + 120.0f + rightAudio.filteredAmplitude * 90.0f, 360.0f);
  uint16_t rightCol = hslToRgb565(rightHue, 1.0f, 0.9f + beat * 0.1f);
  dma_display->fillCircle(rightX, rightY, eyeR, rightCol);

  /*---------------------------------------------------------
    8. SYNC FLASH ON STRONG BEATS
    ---------------------------------------------------------*/
  if (beat > 0.8f)
  {
    uint16_t flash = hslToRgb565(hueBase, 1.0f, 1.0f);
    for (int i = 0; i < NUM_SYNC_PARTS; i += 2)
    {
      int mx = ((int)parts[i].x + (int)parts[i+1].x) / 2;
      int my = ((int)parts[i].y + (int)parts[i+1].y) / 2;
      dma_display->drawPixel(mx, my, flash);
    }
  }
}



 
 

 




void drawRecursiveTile(float t, int x=0, int y=0, int w=PANEL_RES_X, int h=PANEL_RES_Y, int depth=0) {
  if (depth > 5 || w < 2 || h < 2) return;

 WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat = globalAudio.filteredAmplitude;
  float rawAmp = globalAudio.amplitude;

  // Smoother color progression with audio modulation
  float hueShift = face.hueBase + (depth * 30.0f); // Reduced from 45° to 30° for smoother transitions
  
  // Audio-reactive hue modulation
  hueShift += beat * 40.0f; // Beat affects hue
  hueShift += sinf(t * 0.5f + depth) * 15.0f; // Gentle oscillation
  
  // Smooth brightness progression with audio influence
  float baseBrightness = 0.3f + (depth * 0.03f); // Gradual brightness increase
  float brightnessMod = sinf(t * 1.5f + depth) * 0.45f; // Subtle pulsing
  float beatBrightness = beat * 0.4f; // Direct beat response
  
  float brightness = baseBrightness + brightnessMod + beatBrightness;
  brightness = constrain(brightness, 0.1f, 0.9f); // Keep in safe range
  
  // Smooth saturation progression
  float saturation = 0.7f + (depth * 0.03f); // Gradual saturation increase
  saturation += beat * 0.2f; // Beat boosts saturation
  saturation = constrain(saturation, 0.3f, 0.9f);

  uint16_t col = hslToRgb565(fmodf(hueShift, 360.0f), saturation, brightness);



  dma_display->drawRect(x, y, w, h, col);
  if (random(100) < 70) {
    int split = random(1,3);  // Horz/vert/random
    if (split == 1) {  // Horz
      int mid = h/2 + random(-h/4, h/4);
      drawRecursiveTile(t, x, y, w, mid, depth+1);
      drawRecursiveTile(t, x, y+mid, w, h-mid, depth+1);
    } else {  // Vert
      int mid = w/2 + random(-w/4, w/4);
      drawRecursiveTile(t, x, y, mid, h, depth+1);
      drawRecursiveTile(t, x+mid, y, w-mid, h, depth+1);
    }
  }
}

void drawQuantumSuperpos(float t) {
  for (int y=0; y<PANEL_RES_Y; y++) {
    for (int x=0; x<PANEL_RES_X; x++) {

     WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;        

      float wave1 = sinf((x + y + t*beat)/10.0f) * 0.5f + 0.5f;
      float wave2 = cosf((x - y + t*rawAmp)/8.0f) * 0.5f + 0.5f;
      float superpos = (wave1 + wave2) / 2.0f;
      uint16_t col = hslToRgb565(face.hueBase + superpos*180+beat, 0.8f, superpos+rawAmp);

       
    

      dma_display->drawPixel(x, y, col);
    }
  }
}

void drawDivinationRunes(float t) {


   //  fastFillScreen(hslToRgb565(face.hueBase, 0.2f, 0.1f));

   

   WaveData globalAudio = calculateWaveData(PANEL_RES_X / 2, t);
  float beat   = globalAudio.filteredAmplitude;   // 0-1, smooth envelope
  float rawAmp = globalAudio.amplitude;        

  dma_display->fillScreen(hslToRgb565(face.hueBase+rawAmp, 0.2f, 0.1f));

  // Random runes (simple lines/symbols)
  for (int i=0; i<10+(int)(beat*3); i++) {
    int x = random(PANEL_RES_X), y = random(PANEL_RES_Y);
    int type = random(4);
    uint16_t col = hslToRgb565(face.hueBase + random(360)+beat, 0.9f+rawAmp, 0.7f);
    if (type == 0) dma_display->drawCircle(x, y, random(3,8), col);
    else if (type == 1) dma_display->drawLine(x-5, y-5, x+5, y+5, col);
    else if (type == 2) dma_display->drawTriangle(x-5, y+5, x+5, y+5, x, y-5, col);
    else dma_display->drawRect(x-4, y-4, 8, 8, col);
  }
  // Tie to portrait: Larger runes at eyes
}

// ---------------------------------------------------------------
// 7. Main Logic (expanded switch)
// ---------------------------------------------------------------
void gae() {
  Serial.begin(115200);
  Serial.println("\n=== Advanced Generative Art Engine ===");

    

  randomSeed(analogRead(0));
  noiseInit();
  face.randomize();
  currentStyle = (ArtStyle)random(NUM_STYLES);
  nextSwitch = millis() + random(20000, 60000);
  switchInterval = 6767;

  // Init new FX
  initLife();
  initFluid();
  initWFC();
  initSyncMotion();

 
}

void gae_loop() {
  uint32_t now = millis();
  float t = now * 0.003f;



  // Draw current style (expanded)
//dma_display->clearScreen();  // Clear each frame for some styles
  switch (currentStyle) {
    case ABSTRACT_CUBIST: drawAbstractCubist(t); break;
    case NOISE_FACE: drawNoiseFace(t); break;
    case FRACTAL_EYES: drawFractalEyes(t); break;
    case POINTILLIST: drawPointillist(t); break;
    case SURREAL_MELT: drawSurrealMelt(t); break;
    case GEOMETRIC_MINIMAL: drawGeometricMinimal(t); break;
    case VAPORWAVE_GLITCH: drawVaporwaveGlitch(t); break;
    case EXPRESSIONIST_BRUSH: drawExpressionistBrush(t); break;
    case NEO_POP_COLLAGE: drawNeoPopCollage(t); break;
    case DIVINATION_ICHING: drawDivinationIChing(t); break;
    case CELLULAR_LIFE: drawCellularLife(t); break;
    case QUANTUM_GROVER: drawQuantumGrover(t); break;
    case RANDOMNESS_FIELD: drawRandomnessField(t); break;
    case SYNCHRONICITY_MOTION: drawSynchronicityMotion(t); break;
    case RECURSIVE_TILE: drawRecursiveTile(t); break;
    case QUANTUM_SUPERPOS: drawQuantumSuperpos(t); break;
    case DIVINATION_RUNES: drawDivinationRunes(t); break;

    default: currentStyle=DIVINATION_RUNES;    break;
  }

  
  dma_display->flipDMABuffer();
  delay(3);  // ~30 FPS

  // Switch style
  if (now > nextSwitch) {
    currentStyle = (ArtStyle)random(NUM_STYLES);
    face.randomize();
    nextSwitch = now + switchInterval;
    switchInterval = 1 * random(5000, 9999);
    // Re-init if needed
    if (currentStyle == CELLULAR_LIFE) initLife();
 if (currentStyle == SYNCHRONICITY_MOTION) initSyncMotion();
  }
}