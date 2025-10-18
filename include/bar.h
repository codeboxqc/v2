#include <Arduino.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h>
 
 
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>



#define PANEL_RES_X 64
#define PANEL_RES_Y 64

#define  WIDTH  64
#define  HEIGHT 64


#define PANEL_CHAIN 1
#define I2S_WS    16 
#define I2S_SD    9
#define I2S_SCK   14

#define SAMPLE_RATE 44100
#define SAMPLES 256   //1024

extern MatrixPanel_I2S_DMA *dma_display;
 
constexpr int nBands = 16;

extern MatrixPanel_I2S_DMA *dma_display;
extern double bins[nBands];
extern double barHeight[nBands];
extern double maxBin;
extern double peakHold[nBands];
extern int32_t samples[SAMPLES];  // Add this for waveform access

uint16_t hsvToRgb(float h, float s, float v);
void drawBars();
double getWaveEnergy();

struct Flame {
  int width;
  int height;
  uint8_t* buffer;
  uint8_t* fire_buffer;
};


extern Flame flame;

 
void drawBars();
void drawFlameBars();


void initFlame(Flame* f);
void updateFlame(Flame* f);
void drawFlameBarsFromBuffer(Flame* f) ;


void drawCircularWaveform();
void drawWaveform();



void initBodies();
void updateBodies();
void drawOrbitalTrails();

void body3();
void initBodies();
void defineShapes();