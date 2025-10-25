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





// Pure calculation function - no side effects
// Corrected WaveData structure with y coordinate
struct WaveData {
    float amplitude;
    float normalizedX;
    int y;  // ADD THIS - the calculated Y position
    uint8_t r, g, b;
    float filteredAmplitude;
};

WaveData calculateWaveData(int x, float time);


// Audio filter class
class AudioFilter {
private:
    float value = 0;
    float alpha = 0.1f;
public:
    float process(float input) {
        value = value * (1 - alpha) + input * alpha;
        return value;
    }
    void setSmoothness(float smoothness) { 
        alpha = constrain(smoothness, 0.01f, 0.3f); 
    }
};

// ADD THIS EXTERN DECLARATION:
extern AudioFilter amplitudeFilter;




 
void drawBars();
void drawFlameBars();


void initFlame(Flame* f);
void updateFlame(Flame* f);
void drawFlameBarsFromBuffer(Flame* f) ;


void drawCircularWaveform();
void drawWaveform();
void drawWaveform2();



void initBodies();
void updateBodies();
void drawOrbitalTrails();

void body3(int trans);
void initBodies();
void defineShapes();

void updateSupercharged(int charge);
void updateSupercharged();
//void updateSuperchargedAudio();


void logo();
void drawCircularWaveform2();
void DRB();
void drawParticles();
void drawCircularWaveformPure();
void drawLine(int x1, int y1, int x2, int y2, 
             uint8_t r1, uint8_t g1, uint8_t b1,
             uint8_t r2, uint8_t g2, uint8_t b2);
 
 


void drawBarsArt();