#include <Arduino.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h>
#include <ctime>

#include "bar.h" 

// HUB75 setup
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>


///////////////////////////////////////////////////////////////////
int visualizerMode = 6;  // 0 = bars, 1 = waveform, 2 = circular waveform
/////////////////////////////////////////////////////////////////////


MatrixPanel_I2S_DMA *dma_display;
HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, PANEL_CHAIN);

// INMP441 I2S mic pins


int32_t samples[SAMPLES];
double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLE_RATE);

 

// Optimized frequency bands for music (50 Hz to 16 kHz)
const uint16_t bandFrequencies[] = {
  50, 100, 160, 250, 400, 630, 1000, 1600,
  2500, 4000, 6300, 8000, 10000, 12500, 14000, 16000
};




///////////////////////////////////////////////////////////////
void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Try I2S_CHANNEL_FMT_RIGHT if no input
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024, //512  // Reduced for performance
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("I2S driver install failed: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("I2S pin config failed: %d\n", err);
    while (true);
  }
}


 


void readMicrophone() {
  size_t bytesRead;
  for (int i = 0; i < SAMPLES; i++) {
    int32_t sample = 0;
    i2s_read(I2S_NUM_0, &sample, sizeof(sample), &bytesRead, portMAX_DELAY);
    samples[i] = sample >> 8; // Adjusted scaling
   
  }
}

/////////////////////////////////////////////////////////////////





void applyWindow() {
  for (int i = 0; i < SAMPLES; i++) {
    double window = 0.5 * (1 - cos(2 * PI * i / (SAMPLES - 1)));
    vReal[i] = (double)samples[i] * window;
    vImag[i] = 0.0;
    if (i < 5) { // Debug first 5 values
      Serial.printf("vReal[%d]: %f\n", i, vReal[i]);
    }
  }
}

void calculateBands() {
  for (int i = 0; i < nBands; i++) {
    bins[i] = 0;
  }

  double freqPerBin = (double)SAMPLE_RATE / SAMPLES;
  for (int band = 0; band < nBands - 1; band++) {
    int startBin = bandFrequencies[band] / freqPerBin;
    int endBin = bandFrequencies[band + 1] / freqPerBin;
    startBin = constrain(startBin, 0, SAMPLES/2);
    endBin = constrain(endBin, 0, SAMPLES/2);
    if (startBin >= endBin) continue;

    double sum = 0;
    int count = 0;
    for (int bin = startBin; bin < endBin; bin++) {
      if (bin < SAMPLES/2) {
        double magnitude = sqrt(vReal[bin] * vReal[bin] + vImag[bin] * vImag[bin]);
        sum += magnitude;
        count++;
      }
    }
    if (count > 0) {
      bins[band] = sum / count;
      if (bins[band] > maxBin) maxBin = bins[band]; // Update max for normalization
      Serial.printf("Band %d: %f\n", band, bins[band]); // Debug band values
    }
  }
}

void updateBarHeights() {
  // Normalize based on running max
  double sensitivity = 9.0; // Reduced scaling factor

  for (int i = 0; i < nBands; i++) {
    double normalized = bins[i] / max(1.0, maxBin); // Normalize to max amplitude
    double newHeight = log1p(normalized * 100) * sensitivity; // Adjusted scaling
    newHeight = constrain(newHeight, 0, PANEL_RES_Y);
    
    // Faster decay for dynamic response
    if (newHeight > barHeight[i]) {
      barHeight[i] = newHeight;
    } else {
      barHeight[i] = max(barHeight[i] * 0.85, newHeight); // Faster decay
    }
    barHeight[i] = constrain(barHeight[i], 0, PANEL_RES_Y);
    Serial.printf("Bar %d height: %f\n", i, barHeight[i]); // Debug heights
  }
  // Slowly decay maxBin to adapt to changing audio levels
  maxBin *= 0.99;
}


double getWaveEnergy() {
  double sum = 0;
  for (int i = 0; i < nBands; i++) {
    sum += bins[i];
  }
  return sum / max(1.0, nBands * maxBin);  // Normalize to [0,1]
}


void updateVisualizer() {
  updateBarHeights();
  
}




//////////////////////////////////////////////////////////
//testing
void drawVisualizer() {
    static int visualizerMode = 0;
    static time_t lastChange = 0;

    time_t now = time(nullptr);
    if (difftime(now, lastChange) >= 60.0) { // 60 seconds passed
        visualizerMode = (visualizerMode + 1) % 10; // cycle 0–9
        lastChange = now;
    }

    switch (visualizerMode) {
        case 0: drawBars(); break;
        case 1: drawFlameBars(); break;
        case 2: drawFlameBarsFromBuffer(&flame); break;
        case 3: drawCircularWaveform(); break;
        case 4: drawWaveform(); break;
        case 5: body3(); break;
        case 6: updateSupercharged(); break;
        case 7: DRB(); break;
        case 8: drawCircularWaveformPure(); break;
        case 9: drawParticles(); break;
    }
}
/////////////////////////////////////////////////////////



/// ///////////////////////////////////
//
//
//
/////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("ESP32 HUB75 Music Visualizer Starting");

  // HUB75 pin config
  mxconfig.gpio.r1 = 17;
  mxconfig.gpio.b1 = 8;
  mxconfig.gpio.r2 = 3;
  mxconfig.gpio.b2 = 10;
  mxconfig.gpio.a = 15;
  mxconfig.gpio.c = 7;
  mxconfig.gpio.clk = 5;
  mxconfig.gpio.oe = 12;
  mxconfig.gpio.g1 = 18;  
  mxconfig.gpio.g2 = 2;  
  mxconfig.gpio.e = 13;  
  mxconfig.gpio.b = 11;
  mxconfig.gpio.d = 4;
  mxconfig.gpio.lat = 6;

  // Initialize display
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  if (!dma_display->begin()) {
    Serial.println("Display initialization failed!");
    while (true);
  }
  dma_display->clearScreen();
  dma_display->setBrightness(128); // Adjust brightness (0-255)
  
  // Test display with a green screen
  //dma_display->fillScreen(dma_display->color565(0, 255, 0));
  //dma_display->flipDMABuffer();
  //delay(1000);
  dma_display->clearScreen();
  dma_display->flipDMABuffer();
  Serial.println("Display initialized");

  // Initialize I2S
  setupI2S();
  Serial.println("I2S initialized");


  ////////////////////////////////////////
  // Initialize bar arrays
  for (int i = 0; i < nBands; i++) {
    barHeight[i] = 0;
    peakHold[i] = 0;
  }

   logo();
  
  initFlame(&flame);
  defineShapes(); //called before initBodies
  initBodies();
   
//////////////////////////////////////////////


  Serial.println("Visualizer ready!");
}










//////////////////////////////////////////////
void loop() {

  
 


 readMicrophone();
  applyWindow();
  FFT.dcRemoval();
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  calculateBands();
  updateFlame(&flame);        // procedural fire
  updateVisualizer();         // audio-reactive logic
  drawVisualizer();           // visual output

  delay(20);
  
}
/////////////////////////////////////////////