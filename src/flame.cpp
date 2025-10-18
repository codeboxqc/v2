#include <Arduino.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
 
#include "bar.h" 


Flame flame;
 
 

 

void initFlame(Flame* f) {
  f->width = WIDTH;
  f->height = HEIGHT;
  f->buffer = (uint8_t*)malloc(f->width * f->height);
  f->fire_buffer = (uint8_t*)malloc(f->width * f->height);
  memset(f->buffer, 0, f->width * f->height);
  memset(f->fire_buffer, 0, f->width * f->height);
}

void updateFlame(Flame* f) {
  for (int x = 2; x < WIDTH - 2; x += random(4, 7)) {
    if (random(3) == 0) {
      for (int dx = 0; dx < 10; dx++) {
        int y = HEIGHT - 1;
        f->buffer[y * WIDTH + x + dx] = 255;
      }
    }
  }

  for (int y = 1; y < HEIGHT; y++) {
    for (int x = 1; x < WIDTH - 1; x++) {
      int idx = y * WIDTH + x;
      int avg = (f->buffer[idx] +
                 f->buffer[idx - 1] +
                 f->buffer[idx + 1] +
                 f->buffer[idx - WIDTH]) >> 2;
      int newY = y - 1;
      f->fire_buffer[newY * WIDTH + x] = avg > 2 ? avg - 2 : 0;
    }
  }

  memcpy(f->buffer, f->fire_buffer, WIDTH * HEIGHT);
  memset(f->fire_buffer, 0, WIDTH * HEIGHT);
}

void drawFlameBarsFromBuffer(Flame* f) {
  dma_display->clearScreen();

  int barWidth = PANEL_RES_X / nBands;
  int spacing = 1;

  for (int band = 0; band < nBands; band++) {
    int x = band * barWidth;
    int height = (int)barHeight[band];

    for (int y = 0; y < height; y++) {
      int flameY = HEIGHT - 1 - y;
      for (int barX = x + spacing; barX < x + barWidth - spacing; barX++) {
        int flameX = map(barX, 0, PANEL_RES_X, 0, f->width - 1);
        uint8_t v = f->buffer[flameY * f->width + flameX];

        uint8_t r, g, b;
        if (v == 0) {
          r = g = b = 0;
        } else if (v < 85) {
          r = v * 3; g = 0; b = 0;
        } else if (v < 170) {
          r = 255; g = (v - 85) * 3; b = 0;
        } else {
          r = 255; g = 255; b = (v - 170) * 3;
        }

        int yPos = PANEL_RES_Y - 1 - y;
        if (barX >= 0 && barX < PANEL_RES_X && yPos >= 0) {
          dma_display->drawPixelRGB888(barX, yPos, r, g, b);
        }
      }
    }
  }

  dma_display->flipDMABuffer();
}

 