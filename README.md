🌌 3-Body Orbital Visualizer with Waveform Modulation
A real-time visual instrument for ESP32-S3 + HUB75 panels. This sketch simulates 3-body orbital physics with expressive trails, shape transitions, and waveform-reactive distortion. Designed for 64×64 LED panels using DMA rendering.

ESP32-S3-WROOM Dual-core 32-bit 240 MHz
esp32dev compatible need fast one and 8mb memory

esp32 

hub75   led 64x64

INMP441 micro

🔧 Features
⚛️ Physics-based 3-body simulation with gravitational interactions

🔁 Smooth transitions between orbital configurations (figure-eight, triangle, linear, etc.)

🎵 Waveform-reactive trail distortion using live audio input

🌈 Color shimmer and glow modulated by amplitude and time

🧠 Optimized for ESP32-S3 with HUB75 DMA panel driver

🧪 Hardware Requirements
ESP32-S3 microcontroller

HUB75-compatible 64×64 RGB LED panel

INMP441 or similar I2S microphone (for waveform input)

External 5V power supply for panel

📦 Dependencies
ESP32-HUB75-MatrixPanel-I2S-DMA

ArduinoFFT

Arduino core for ESP32

🚀 Getting Started
Clone this repo and open 3body.cpp in Arduino IDE or PlatformIO.

Wire up your panel and mic according to the pin config in setup().

Flash to ESP32-S3 and watch the cosmos unfold.

🎨 Customization
Modify shapePos[] and shapeVel[] to create new orbital configurations.

Adjust scale to fit different panel sizes (default: 28.0 for 64×64).

Tune waveform modulation depth via yOffset = amplitude * 6.0f + sin(...).

Add Strudel or OSC triggers to morph shapes or pulse gravity.








Freenove ESP32-S3 ESP32 S3 Wireless Board, Dual-core 32-bit 240 MHz Microcontroller https://www.aliexpress.com/item/1005007758555205.html?spm=a2g0o.order_list.order_list_main.39.3f7f18027b8KoF Freenove Breakout Board for ESP32 / ESP32-S3 WROVER WROOM, Terminal Block Shield HAT, 5V 3.3V Power Outputs, GPIO Status LED https://www.aliexpress.com/item/1005005879655901.html?spm=a2g0o.order_list.order_list_main.11.3f7f18027b8KoF

16Pin Jumper Wire Dupont Line 20cm Male To Female
2 Female To Female pawer from Breakout Board to panel power up https://www.aliexpress.com/p/order/detail.html?spm=a2g0o.order_list.order_list_main.41.3f7f18027b8KoF&orderId=8204301001571878

3 wired example how i do it! do same all wired





https://buymeacoffee.com/www.nutz.club
