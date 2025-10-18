🌌 3-Body Orbital Visualizer with Waveform Modulation
A real-time visual instrument for ESP32-S3 + HUB75 panels. This sketch simulates 3-body orbital physics with expressive trails, shape transitions, and waveform-reactive distortion. Designed for 64×64 LED panels using DMA rendering.

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
