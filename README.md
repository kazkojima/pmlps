# PMLPS : Poor Man's Local Positioning System

Local positioning system using OpenMV cam M7. The blob data is sent to the host PC with UDP by ESP32 connected to cam via SPI.

This program uses libvsr Geometric Algebra library:
http://versor.mat.ucsb.edu/
Setup libvsr first. Makefile assumes that pslps and versor(libvsr) directories are in the same parent directory.

Caveat: Everything is experimental.

[OpenVM cam M7] -- SPI --> [ESP32] -- UDP/WiFi --> [Host PC]

SPI conection is here:

| OpenVM/M7(master) | ESP32(slave)    |
| ----------------- |:---------------:|
| P0(MOSI)          | IO23(MOSI)      |
| P1(MISO)          | IO19(MISO)      |
| P2(SCK)           | IO18(CLK)       |
| P3(NSS)           | IO5(CS)         |
| P4(HANDSHAKE)     | IO22(HANDSHAKE) |

openmv/:
  led_maker_tracking.py which is a MicroPython script for openmv cam.

esp32/:
  sender applicarion on esp-idf. Require esp-idf environment. Configure with 'make menuconfig' for your Wi-Fi setting.

host/:
  pmlps host program. Currently send VISION_POSITION_DELTA mavlink messages to tcp:192.168.11.1:5900 which is assumed to be the telemetry port of arducopter.
  Highly experimental.
