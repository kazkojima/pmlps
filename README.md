# PMLPS : Poor Man's Local Positioning System

Local positioning system using OpenMV cam M7. The blob data is sent to the host PC with UDP by ESP32 connected to cam via SPI.

This program uses libvsr Geometric Algebra library:
http://versor.mat.ucsb.edu/
Setup libvsr first. Makefile assumes that pslps and versor(libvsr) directories are in the same parent directory.

Caveat: Everything is experimental.

[OpenVM cam M7] <-- SPI --> [ESP32] <-- UDP/WiFi --> [Host PC]

SPI connection is here:

| OpenVM/M7(master) | ESP32(slave)    |
| ----------------- |:---------------:|
| P0(MOSI)          | IO23(MOSI)      |
| P1(MISO)          | IO19(MISO)      |
| P2(SCK)           | IO18(CLK)       |
| P3(NSS)           | IO5(CS)         |
| P4(HANDSHAKE)     | IO22(HANDSHAKE) |

These ESP32 IO pins are default and configurable when building esp32 firmware.

![picture of SPI connection](https://github.com/kazkojima/pmlps/blob/junkyard/images/spiconn.png)

The 3 directories below are corresponding to the above 3 componets, OpenVM cam M7, ESP32 and HOst PC.

openmv/:
  led_maker_tracking.py which is a MicroPython script for openmv cam.

esp32/:
  sender application on esp-idf. Require esp-idf environment. Configure with 'make menuconfig' for your Wi-Fi setting.

host/:
  pmlps host program. Currently send VISION_POSITION_DELTA mavlink messages to tcp:192.168.11.1:5900 which is assumed to be the telemetry port of arducopter.
  Highly experimental.

There is a tiny article for this positioning system:
https://kazkojima.github.io/pmlps-en.html

##  optional daughter board

daughter-hardware/ is a optional directory for a simple ESP32 daughter board for OpenMV cam M7 instead of usual ESP32 devkit boards. See daughter-hardware/ for its KiCad files and others. It's a simple ESP-WROOM-32 board with the connector for ST's VL53L1X breakout.

If you enable VL53L1X_ENABLE during configuring ESP-WROOM-32 firmware in esp32/, you have to make a symbolic link esp32/main/symlink-STM32CubeExpansion_53L1A1_V1.0.0 to ST's driver tree. See 
https://kazkojima.github.io/esp32-vl53l1x.html
for detail.



