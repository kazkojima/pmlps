# PMLPS : Poor Man's Local Positioning System

Local positioning system using OpenMV cam M7. The blob data send the host PC with UDP by ESP32 connected to cam via SPI.
This program uses libvsr Geometric Algebra library:
http://versor.mat.ucsb.edu/
Setup libvsr first. Makefile assumes that pslps and versor(libvsr) directories are in the same parent directory.

Caveat: Everything is experimental.

[OpenVM cam M7] -- SPI -- [ESP32] -- UDP/WiFi -- [Host PC]

openmv/:
  led_maker_tracking.py which is a MicroPython script for openmv cam.

esp32/:
  sender applicarion on esp-idf. Require esp-idf environment. Configure with 'make menuconfig' for your Wi-Fi setting.

host/:
  pmlps host program.

