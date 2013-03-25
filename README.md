gr-ssrptx
=========

GNURadio Simple Software Radio Peripherals HF-VHF Transmitter Hardware Module

This is software module for small board, based on Cypress FX2 microcontroller and AD9957 RF transmitter frontend. It able to create signal in HF-VHF bands. In current register configuration maximal frequency is 206MHz - with inverse sinc filter, and 250MHz - without that filter (it's recommended to turn that filter on to make clear signal).

The precompiled firmware for FX2 (txS_1024.ihx) is already included. You also don't need any libraries, except usb. The software tested on Linux environment.

You will have 1638400 Hz bandwidth for your signal. The samplerate is fixed to 2048000 complex samples per second. The input format - is "sc16" - short complex 16 bit. More info at http://tipok.org.ua/node/41

To build this project, you need this tools:
------------

* [GNURadio](http://gnuradio.org)


