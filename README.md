NanoVNA - Very tiny handheld Vector Network Analyzer
==========================================================

[![GitHub release](http://img.shields.io/github/release/ttrftech/NanoVNA.svg?style=flat)][release]
[![CircleCI](https://circleci.com/gh/ttrftech/NanoVNA.svg?style=shield)](https://circleci.com/gh/ttrftech/NanoVNA)

[release]: https://github.com/ttrftech/NanoVNA/releases

<div align="center">
<img src="/doc/nanovna.jpg" width="480px">
</div>


# About

This is my attemp to add the STM32F303CCT6 to this popular NanoVNA project.  The befit is larger SRAM 40KB from 16KB, and larger flash 256KB from 128KB.  For detail information, please refer to the original project page.
https://github.com/ttrftech/NanoVNA

# Compile

To compile for STM32F303CCT6, add TARGET=F303

   $ make TARGET=F303

Since F303 doesn't support crystall-less DFU, a separate ST_Link is required to flash the firmware.  Users needs to solder the pins on the NanoVNA SWD port.

To flash firmware at Linux, refer to

https://github.com/texane/stlink

The flash firmware command at Linux(Ubuntu) is

    $ st-flash write build/ch.bin 0x8000000

Rememebr to short the BOOT pin to VCC at NanoVNA board for firmware update.

At Ubuntu, the shell can be accessed via

   $ sudo screen /dev/ttyACM0 19200

Till now, the ADC porting is not done yet.  So the touch function doesn't work.  I will update the tree as soon as I finish the ADC porting.

Thanks hugen79 for providing the hardware for my porting work and debugging support.  For hardware mod information, please visit his page at

https://github.com/hugen79/NanoVNA-H

# Hardware mod candidate list:

- Add D2 Schottky diode to measure the battery.  Easy.
- Replace STM32F072CBT6 to STM32F303CCT6.  Add 1.5K resistor between VDD and USB_DP.  Harder.
- Add 8MHz chrystal to PCB.  Better with PCB revision.
