#!/bin/sh
cp build/ch.bin release/NanoVNA-H4_$(date +"%Y%m%d").bin
cp build/ch.hex release/NanoVNA-H4_$(date +"%Y%m%d").hex
dfu-tool convert dfuse build/ch.bin release/NanoVNA-H4_$(date +"%Y%m%d").dfu
dfu-tool set-vendor release/NanoVNA-H4_$(date +"%Y%m%d").dfu 0483
dfu-tool set-release release/NanoVNA-H4_$(date +"%Y%m%d").dfu 0000
#dfu-tool set-product release/NanoVNA-H4_$(date +"%Y%m%d").dfu 0000
dfu-tool set-address release/NanoVNA-H4_$(date +"%Y%m%d").dfu 0x8000000
dfu-tool set-alt-setting-name release/NanoVNA-H4_$(date +"%Y%m%d").dfu NanoVNA-H4

