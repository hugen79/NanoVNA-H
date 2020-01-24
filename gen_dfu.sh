#!/bin/sh
cp build/ch.bin release/NanoVNA-H4_$(date +"%Y%m%d").bin
cp build/ch.hex release/NanoVNA-H4_$(date +"%Y%m%d").hex
dfu-tool convert raw build/ch.bin release/NanoVNA-H4_$(date +"%Y%m%d").dfu

