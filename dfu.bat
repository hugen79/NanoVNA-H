# make clean
make TARGET=F303
lsusb
st-flash write build/ch.bin 0x8000000
#st-flash write release/NanoVNA-H4_20200124.dfu 0x8000000
