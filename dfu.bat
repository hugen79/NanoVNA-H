# make clean
make TARGET=F303
lsusb
st-flash write build/ch.bin 0x8000000
