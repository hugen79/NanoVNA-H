NanoVNA - Very tiny handheld Vector Network Analyzer
==========================================================
DIY的矢量网络分析仪，原项目地址[https://github.com/ttrftech/NanoVNA](https://github.com/ttrftech/NanoVNA),修改了部分电路，增加了电池管理电路，重新设计了PCB。改进了的频率算法，可以利用si5351的奇次谐波扩展支持到900MHz的测量频率，设计了金属屏蔽片，可以减少外部干扰提高测量精度，si5351直接输出的50K-300MHz频段提供优于70dB的动态，扩展的300M-600MHz波段可以提供优于50dB的动态，600M-900M波段优于40dB的动态。
起先我在38hot分享我的修改，但是后来抄板的商家无耻的宣称是他们自己修改了设计，我非常气愤，并一度暂停了相关的分享，现在重新把修改后的代码分享到github，希望这样能够方便参与改进NanoVNA的爱好者，而不是那些粗制滥造的抄板者。
我在gen111.taobao.com出售安装完整的NanoVNA，如果您是海外用户，您可以通过[alibaba]( https://www.alibaba.com/product-detail/Hugen-NanoVNA-H-New-item-Original_62342877955.html )购买我制作的硬件。您也可以自己参考[原理图](doc/Schematic_nanovna-H_Sheet-1_20190902.png)设计您自己的PCB，需要指出的是模拟电路部分屏蔽与隔离都是重要的，电桥部分是经过严格匹配的，如果你需要使用谐波扩展300M以上测量需要尤为注意。在网络商发现的一些克隆品为了简化安装都错误的进行了修改，这样做是可能导致800M左右的动态小于20dB，失去了测了测量的意义。
另外设计了简单实用的PC控制软件NanoVNASharp，可以通过PC端软件导出Touchstone(snp)文件用于各种无线电设计和仿真软件，NanoVNASharp是一个单独的项目，暂无开源计划。

We remade NanoVNA based on edy555 (https://github.com/ttrftech/NanoVNA) , but  modified some circuits, added battery management circuits, and redesigned the PCB. The improved frequency algorithm can use the odd harmonic extension of si5351 to support the measurement frequency up to 900MHz. The metal shield is designed to reduce the external interference and improve the measurement accuracy. The 50k-300MHz frequency range of the si5351 direct output provides better than 70dB dynamic. The extended 300M-600MHz band provides better than 50dB of dynamics, and the 600M-900M band is better than 40dB of dynamics.
I share my changes on the forum(http://bbs.38hot.net/thread-756047-1-1.html) and provide the original code to users who have purchased my hardware. But a bad clone appeared and claimed to have made improvements by themselves. I was very angry and once suspended the relevant sharing. Now I will share the modified code to github, hoping to participate in the improvement of NanoVNA enthusiasts. Not those bad clones.
I'm selling the full NanoVNA at gen111.taobao.com, if you are an overseas user, you can purchase the hardware I made through [alibaba]( https://www.alibaba.com/product-detail/Hugen-NanoVNA-H-New-item-Original_62342877955.html). you can also design your own PCB by [reference schematic](doc/Schematic_nanovna-H_Sheet-1_20190902.png), it should be noted that analog circuit partial shielding and isolation are important, the bridge part is strictly matched, if you need to use harmonic extension 300M or more measurements need to pay special attention. Some clones found in the network have been modified incorrectly in order to simplify the installation, which may result in a dynamic of about 800M less than 20dB, which has lost the significance of measuring.
NanoVNASharp is a PC control software can export Touchstone (snp) files which can be used for various radio design and simulation software. NanoVNASharp is a separate project with no open source plans.



![](D:\works\c\NanoVNA-H\doc\NanoVNA-H-2.jpg)


## 编译
## Build firmware
It is recommended to compile with gcc-arm-none-eabi 8, and exceptions may occur with other versions of the compiler.
Please sync the CibiOS submodule before compiling.

```
$ git submodule update --init --recursive
```



### MacOSX

Install cross tools and firmware updating tool.

```
$ brew tap px4/px4
$ brew install gcc-arm-none-eabi-80
$ brew install dfu-util

```

### Linux

Download arm cross tools from [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).

```
$ wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
$ sudo tar xfj -C /usr/local gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
$ PATH=/usr/local/gcc-arm-none-eabi-8-2018-q4-major/bin:$PATH
$ sudo apt install -y dfu-util
```


### Normal version
```
$ make
```

### AA version
```
$ make ANTENNA_ANALYZER=YES
```



### Windows
Follow [these instructions](https://gnu-mcu-eclipse.github.io/install/)to install gnu-mcu-eclipse.

Existing Code as Makefile Projiect.
Project > Properties > C/C++ Build > Setting:
Confirm that Toolchains is "GNU MCU Eclipse ARM Embedded GCC (arm-none-eabi-gcc)"

Project > Properties > C/C++ Build > Tool Chain Editor:
```
Current toolchain: ARM Cross GCC
Current builder: Gnu Make Builder
```

### Normal version
 just Build Project.

### AA version
Project > Properties > C/C++ Build >  Environment >Add:
```
Name: ANTENNA_ANALYZER
Value: YES
```

Build Project.

### Debug use  Eclipse + cmsis-dap +openocd
Debugger Configurations > GBD OpenOCD Debugging, Double click to create a new setting, Select “Debugger“ label, add  config option.
```
-f NanoVNA_DAP.cfg
```



感谢edy555创建了这个项目，所有软件版权归edy555所有。
 Thanks to edy555 for creating this project, all software copyrights are owned by edy555
[https://github.com/ttrftech/NanoVNA](https://github.com/ttrftech/NanoVNA)；
感谢cho45对项目做出重大改进。
Thanks to cho45 for making major improvements to the project. [https://github.com/cho45/NanoVNA](https://github.com/cho45/NanoVNA) 

以下为原项目自述
==========================================================
The following is the original project readme
==========================================================
# About

NanoVNA is very tiny handheld Vector Network Analyzer (VNA). It is standalone with lcd display, portable device with battery. This project aim to provide an RF gadget but useful instrument for enthusiast.

This repository contains source of NanoVNA firmware.

## [](https://github.com/ttrftech/NanoVNA#prepare-arm-cross-tools)Prepare ARM Cross Tools

**UPDATE**: Recent gcc version works to build NanoVNA, no need old version.

### [](https://github.com/ttrftech/NanoVNA#macosx)MacOSX

Install cross tools and firmware updating tool.

```
$ brew tap px4/px4
$ brew install gcc-arm-none-eabi-80
$ brew install dfu-util

```

### [](https://github.com/ttrftech/NanoVNA#linux-ubuntu)Linux (ubuntu)

Download arm cross tools from [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).

```
$ wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
$ sudo tar xfj -C /usr/local gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2
$ PATH=/usr/local/gcc-arm-none-eabi-8-2018-q4-major/bin:$PATH
$ sudo apt install -y dfu-util

```

## [](https://github.com/ttrftech/NanoVNA#fetch-source-code)Fetch source code

Fetch source and submodule.

```
$ git clone https://github.com/ttrftech/NanoVNA.git
$ cd NanoVNA
$ git submodule update --init --recursive

```

## [](https://github.com/ttrftech/NanoVNA#build)Build

Just make in the directory.

```
$ make

```

### [](https://github.com/ttrftech/NanoVNA#build-firmware-using-docker)Build firmware using docker

Using [this docker image](https://cloud.docker.com/u/edy555/repository/docker/edy555/arm-embedded) without installing arm toolchain.

```
$ cd NanoVNA
$ docker run -it --rm -v $(PWD):/work edy555/arm-embedded:8.2 make

```

## [](https://github.com/ttrftech/NanoVNA#flash-firmware)Flash firmware

First, make device enter DFU mode by one of following methods.

*   Jumper BOOT0 pin at powering device
*   Select menu Config->DFU (needs recent firmware)

Then, flash firmware using dfu-util via USB.

```
$ dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D build/ch.bin

```

Or simply use make.

```
$ make flash

```

## [](https://github.com/ttrftech/NanoVNA#control-from-pc)Control from PC

See [python directory](https://github.com/ttrftech/NanoVNA/blob/master/python/README.md).

## [](https://github.com/ttrftech/NanoVNA#note)Note

Hardware design material is disclosed to prevent bad quality clone. Please let me know if you would have your own unit.

## [](https://github.com/ttrftech/NanoVNA#reference)Reference

*   [Schematics](https://github.com/ttrftech/NanoVNA/blob/master/doc/nanovna-sch.pdf)
*   [PCB Photo](https://github.com/ttrftech/NanoVNA/blob/master/doc/nanovna-pcb-photo.jpg)
*   [Block Diagram](https://github.com/ttrftech/NanoVNA/blob/master/doc/nanovna-blockdiagram.png)
*   Kit available from [https://ttrf.tk/kit/nanovna](https://ttrf.tk/kit/nanovna)

## [](https://github.com/ttrftech/NanoVNA#credit)Credit

*   [@edy555](https://github.com/edy555)

### [](https://github.com/ttrftech/NanoVNA#contributors)Contributors

*   [@hugen79](https://github.com/hugen79)
*   [@cho45](https://github.com/cho45)
