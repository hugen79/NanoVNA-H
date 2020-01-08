NanoVNA-H 4 - handheld Vector Network Analyzer 
==========================================================


## About

NanoVNA 是高桥知宏（[edy555](https://github.com/edy555)）先生创建的非常小巧的手持矢量网络分析仪项目，非常低的成本，方便制造并且提供了不错的测量精度。高桥先生对NanoVNA的软件采用了GPL协议开源，旨在吸引更多的年轻人关注射频与无线电。我在对高桥先生的软件进行简单的修改使其可以使用奇次谐波进行测量，并制作了可以测量到900MHz的版本在社区分享，受到了社区的广泛关注。后来我便开始批量制作了NanoVNA，在高桥先生的建议下，我将我制作的版本重命名为NanoVNA-H。社区开始为NanoVNA贡献新的代码，新的功能被添加，[cho45](https://github.com/cho45)为NanoVNA增加了TDR和截图功能，[QRP73](https://github.com/qrp73)帮助改进了Si5351和AIC3204的驱动。还有更多的功能需求被提出，STM32F072CBT6的资源就不够了，于是在[https://groups.io/](https://groups.io/)的一次讨论中廖先生（[AA6KL](https://github.com/AA6KL)）提出了使用STM32F303CCT6替代STM32F072CBT6的想法。我为廖先生提供了安装有STM32F303CCT6的NanoVNA-H测试版，得益于高桥先生早期的工作，移植比较顺利。由于STM32F303CCT6具有更快的SPI总线速度，我们开始考虑社区广泛抱怨的显示尺寸问题，经过一些选择和测试，最终我们使用了4英寸液晶屏。

经过Issues # #14的讨论，我们现在已经将NanoVNA的测量频率提升至1.5G。由于采用更大的屏幕，我们现在有更多的PCB空间用于安装元件和电池，因此现在我们使用独立的电荷泵为混频器供电并增加了钽电容，可以进一步降低电源噪声，1950mAh的电池可以独立使用8个小时以上。为4.0英寸全视角液晶屏增加了DC调光，室外使用时可以看的更加清晰室内使用更舒适。

为了方便极客们创造更多的可能，PCB上引出了UART1和PC14，PC15。得益于NanoVNA-H的制造经验，我们为NanoVNA-H 4制作了新的ABS外壳，可以有效的保护SMA接头和按键，我们向社区出售的每一套NanoVNA-H 4都安装了完整的ABS外壳，并被包装在精美的礼品盒中，可以保证长途快递不会损坏。
NanoVNA-H 4与之前版本的NanoVNA保持了高度的软件兼容性，所有为NanoVNA开发的软件都可以直接使用，仅仅需要注意的是，由于使用了更大的液晶屏，“capture”命令返回的数据长度是原来的两倍，开发者可以很容易的修改代码实现兼容。

 我在[gen111.taobao.com](https://gen111.taobao.com/)出售安装完整的NanoVNA，如果您是海外用户，您可以通过[alibaba](https://www.alibaba.com/product-detail/4-2-version-1950mAh-battery-Original_62455845943.html)购买我制作的硬件。您也可以自己参考[原理图](https://github.com/hugen79/NanoVNA-H/blob/F303/doc/Schematic_NanoVNA-H4_2.pdf)设计您自己的PCB，需要指出的是模拟电路部分屏蔽与隔离都是重要的，电桥部分是经过严格匹配的，如果你需要使用谐波扩展300M以上测量需要尤为注意。

NanoVNA is a very compact and portable vector network analyzer created by Mr. TAKAHASHI Tomohiro([edy555](https://github.com/edy555)),  which cost is very low.  It is easy to manufacture and provide good measurement accuracy. Mr. Takahashi uses the GPL protocol to open source the NanoVNA software, aiming to attract more young people to pay attention to radio frequency and wireless.
Based on Mr. Takahashi's software, I made simple modification to make it measure with odd harmonic. What's more, I have made a version that can measure 900MHz and shared it in the community, which has received widespread attention from the community.  Later on,  I started  to mass produce NanoVNA.  At the suggestion of Mr. Takahashi, I renamed my version NanoVNA-H. The community began to contribute new code for NanoVNA,  new features were added, [cho45](https://github.com/cho45) added TDR and screenshot functions to NanoVNA, and [QRP73](https://github.com/qrp73) helped to improve the driver of Si5351 and AIC3204.  With more functional requirements being raised,  the resources of STM32F072CBT6 are not enough, therefore, under a discussion with Mr. Liao([AA6KL](https://github.com/AA6KL))  at https://groups.io/, he proposed the idea to replace STM32F303CCT6 with STM32F072CBT6. I provided Mr. Liao a NanoVNA-H test version with STM32F303CCT6.  Thanks to Mr. Takahashi's early work, the porting was smooth.  Because the STM32F303CCT6 has a faster SPI bus speed, we started to consider the display size problem which is widely complained by the community. After some selection and testing, we finally used a 4-inch LCD screen. 

After discussion with Issues # # 14, we have now increased the measurement frequency of NanoVNA to 1.5GHz.  With the larger screen, we have more PCB space for components and batteries. Therefore, we use a separate charge pump to power the mixer and add tantalum capacitors, which can further reduce power supply noise, and the 1950mAh battery can be used independently for more than 8 hours. We add DC dimming to the 4.0 inch full-view LCD screen, which make the view more clearly when used outdoors and more comfortable when used indoors.

 In order to make it easier for geeks to create more possibilities, UART1, PC14, and PC15 are introduced on the PCB.  Thanks to the manufacturing experience of NanoVNA-H, we made a new ABS shell for NanoVNA-H 4, which can effectively protect SMA connectors and switchs. Each set of NanoVNA-H 4 sold to the community is installed with a complete ABS shell. It is packed in a beautiful gift box, which can surely protect the NanoVNA-H 4 without being damaged during the long-distance express transportation. NanoVNA-H 4 maintains high degree of software compatibility with previous versions of NanoVNA. All software developed for NanoVNA can be used directly. Just note that with the larger LCD screen, the data returned by the “capture” command is twice as long as the original one, but the developers can easily modify the code for compatibility.

I'm selling the full NanoVNA at [gen111.taobao.com](https://gen111.taobao.com/), if you are an overseas user, you can purchase the hardware I made through [alibaba](https://www.alibaba.com/product-detail/4-2-version-1950mAh-battery-Original_62455845943.html). If you need to trade with PayPal, please contact [Maggie](https://message.alibaba.com/msgsend/contact.htm?action=contact_action&domain=1&id=62455845943&id_f=IDX1dHNHLwwpZgcoe2ZOT8dE0Xz1IdgXyqJKI53BZbaeyOULeGYMG8w0clKRkrrM8Z9p&mloca=main_en_detail&tracelog=tracedetailfeedback&umidToken=Ba90d7fcce8729cf01f81a9af091ea659) through Alibaba. you can also design your own PCB by [reference schematic](https://github.com/hugen79/NanoVNA-H/blob/F303/doc/Schematic_NanoVNA-H4_2.pdf), it should be noted that analog circuit partial shielding and isolation are important, the bridge part is strictly matched, if you need to use harmonic extension 300M or more measurements need to pay special attention. 



## Build firmware

It is recommended to compile with gcc-arm-none-eabi 8, and exceptions may occur with other versions of the compiler. Please sync the CibiOS submodule before compiling.

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

#### Compile

```
$ make
```



### Windows

Follow [these instructions](https://gnu-mcu-eclipse.github.io/install/)to install gnu-mcu-eclipse.

Existing Code as Makefile Projiect. Project > Properties > C/C++ Build > Setting: Confirm that Toolchains is "GNU MCU Eclipse ARM Embedded GCC (arm-none-eabi-gcc)"

Project > Properties > C/C++ Build > Tool Chain Editor:

```
Current toolchain: ARM Cross GCC
Current builder: Gnu Make Builder
```

#### Compile

just Build Project.

#### Debug use Eclipse + cmsis-dap +openocd

Debugger Configurations > GBD OpenOCD Debugging, Double click to create a new setting, Select “Debugger“ label, add config option.

```
-f NanoVNA_DAP.cfg
```



## Credit

Thanks [@edy555](https://github.com/ttrftech/NanoVNA) for creating this tiny feature rich gadget.  edy555 is developing a next-generation VNA with higher performance.

Thanks  [@AA6KL](https://github.com/AA6KL/NanoVNA-F303) for porting the hardware.  