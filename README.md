NanoVNA-H 4 - handheld Vector Network Analyzer 
==========================================================


# About

NanoVNA 是高桥知宏（edy555）先生创建的非常小巧的手持矢量网络分析仪项目，非常低的成本，方便制造并且提供了不错的测量精度。高桥先生对NanoVNA的软件采用了GPL协议开源，旨在吸引更多的年轻人关注射频与无线电。我在对高桥先生的软件进行简单的修改使其可以使用奇次谐波进行测量，并制作了可以测量到900MHz的版本在社区分享，受到了社区的广泛关注。后来我便开始批量制作了NanoVNA，在高桥先生的建议下，我将我制作的版本重命名为NanoVNA-H。社区开始为NanoVNA贡献新的代码，新的功能被添加，cho45为NanoVNA增加了TDR和截图功能，QRP73帮助改进了Si5351和AIC3204的驱动。还有更多的功能需求被提出，STM32F072CBT6的资源就不够了，于是在[https://groups.io/](https://groups.io/)的一次讨论中廖先生（AA6KL）提出了使用STM32F303CCT6替代STM32F072CBT6的想法。我为廖先生提供了安装有STM32F303CCT6的NanoVNA-H测试版，得益于高桥先生早期的工作，移植比较顺利。由于STM32F303CCT6具有更快的SPI总线速度，我们开始考虑社区广泛抱怨的显示尺寸问题，经过一些选择和测试，最终我们使用了4英寸液晶屏。
经过Issues # #14的讨论，我们现在已经将NanoVNA的测量频率提升至1.5G。由于采用更大的屏幕，我们现在有更多的PCB空间用于安装元件和电池，因此现在我们使用独立的电荷泵为混频器供电并增加了钽电容，可以进一步降低电源噪声，1950mAh的电池可以独立使用8个小时以上。为4.0英寸全视角液晶屏增加了DC调光，室外使用时可以看的更加清晰室内使用更舒适。
为了方便极客们创造更多的可能，PCB上引出了UART1和PC14，PC15。得益于NanoVNA-H的制造经验，我们为NanoVNA-H 4制作了新的ABS外壳，可以有效的保护SMA接头和按键，我们向社区出售的每一套NanoVNA-H 4都安装了完整的ABS外壳，并被包装在精美的礼品盒中，可以保证长途快递不会损坏。
NanoVNA-H 4与之前版本的NanoVNA保持了高度的软件兼容性，所有为NanoVNA开发的软件都可以直接使用，仅仅需要注意的是，由于使用了更大的液晶屏，“capture”命令返回的数据长度是原来的两倍，开发者可以很容易的修改代码实现兼容。



Thanks [@edy555](https://github.com/edy555) for creating this tiny feature rich gadget.  edy555 is also working on the F303 version of NanoVNA plus other new features, probably named NanoVNA-V2.

Thanks  [@AA6KL](https://github.com/AA6KL) for porting the hardware.  