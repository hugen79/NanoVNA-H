
# Use CubeMX to modify the pin configuration ~/git/NanoVNA-H4/NANOVNA_STM32_F303/cfg/cfg.ioc

# For cube2chibi installed, download the project at
https://github.com/cburlacu/cube2chibi

# For CubeMX, download from ST Micros wrbsite.
https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-configurators-and-code-generators/stm32cubemx.html

# For ChibiStudio, download from ChibiOS website.
https://osdn.net/projects/chibios/downloads/71342/ChibiStudio_Linux_Preview2.7z/
And refer to
NanoVNA-H4/doc/kickstart_ChibiStudio.txt


# At CubeMX, pick the MCU and assign pin functions.  Save the project ioc file.

# Generate chcfg (XML) file from ioc file.
python ~/git/cube2chibi/cube2chibi.py --ioc ~/git/NanoVNA-H4/NANOVNA_STM32_F303/cfg/cfg.ioc --cube /usr/local/STMicroelectronics/STM32Cube/STM32CubeMX --output ~/git/NanoVNA-H4/NANOVNA_STM32_F303/cfg/board.chcfg

# Use ChibiStudio to open the chcfg file to modify the board name and board ID, and GPIO_output speed to maximum.  Change SUBTYPE_STM32F3xx to STM32F303xC.
board_name: NanoVNA-H 4
board_id: NANOVNA_STM32_F303
subtype: STM32F303xC
Or just run this command at shell.
sed 's/<board_name>.*<\/board_name>/<board_name>NanoVNA-H 4<\/board_name>/g' board.chcfg | sed 's/<board_id>.*<\/board_id>/<board_id>NANOVNA_STM32_F303<\/board_id>/g' | sed 's/<subtype>.*<\/subtype>/<subtype>STM32F303xC<\/subtype>/g' | cat > board.chcfg.fixed

# Run fmpp, which is included at ChibiStudio.
cd ~/git/NanoVNA-H4/NANOVNA_STM32_F303/cfg
~/Downloads/ChibiStudio_Linux_Preview2/ChibiStudio/tools/fmpp/bin/fmpp -C board.fmpp

# Fix the board.mk
# Run 'git difftool board.h'
#  Remove  pins (PC0-PC12, PF0-15) at board.h
#  Refer to the checked in board.h file, change the following
#  GPIOC_I2S_DOUT
#  GPIOD
#  At "External declarations", add USB bus activation/de-activation macro

# Keep original board.mk

