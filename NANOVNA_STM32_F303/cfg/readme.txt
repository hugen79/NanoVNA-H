
#Use BubeMX to modify the pin configuration ~/git/NanoVNA-H4/NANOVNA_STM32_F303/cfg/cfg.ioc

# generate chcfg file from ioc
python ~/git/cube2chibi/cube2chibi.py --ioc ~/git/NanoVNA-H4/NANOVNA_STM32_F303/cfg/cfg.ioc --cube /usr/local/STMicroelectronics/STM32Cube/STM32CubeMX --output ~/git/NanoVNA-H4/NANOVNA_STM32_F303/cfg/board.chcfg

# Use ChibiStudio to open the chcfg file to modify the board name and board ID, and GPIO_output speed to maximum.  Change SUBTYPE_STM32F3xx to STM32F303xC.
board_name: NanoVNA-H 4
board_id: NANOVNA_STM32_F303
subtype: STM32F303xC
run this one:
sed 's/<board_name>.*<\/board_name>/<board_name>NanoVNA-H 4<\/board_name>/g' board.chcfg | sed 's/<board_id>.*<\/board_id>/<board_id>NANOVNA_STM32_F303<\/board_id>/g' | sed 's/<subtype>.*<\/subtype>/<subtype>STM32F303xC<\/subtype>/g' | cat > board.chcfg.fixed


#run fmpp
cd ~/git/NanoVNA-H4/NANOVNA_STM32_F303/cfg
~/Downloads/ChibiStudio_Linux_Preview2/ChibiStudio/tools/fmpp/bin/fmpp -C board.fmpp

#Fix the board.mk
#remove unused pins (PC0-PC12, PF0-15) at board.h
# refer to golden, change the following
#  GPIOC_I2S_DOUT
#  GPIOD
#  /* External declarations.                                                    */
#  Add USB bus activation/de-activation macro

#Keep original board.mk

