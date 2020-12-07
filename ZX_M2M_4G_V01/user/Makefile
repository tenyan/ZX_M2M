ifneq ($(CROSS_COMPILE),)
CROSS-COMPILE:=$(CROSS_COMPILE)
endif

#export PATH=$PATH:/usr/a5buildroot/buildroot-at91/output/host/usr/bin
#export CROSS_COMPILE=arm-buildroot-linux-uclibcgnueabihf-

#CROSS-COMPILE:=/usr/a5buildroot/buildroot-at91/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabihf-

#CROSS-COMPILE:=/workspace/buildroot/buildroot-qemu_mips_malta_defconfig/output/host/usr/bin/mips-buildroot-linux-uclibc-
#CROSS-COMPILE:=/workspace/buildroot/buildroot-qemu_arm_vexpress_defconfig/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabi-
#CROSS-COMPILE:=/workspace/buildroot-git/qemu_mips64_malta/output/host/usr/bin/mips-gnu-linux-
CROSS-COMPILE:=arm-none-linux-gnueabi-
CC:=$(CROSS-COMPILE)gcc
LD:=$(CROSS-COMPILE)ld


release: clean
	$(CC) -Wall -s app.c amx8xx.c BaseFun.c CollectCompositelayer.c CollectHW.c CollectInterfaceLayer.c GpsDataCalc.c GpsHardWareLayer.c GpsInterfaceLayer.c GpsIntergrateLayer.c GpsProtocolLayer.c GsmHardWareLayer.c GsmIntegrateLayer.c GsmInterfaceLayer.c GsmProtocolLayer.c IAP.c Mcu.c McuHW.c McuUpload.c MX25L1606Mem.c PcDebug.c PcHardware.c rtc.c SystemCommand.c SystemProtocol.c SystemWorkMode.c gpio_cfg.c Watchdog.c A5Uart0_MCU.c WorkStatistcs.c HuanBaoProtocol.c Canconfig.c -lm -o helloworld -lpthread -ldl

debug: clean
	$(CC) -Wall -g app.c amx8xx.c BaseFun.c CollectCompositelayer.c CollectHW.c CollectInterfaceLayer.c GpsDataCalc.c GpsHardWareLayer.c GpsInterfaceLayer.c GpsIntergrateLayer.c GpsProtocolLayer.c GsmHardWareLayer.c GsmIntegrateLayer.c GsmInterfaceLayer.c GsmProtocolLayer.c IAP.c Mcu.c McuHW.c McuUpload.c MX25L1606Mem.c PcDebug.c PcHardware.c rtc.c SystemCommand.c SystemProtocol.c SystemWorkMode.c gpio_cfg.c Watchdog.c A5Uart0_MCU.c WorkStatistcs.c HuanBaoProtocol.c Canconfig.c -lm -o helloworld -lpthread -ldl

clean:
	rm -rf helloworld *~

