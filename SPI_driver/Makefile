IDIR =/home/joao/embedded_project/raspberry_pi_SPI_driver/SPI_driver/libraries/RFM69registers.h
ifneq ($(KERNELRELEASE),)
# Called by kbuild
obj-m += rfm69SPIdriver.o

else # Called from command line
KDIR := $(HOME)/buildroot/buildroot-2017.02.5/output/build/linux-1ebe8d4a4c96cd6a90805c74233a468854960f67
#Buildtree path
PWD := $(shell pwd)
CROSS= $(HOME)/buildroot/buildroot-2017.02.5/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabihf-
default:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=arm CROSS_COMPILE=$(CROSS) -I $(IDIR) modules
endif # End kbuild check

clean:
	rm -f -r *.o *.ko .*cmd .tmp* core *.i
