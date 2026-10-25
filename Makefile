obj-m := i2c_mma8452.o

KDIR := $(HOME)/Documents/Linux-Development/Device-Drivers/linux-rpi
ARCH := arm64
CROSS_COMPILE := aarch64-linux-gnu-

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean