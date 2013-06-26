##########################
# where your kernel sources are
KERNELDIR ?= /lib/modules/`uname -r`/build

GCC = gcc
DEPMOD = /sbin/depmod

# end configuration part
##########################

PWD := $(shell pwd)

KERNELSRC ?= /usr/src/linux-headers-$(shell uname -r)
KERNELMODS ?= /lib/modules/$(shell uname -r)

EXTRA_CFLAGS = $(CMODFLAGS)

obj-m := xpad.o

all: module

module: xpad.c
	@$(MAKE) -C $(KERNELSRC) SUBDIRS=$(PWD) M=$(PWD) modules

install: module-install

module-install: xpad.ko
	install -c -o root -g root -m 644 xpad.ko \
	$(KERNELMODS)/kernel/drivers/input/joystick
	$(DEPMOD) -ae

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions *~ *.a *.so *.d
	rm -rf Module.symvers Module.markers modules.order
