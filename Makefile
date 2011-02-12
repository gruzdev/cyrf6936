ifneq ($(KERNELRELEASE),)
EXTRA_CFLAGS += -Wall
EXTRA_CFLAGS += -DDEBUG

obj-m += cyrf6936.o
else
KDIR := /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=`pwd`
endif