obj-m := panel_io_driver.o
KERNELDIR ?= $(shell pwd)/../../edd-linux-2.6.29.6
PWD :=$(shell pwd)
CPPFLAGS += -I/opt/eldk42/ppc_82xx/usr/include
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
	cp panel_io_driver.ko  /tftpboot/
clean:
	rm -rf *.o  *.ko *.mod.c *.order *.symvers
