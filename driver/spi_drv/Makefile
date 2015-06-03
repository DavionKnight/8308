ifneq ($(KERNELRELEASE),)

obj-m := spi_drv.o

else

KDIR := ../../edd-linux-2.6.29.6
all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers *.order

endif
