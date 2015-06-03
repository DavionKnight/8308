ifneq ($(KERNELRELEASE),)

obj-m := oam_fpga_drv.o

else
	
KDIR :=  ../../edd-linux-2.6.29.6
all:
	make -C $(KDIR) M=$(PWD) modules
	#cp oam_fpga_drv.ko ~/edd_update/drv_0204/
	#cp oam_fpga_drv.ko ~/edd_update/drv_0402/
clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers

endif
