obj-m := nxp-csi2rx-driver.o

ccflags-y += -DDEBUG
ccflags-y += -DDEBUG_APPS
#ccflags-y += -DUNUSED_ATTR
#ccflags-y += -DTIME_STAMP_EN
SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) 

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

