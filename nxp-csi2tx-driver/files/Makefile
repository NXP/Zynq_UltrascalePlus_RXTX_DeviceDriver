obj-m := nxp-csi2tx-driver.o

#ccflags-y += -DDEBUG
#ccflags-y += -DTIME_STAMP_EN
#ccflags-y += -DMODULE_PARAM_EN
#ccflags-y += -DUNUSED_ATTR

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

