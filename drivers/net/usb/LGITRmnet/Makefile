#
# Makefile for the kernel USB device drivers.
#

# Object files in subdirectories
obj-y := LGITRmnet.o
LGITRmnet-objs := qmi_ctl.o qmi_svc.o util.o rmnet.o
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all: clean build

build:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
