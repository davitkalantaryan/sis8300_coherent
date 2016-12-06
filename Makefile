#
# Makefile for creating 'sis8300_coherent' driver
# 'sis8300_coherent' is LINUX PCIe driver for MTCA devices
# Any problem concerning to functionalities of the driver
# Please contact
#		D. Kalantaryan (davit.kalantaryan@desy.de)
#
	
MODULE_NAME = sis8300_coherent

KVERSION = $(shell uname -r)
#HOSTNAME2 = $(shell hostname)
#KO_FILES = ../../../ko_files/$(HOSTNAME2)_$(KVERSION)
KO_FILES = ko_files
CUR_DIR=$(PWD)
MODULE_DIR = /lib/modules/$(KVERSION)/desy_mtca


$(MODULE_NAME)-objs := \
	sis8300_main.o

#obj-m := pcie_gen_drv.o
obj-m := $(MODULE_NAME).o 


default: compile

all: compile copy install insert

copy:
	mkdir -p $(MODULE_DIR)
	sudo cp $(KO_FILES)/$(MODULE_NAME).ko $(MODULE_DIR)/.

install:
	#-sudo rmmod pcie_gen_drv
	#sudo insmod pcie_gen_drv.ko
	#sudo cp pcie_gen_drv.ko /lib/modules/$(KVERSION)/desy_zeuthen/.

insert:
	-sudo  $(MODULE_NAME)
	sudo insmod $(KO_FILES)/$(MODULE_NAME).ko

compinsert: compile insert

compile:
	cp mtcagen/Module.symvers .
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
	mkdir -p $(KO_FILES)
	mv $(MODULE_NAME).ko $(KO_FILES)/.
	#cp upciedev/upciedev.ko $(KO_FILES)/.
clean:
	test ! -d /lib/modules/$(KVERSION) || make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
	
	
#EXTRA_CFLAGS	+= -I/usr/include
#EXTRA_CFLAGS	+= -I/doocs/develop/include
#EXTRA_CFLAGS	+= -I/doocs/develop/common/include
EXTRA_CFLAGS	+= -DUSE_SEMAPHORE
