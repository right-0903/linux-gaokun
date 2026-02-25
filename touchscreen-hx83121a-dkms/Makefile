# SPDX-License-Identifier: GPL-2.0
MODNAME = himax-spi
MODVER = 0.1

obj-m += $(MODNAME).o

KVER = $(shell uname -r)
KDIR = /lib/modules/$(KVER)/build
MDIR = /usr/src/$(MODNAME)-$(MODVER)

all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean

dkmsinstall:
	cp -R . /usr/src/$(MODNAME)-$(MODVER)
	dkms install -m $(MODNAME) -v $(MODVER)

dkmsremove:
	dkms remove -m $(MODNAME) -v $(MODVER) --all
	rm -rf /usr/src/$(MODNAME)-$(MODVER)
