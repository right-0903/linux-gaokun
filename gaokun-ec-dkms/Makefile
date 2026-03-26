# SPDX-License-Identifier: GPL-2.0-only
MODNAME = huawei-gaokun-ec
MODVER = 1.0

obj-m += $(MODNAME).o

obj-m += huawei-gaokun-battery.o \
	 ucsi_huawei_gaokun.o


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

dkms-headers:
	sed -i 's#<linux/platform_data/huawei-gaokun-ec\.h>#"huawei-gaokun-ec\.h"#' ./*huawei*.c

kernel-headers:
	sed -i 's#"huawei-gaokun-ec\.h"#<linux/platform_data/huawei-gaokun-ec\.h>#' ./*huawei*.c
