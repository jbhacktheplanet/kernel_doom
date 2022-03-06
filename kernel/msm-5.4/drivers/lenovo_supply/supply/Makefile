# SPDX-License-Identifier: GPL-2.0
subdir-ccflags-$(CONFIG_LENOVO_SUPPLY_DEBUG) := -DDEBUG

lenovo_supply-y				:= lenovo_supply_core.o
lenovo_supply-$(CONFIG_SYSFS)		+= lenovo_supply_sysfs.o

obj-$(CONFIG_LENOVO_SUPPLY)		+= lenovo_supply.o
obj-$(CONFIG_LENOVO_SUPPLY) 		+= ti/
