# arch/arm/mach-s5p6440/Makefile
#
# Copyright 2008 Openmoko, Inc.
# Copyright 2008 Simtec Electronics
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

# Core support for S5P6440 system

obj-$(CONFIG_CPU_S5P6440)	+= cpu.o
obj-$(CONFIG_CPU_S5P6440)	+= dma.o

# Helper and device support

obj-$(CONFIG_S5P6440_SETUP_SDHCI)	+= setup-sdhci.o

obj-$(CONFIG_PM)    += pm.o

# machine support

obj-$(CONFIG_MACH_SMDK6440)	+= mach-smdk6440.o
