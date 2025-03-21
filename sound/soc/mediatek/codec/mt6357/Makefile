// SPDX-License-Identifier: GPL-2.0
ifneq ($(CONFIG_MACH_MT6739),y)
subdir-ccflags-y += -DAUDIO_USING_WRAP
endif
# PMIC object
ifeq ($(CONFIG_SND_SOC_AW87XXX), y)
obj-y += mtk-soc-codec-6357-pd2140.o mtk-auddrv-ana.o
else
obj-y += mtk-soc-codec-6357.o mtk-auddrv-ana.o
endif
