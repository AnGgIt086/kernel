#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mmp/
ccflags-y += -I$(srctree)/drivers/staging/android/mtk_ion/
ccflags-y  += -I$(srctree)/drivers/staging/android/mtk_ion/mtk/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/smi/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mmdvfs/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/timer/timesync/
ccflags-y += -I$(srctree)/include/dt-bindings/memory/
ccflags-y += -I$(srctree)/drivers/iommu/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/mt6885/include/
ccflags-y += -I$(srctree)/drivers/clk/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/imgsensor/inc/
<<<<<<< HEAD

=======
#chenhan add for ois#
$(warning $(srctree))
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/lens/mtk/main/inc \
                    -I$(srctree)/drivers/misc/mediatek/lens/mtk/main/common/ois/inc
#add end#
>>>>>>> a3d615ec2 (PD2204F_EX/PD2204F_EX_SC:[B220518-3112]OIS_kernel-4.19移植)
obj-y += camera_isp.o
obj-y += cam_qos.o