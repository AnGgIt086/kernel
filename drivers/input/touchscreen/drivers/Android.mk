LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

ifeq ($(BBK_PRODUCT_SOLUTION),QCOM)
ifneq ($(strip $(TARGET_KERNEL_VERSION)),)
ifeq ($(TARGET_KERNEL_VERSION),$(filter $(TARGET_KERNEL_VERSION),5.10 5.15))
# This makefile is only for DLKM
ifneq ($(findstring vendor,$(LOCAL_PATH)),)
DLKM_DIR   := $(TOP)/device/qcom/common/dlkm

LOCAL_ADDITIONAL_DEPENDENCIES := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
KBUILD_OPTIONS := MODNAME=vivo_ts
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)
VIVO_TP_EXTRA_SYMBOLS := $(shell pwd)/$(call intermediates-dir-for,DLKM,msm_drm-module-symvers)/Module.symvers
VIVO_TP_EXTRA_SYMBOLS += $(shell pwd)/$(call intermediates-dir-for,DLKM,vivo_mb-module-symvers)/Module.symvers
KBUILD_EXTRA_SYMBOLS := $(VIVO_TP_EXTRA_SYMBOLS)
KBUILD_OPTIONS += KBUILD_EXTRA_SYMBOLS=$(VIVO_TP_EXTRA_SYMBOLS)

include $(CLEAR_VARS)
# For incremental compilation
LOCAL_SRC_FILES           := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE              := vivo_ts-module-symvers
LOCAL_MODULE_STEM         := Module.symvers
LOCAL_MODULE_KBUILD_NAME  := Module.symvers
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

include $(CLEAR_VARS)
LOCAL_SRC_FILES   := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)
LOCAL_MODULE      := vivo_ts.ko
LOCAL_MODULE_KBUILD_NAME := vivo_ts.ko
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_REQUIRED_MODULES := msm_drm-module-symvers
LOCAL_REQUIRED_MODULES += vivo_mb-module-symvers
LOCAL_ADDITIONAL_DEPENDENCIES := $(call intermediates-dir-for,DLKM,msm_drm-module-symvers)/Module.symvers
LOCAL_ADDITIONAL_DEPENDENCIES += $(call intermediates-dir-for,DLKM,vivo_mb-module-symvers)/Module.symvers

include $(DLKM_DIR)/Build_external_kernelmodule.mk
include $(call all-makefiles-under,$(LOCAL_PATH))
endif

endif
endif
endif
