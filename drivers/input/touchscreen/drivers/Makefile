ifeq ($(CONFIG_VIVO_TS),)
  CONFIG_VIVO_TS=y
endif

obj-$(CONFIG_VIVO_TS) += vivo_ts.o

vivo_ts-y += vts_core.o
vivo_ts-y += vts_node.o
vivo_ts-y += vts_incell.o
vivo_ts-y += vts_class.o
vivo_ts-y += vts_report.o
vivo_ts-y += vts_op.o
vivo_ts-y += vts_spi_drv.o

vivo_ts-y += $(addprefix ili_9882n/, \
				ili9881x.o \
				ili9881x_hostdl.o \
				ili9881x_ic.o \
				ili9881x_node.o \
				ili9881x_plat.o \
				ili9881x_spi.o \
				ili9881x_touch.o \
				vivo_v2.o \
			)

vivo_ts-y += vts_init.o

ifeq ($(BBK_PRODUCT_SOLUTION),MTK)
	KBUILD_CFLAGS += -DMTK_FOR_VIVO_TS
else ifeq ($(BBK_PRODUCT_SOLUTION),QCOM)
	KBUILD_CFLAGS += -DQCOM_FOR_VIVO_TS
else ifeq ($(BBK_PRODUCT_SOLUTION),SAMSUNG)
	KBUILD_CFLAGS += -DSAMSUNG_FOR_VIVO_TS
endif

ifeq ($(TOUCH_LOG_COLLECT_CONFIG),yes)
	KBUILD_CFLAGS += -DTOUCH_LOG_COLLECT_SUPPORT
endif

ifeq ($(BBK_PRODUCT_PLATFORM),SM8250)
	KBUILD_CFLAGS += -DTOUCH_NETLINK_COM_ALTERNATE
endif

ifeq ($(VTS_DOUBLE_SCREEN_PROJECT_DEBUG_COMPILE), yes)
	KBUILD_CFLAGS += -DVTS_SECOND_LCMID
endif

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)
modules_install:
	$(MAKE) M=$(M) -C $(KERNEL_SRC) modules_install
