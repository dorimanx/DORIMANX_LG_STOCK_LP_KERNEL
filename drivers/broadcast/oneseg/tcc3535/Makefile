#
# Makefile for the ISDB-Tmm TCC3530 drivers.
#

# Each configuration option enables a list of files.
# LGE

CFLAGS_KERNEL+=-D_MODEL_TCC3535_
CFLAGS_KERNEL+=-D_I2C_STS_

obj-$(CONFIG_LGE_BROADCAST_ISDBT_JAPAN)		+= broadcast_tcc3530_drv_if.o
obj-$(CONFIG_LGE_BROADCAST_ISDBT_JAPAN)		+= broadcast_tcc353x.o
obj-$(CONFIG_LGE_BROADCAST_ISDBT_JAPAN)		+= tcc353x_hal.o
obj-$(CONFIG_LGE_BROADCAST_ISDBT_JAPAN)		+= Tcc353xDriver/
obj-$(CONFIG_LGE_BROADCAST_ISDBT_JAPAN)		+= broadcast_mmbi_chk_dev.o
EXTRA_CFLAGS += -Idrivers/broadcast/oneseg
EXTRA_CFLAGS += -Idrivers/broadcast/oneseg/tcc3535/Tcc353xDriver/common
EXTRA_CFLAGS += -Idrivers/broadcast/oneseg/tcc3535/Tcc353xDriver/api/inc
EXTRA_CFLAGS += -Idrivers/broadcast/oneseg/tcc3535/Tcc353xDriver/PAL
EXTRA_CFLAGS += -Idrivers/broadcast/oneseg/tcc3535/Tcc353xDriver/sample/monitoring
EXTRA_CFLAGS += -Idrivers/broadcast/oneseg/tcc3535/Tcc353xDriver/sample/main

