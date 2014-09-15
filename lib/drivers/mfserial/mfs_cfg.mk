# This is the Part of MFS Driver 

MFS_INC = $(MFS_LIB)
MFS_SRC = $(MFS_LIB)

MFS_I2C = $(MFS_LIB)/i2c
MFS_SIO = $(MFS_LIB)/sio
MFS_SPI = $(MFS_LIB)/spi
MFS_UART = $(MFS_LIB)/uart

LIBINCDIRS += $(MFS_INC)			\
			  $(MFS_I2C)			\
			  $(MFS_SIO)			\
			  $(MFS_SPI)			\
			  $(MFS_UART)

CFILES += \
 $(MFS_SRC)/MfsDev_FM3.c 			\
 $(MFS_I2C)/I2cDev_FM3.c 			\
 $(MFS_SIO)/SioDev_FM3.c 			\
 $(MFS_SPI)/SpiDev_FM3.c 			\
 $(MFS_UART)/UartDev_FM3.c


