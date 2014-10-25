# This is the Part of original library 

MYLIB_INC = $(MYLIB)
MYLIB_SRC = $(MYLIB)

LIBINCDIRS += $(MYLIB_INC)

CFILES += \
 $(MYLIB_SRC)/timer.c 				\
 $(MYLIB_SRC)/scheduler.c 			\
 $(MYLIB_SRC)/PID.c 				\
 $(MYLIB_SRC)/rc_io.c				\
 $(MYLIB_SRC)/i2c.c					\
 $(MYLIB_SRC)/fram.c				\
 $(MYLIB_SRC)/AHRS.c				\
 $(MYLIB_SRC)/PX4FLOW.c				\
 $(MYLIB_SRC)/LSM303DLH.c			\
 $(MYLIB_SRC)/WMP.c


