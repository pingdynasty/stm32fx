TEMPLATEROOT = ../..

CFLAGS  = -g
ASFLAGS = -g
# LDFLAGS = -lgcc -lc

# object files
OBJS = $(STARTUP) $(PERIPH) $(DISCOVERY) main.o loop.o 
# OBJS += $(DSPLIB)/SupportFunctions/arm_float_to_q31.o
# OBJS += $(DSPLIB)/FilteringFunctions/arm_fir_init_q31.o
# OBJS += $(DSPLIB)/FilteringFunctions/arm_fir_q31.o
# OBJS += $(DSPLIB)/SupportFunctions/arm_copy_q31.o
OBJS += $(DSPLIB)/SupportFunctions/arm_float_to_q15.o
OBJS += $(DSPLIB)/FilteringFunctions/arm_fir_init_q15.o
OBJS += $(DSPLIB)/FilteringFunctions/arm_fir_q15.o
OBJS += $(DSPLIB)/FilteringFunctions/arm_fir_fast_q15.o
OBJS += $(DSPLIB)/SupportFunctions/arm_copy_q15.o

# include common make file
include $(TEMPLATEROOT)/Makefile.f3
