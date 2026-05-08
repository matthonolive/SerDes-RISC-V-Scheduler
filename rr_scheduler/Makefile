MARCH = rv32imc_zicsr_zifencei
EFFORT = -Os
USER_FLAGS += -Wl,--defsym,__neorv32_rom_size=64k
USER_FLAGS += -Wl,--defsym,__neorv32_ram_size=32k

# Add your extra source files
APP_SRC =  main.c scheduler.c context.S timer.S

NEORV32_HOME ?= ../../..
include $(NEORV32_HOME)/sw/common/common.mk
