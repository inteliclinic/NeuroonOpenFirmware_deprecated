SHELL := /bin/bash

NO_COLOR=\x1b[0m
OK_COLOR=\x1b[32;01m
ERROR_COLOR=\x1b[31;01m
WARN_COLOR=\x1b[33;01m
BLUE_COLOR=\x1b[34;01m

PROJECT=neuroon2
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
CCP = arm-none-eabi-g++
RM=rm -rf
LINK_SCRIPT=nrf51822_sdk/linker/gcc_nrf51_s110_xxaa_app.ld

OPTIMIZATION_DBG=g
DEBUGLVL_DBG=3
OPTIMIZATION_RLS=2
DEBUGLVL_RLS=0

INCLUDES=-Iinc\
	 -Iinc/states\
	 -Iinc/states/state_testtool\
	 -Iinc/dev_handlers\
	 -Iinc/ltc_drivers\
	 -Iinc/rc_API\
	 -Inrf51822_lib/inc\
	 -Inrf51822_sdk/inc\
	 -Inrf51822_sdk/inc/s110\
	 -Ineuroon_unified_communication/API\
	 -Ineuroon_unified_communication/src

GIT_VERSION = $(shell git describe --tags --abbrev=5 --dirty=-D)

BIN_DIR = Bin
OBJ_DIR_R = Release.obj
OBJ_DIR_D = Debug.obj

LINKER_FLAGS = -mcpu=cortex-m0 -mthumb -fmessage-length=0 -fsigned-char\
	       -Wl,--gc-sections -T$(LINK_SCRIPT)\
	       -lm -lc -lnosys -lrdimon -lrdpmon -lg

SECONDARY_FLASH += \
$(PROJECT).bin \
$(PROJECT)_DBG.bin

SECONDARY_SIZE += \
$(PROJECT).siz \
$(PROJECT)_DBG.siz

SRCS += \
system_nrf51.c \
$(wildcard nrf51822_lib/src/*.c) \
$(wildcard nrf51822_sdk/src/*.c) \
$(wildcard src/*.c) \
$(wildcard src/rc_API/*.c) \
$(wildcard src/dev_handlers/*.c) \
$(wildcard src/ltc_drivers/*.c)\
$(wildcard src/states/*.c)\
$(wildcard src/states/state_testtool/*.c)\
$(wildcard neuroon_unified_communication/src/*.c) \

ASMSRC = gcc_startup_nrf51.S


override CFLAGS += -c -mcpu=cortex-m0 -mthumb -fmessage-length=0\
		 -fsigned-char -ffunction-sections -fdata-sections -DBOARD_PCA10001\
		 -DBLE_STACK_SUPPORT_REQD -DDEBUG_LOGS_BLE -DSAMPLING_RATE=125 -DNRF51\
		 $(INCLUDES) -std=gnu11 -Wall -Wextra -Wno-unused-parameter\
		 -DNEURON_FIRM_VERSION=\"$(GIT_VERSION)\"\
		 --specs=nano.specs --specs=nosys.specs

ASSEMBLER_FLAGS = -c -mcpu=cortex-m0 -mthumb -fmessage-length=0\
		  -fsigned-char -ffunction-sections -fdata-sections \
		  -x assembler-with-cpp -DBOARD_PCA10001 -DNRF51 -DBLE_STACK_SUPPORT_REQD\
		  -DDEBUG_LOGS_BLE -DSAMPLING_RATE=125 -Wall -pedantic -Wextra\
		  --specs=nano.specs --specs=nosys.specs

CFLAGS_DBG = $(CFLAGS) -O$(OPTIMIZATION_DBG) -g$(DEBUGLVL_DBG) -DNEUROON_CLI_DBG
CFLAGS_RLS = $(CFLAGS) -O$(OPTIMIZATION_RLS) -g$(DEBUGLVL_RLS)

AFLAGS_DBG = $(ASSEMBLER_FLAGS) -O$(OPTIMIZATION_DBG) -g$(DEBUGLVL_DBG) -DNEUROON_CLI_DBG
AFLAGS_RLS = $(ASSEMBLER_FLAGS) -O$(OPTIMIZATION_RLS) -g$(DEBUGLVL_RLS)

OBJS_R := $(SRCS:%.c=$(OBJ_DIR_R)/%.o) $(ASMSRC:%.S=$(OBJ_DIR_R)/%.o)
DEPS_R := $(SRCS:%.c=$(OBJ_DIR_R)/%.d) $(ASMSRC:%.S=$(OBJ_DIR_R)/%.d)
OBJS_D := $(SRCS:%.c=$(OBJ_DIR_D)/%.o) $(ASMSRC:%.S=$(OBJ_DIR_D)/%.o)
DEPS_D := $(SRCS:%.c=$(OBJ_DIR_D)/%.d) $(ASMSRC:%.S=$(OBJ_DIR_D)/%.d)

OPENOCD = openocd -f interface/stlink-v2-1.cfg -f target/nrf51.cfg -f openocd.cnf

all: secondary-outputs

release: $(PROJECT).bin $(PROJECT).siz

debug: $(PROJECT)_DBG.bin $(PROJECT)_DBG.siz
	mv Bin/$(PROJECT)_DBG.bin Bin/$(PROJECT).bin

$(PROJECT).elf: $(OBJS_R)
	@echo -e "$(BLUE_COLOR)[LD]$(NO_COLOR) $@"
	@mkdir -p $(BIN_DIR)
	@$(CC) $(OBJS_R) $(LINKER_FLAGS) -Wl,-Map,"$(BIN_DIR)/$(PROJECT).map" -o "$(BIN_DIR)/$@"
	@echo -e "Neuroon in $(GIT_VERSION) was build"

$(PROJECT).bin: $(PROJECT).elf
	@echo -e "$(BLUE_COLOR)[LD]$(NO_COLOR) $@"
	@$(OBJCOPY) -O binary "$(BIN_DIR)/$<" "$(BIN_DIR)/$@"

$(PROJECT).siz: $(PROJECT).elf
	@echo -e "$(OK_COLOR)[INFO]$(NO_COLOR) SIZE"
	@$(SIZE) --format=berkeley "$(BIN_DIR)/$<"

$(PROJECT)_DBG.elf: $(OBJS_D)
	@echo -e "$(BLUE_COLOR)[LD]$(NO_COLOR) $@"
	@mkdir -p $(BIN_DIR)
	@$(CC) $(OBJS_D) $(LINKER_FLAGS) -Wl,-Map,"$(BIN_DIR)/$(PROJECT)_DBG.map" -o "$(BIN_DIR)/$@"
	@echo -e "Neuroon in $(GIT_VERSION) was build"

$(PROJECT)_DBG.bin: $(PROJECT)_DBG.elf
	@echo -e "$(BLUE_COLOR)[LD]$(NO_COLOR) $@"
	@$(OBJCOPY) -O binary "$(BIN_DIR)/$<" "$(BIN_DIR)/$@"

$(PROJECT)_DBG.siz: $(PROJECT)_DBG.elf
	@echo -e "$(OK_COLOR)[INFO]$(NO_COLOR) SIZE"
	@$(SIZE) --format=berkeley "$(BIN_DIR)/$<"

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

clean:
	@$(RM) $(OBJS_R) $(OBJS_D) $(DEPS_R) $(BIN_DIR)/$(PROJECT).bin $(BIN_DIR)/$(PROJECT).elf\
	  $(BIN_DIR)/$(PROJECT).map $(BIN_DIR)/$(PROJECT)_DBG.bin $(BIN_DIR)/$(PROJECT)_DBG.elf\
	  $(BIN_DIR)/$(PROJECT)_DBG.map
	@echo "Cleaned!"

.PHONY: all clean dependents debug release
.SECONDARY:

include makefile.openocd

-include $(OBJS_R:.o=.d)
-include $(OBJS_D:.o=.d)

$(OBJ_DIR_R)/%.o: %.c
	@echo -e "$(OK_COLOR)[CC]$(NO_COLOR) $@"
	@mkdir -p $(dir $@)
	@$(CC) $(CFLAGS_RLS) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"

$(OBJ_DIR_R)/%.o: %.S
	@echo -e "$(WARN_COLOR)[ASM]$(NO_COLOR) $@"
	@mkdir -p $(dir $@)
	@$(CC) $(AFLAGS_RLS) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"

$(OBJ_DIR_D)/%.o: %.c
	@echo -e "$(OK_COLOR)[CC]$(NO_COLOR) $@"
	@mkdir -p $(dir $@)
	@$(CC) $(CFLAGS_DBG) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"

$(OBJ_DIR_D)/%.o: %.S
	@echo -e "$(WARN_COLOR)[ASM]$(NO_COLOR) $@"
	@mkdir -p $(dir $@)
	@$(CC) $(AFLAGS_DBG) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
