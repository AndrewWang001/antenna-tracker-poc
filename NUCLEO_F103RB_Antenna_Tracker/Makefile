# Command-line build for the NUCLEO-F103RB antenna tracker.
# STM32CubeIDE builds the same sources (Debug/ config); this Makefile is for
# CI and for building without the IDE.
#
#   make                      -> build/antenna_tracker.elf/.bin/.hex
#   make flash                -> program over ST-LINK (STM32_Programmer_CLI on PATH)
#   make PREFIX=/path/to/bin/arm-none-eabi-
PREFIX  ?= arm-none-eabi-
CC       = $(PREFIX)gcc
OBJCOPY  = $(PREFIX)objcopy
SIZE     = $(PREFIX)size
BUILD    = build
TARGET   = $(BUILD)/antenna_tracker

RTOS     = Middlewares/Third_Party/FreeRTOS/Source
HAL      = Drivers/STM32F1xx_HAL_Driver

SRCS  = $(wildcard Core/Src/*.c)
SRCS += $(wildcard $(HAL)/Src/*.c)
SRCS += $(wildcard $(RTOS)/*.c) $(RTOS)/CMSIS_RTOS_V2/cmsis_os2.c \
        $(RTOS)/portable/GCC/ARM_CM3/port.c $(RTOS)/portable/MemMang/heap_4.c
ASMS  = Core/Startup/startup_stm32f103rbtx.s

INCS  = -ICore/Inc -I$(HAL)/Inc -I$(HAL)/Inc/Legacy \
        -IDrivers/CMSIS/Device/ST/STM32F1xx/Include -IDrivers/CMSIS/Include \
        -I$(RTOS)/include -I$(RTOS)/CMSIS_RTOS_V2 -I$(RTOS)/portable/GCC/ARM_CM3

MCU     = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
CFLAGS  = $(MCU) -std=gnu11 -Os -g3 -DUSE_HAL_DRIVER -DSTM32F103xB $(INCS) \
          -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP
LDFLAGS = $(MCU) -TSTM32F103RBTX_FLASH.ld --specs=nano.specs -Wl,--gc-sections \
          -Wl,-Map=$(TARGET).map -Wl,--print-memory-usage -static \
          -Wl,--start-group -lc -lm -Wl,--end-group

OBJS = $(SRCS:%.c=$(BUILD)/%.o) $(ASMS:%.s=$(BUILD)/%.o)

all: $(TARGET).elf $(TARGET).bin $(TARGET).hex

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -x assembler-with-cpp -c $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

flash: $(TARGET).elf
	STM32_Programmer_CLI -c port=SWD -w $< -v -rst

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)

.PHONY: all flash clean
