PROJECT := twitch_chassic
BUILD_PRESET ?= Debug
BUILD_DIR := build/$(BUILD_PRESET)

ELF := $(BUILD_DIR)/$(PROJECT).elf
HEX := $(BUILD_DIR)/$(PROJECT).hex
BIN := $(BUILD_DIR)/$(PROJECT).bin
ASM := $(BUILD_DIR)/$(PROJECT).asm

PYOCD ?= python3 -m pyocd
PYOCD_TARGET ?= stm32f405rgtx
PYOCD_FREQ ?= 1MHz
PYOCD_PACK ?= $(HOME)/.local/share/pyocd/packs/Keil.STM32F4xx_DFP.3.1.1.pack

.PHONY: all configure build clean size hex bin asm artifacts flash erase reset release flash-release

all: artifacts

configure:
	cmake --preset $(BUILD_PRESET)

build: configure
	cmake --build --preset $(BUILD_PRESET)

size: build
	arm-none-eabi-size $(ELF)

hex: build
	arm-none-eabi-objcopy -O ihex $(ELF) $(HEX)

bin: build
	arm-none-eabi-objcopy -O binary -S $(ELF) $(BIN)

asm: build
	arm-none-eabi-objdump $(ELF) -dSC > $(ASM)

artifacts: build hex bin size

flash: artifacts
	$(PYOCD) load --pack $(PYOCD_PACK) -t $(PYOCD_TARGET) -f $(PYOCD_FREQ) -e chip $(ELF)

erase:
	$(PYOCD) erase --pack $(PYOCD_PACK) -t $(PYOCD_TARGET) -f $(PYOCD_FREQ) --chip

reset:
	$(PYOCD) reset --pack $(PYOCD_PACK) -t $(PYOCD_TARGET) -f $(PYOCD_FREQ)

clean:
	rm -rf build

release:
	$(MAKE) artifacts BUILD_PRESET=Release

flash-release:
	$(MAKE) flash BUILD_PRESET=Release
