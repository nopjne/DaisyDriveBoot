all: OS64daisyboot.z64
.PHONY: all

BUILD_DIR = build
include $(N64_INST)/include/n64.mk

SRC = daisyboot.c
OBJS = $(SRC:%.c=$(BUILD_DIR)/%.o)
DEPS = $(SRC:%.c=$(BUILD_DIR)/%.d)

OS64daisyboot.z64: N64_ROM_TITLE = "DaisyDrive64 Bootldr"

$(BUILD_DIR)/OS64daisyboot.elf: $(OBJS)

clean:
	rm -rf $(BUILD_DIR) *.z64
.PHONY: clean

-include $(DEPS)
