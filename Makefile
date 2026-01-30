# Include the configuration file
include config.mk

PROJECT_NAME := $(notdir $(CURDIR))

# Default target
all: compile

compile:
	@if [ ! -f $(PROJECT_NAME).ino ]; then \
		echo "// Auto-generated dummy file to satisfy arduino-cli" > $(PROJECT_NAME).ino; \
	fi

	mkdir -p ./.cache/arduino
	arduino-cli compile -b $(BOARD_FQBN) --build-path $(BUILD_DIR) -v
	cp $(BUILD_DIR)/compile_commands.json .
	sed -i compile_commands.json -e 's/\.ino\.cpp/.ino/g' -e 's|$(BUILD_DIR)/sketch/||g'

upload: compile
	arduino-cli upload -p $(BOARD_PORT) -b "$(BOARD_FQBN)" --input-dir $(BUILD_DIR) .

monitor:
	arduino-cli monitor -p $(BOARD_PORT)

# TODO: Add a target that generates debug commands for the current configuration.

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR) ./.cache/arduino

.PHONY: all compile upload clean
