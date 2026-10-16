#!/usr/bin/env bash
# Compile this standalone C firmware without uploading or touching EEPROM.
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir="${BUILD_DIR:-build/mega}"
mkdir -p "$build_dir"
for source in ./*.c; do
    avr-gcc -mmcu=atmega2560 -DF_CPU=16000000UL -std=gnu99 \
        -Os -Wall -ffunction-sections -fdata-sections \
        -c "$source" -o "$build_dir/$(basename "${source%.c}").o"
done
avr-gcc -mmcu=atmega2560 -Wl,--gc-sections "$build_dir"/*.o -lm \
    -o "$build_dir/grbl.elf"
avr-objcopy -O ihex -R .eeprom "$build_dir/grbl.elf" "$build_dir/grbl.hex"
avr-size --mcu=atmega2560 --format=avr "$build_dir/grbl.elf"
