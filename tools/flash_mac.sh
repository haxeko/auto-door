#!/usr/bin/env bash
# Flash the auto-door firmware to an ESP32-C6 from macOS.
#
# Usage:
#   ./tools/flash_mac.sh [-p /dev/cu.xxx] [-d firmware_dir] [--erase] [--monitor]
#
#   -p, --port     serial port (default: auto-detect /dev/cu.usbmodem* or /dev/cu.usbserial*)
#   -d, --dir      folder with the .bin files (default: ./firmware)
#   --erase        erase the whole flash first (recommended the first time; wipes Matter pairing)
#   --monitor      open the serial console after flashing (exit: Ctrl+A then K, then y)
#
# Uses auto-door-esp32c6-*-merged.bin at 0x0 if present, otherwise the four separate files.
set -euo pipefail

PORT=""
DIR="firmware"
ERASE=0
MONITOR=0
BAUD=460800

while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|--port) PORT="$2"; shift 2 ;;
        -d|--dir) DIR="$2"; shift 2 ;;
        --erase) ERASE=1; shift ;;
        --monitor) MONITOR=1; shift ;;
        -h|--help) sed -n '2,11p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

# esptool: Homebrew/pip installs `esptool` (v5) or `esptool.py` (v4)
if command -v esptool >/dev/null 2>&1; then
    ESPTOOL=(esptool)
elif command -v esptool.py >/dev/null 2>&1; then
    ESPTOOL=(esptool.py)
elif python3 -c "import esptool" >/dev/null 2>&1; then
    ESPTOOL=(python3 -m esptool)
else
    echo "esptool not found. Install it with:  brew install esptool   or   python3 -m pip install --user esptool" >&2
    exit 1
fi

if [[ -z "$PORT" ]]; then
    PORTS=()
    for p in /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART*; do
        [[ -e "$p" ]] && PORTS+=("$p")
    done
    if [[ ${#PORTS[@]} -eq 0 ]]; then
        echo "No ESP32 serial port found. Plug the board in (data cable) and check: ls /dev/cu.*" >&2
        exit 1
    fi
    if [[ ${#PORTS[@]} -gt 1 ]]; then
        echo "Several ports found, choose one with -p:" >&2
        printf '  %s\n' "${PORTS[@]}" >&2
        exit 1
    fi
    PORT="${PORTS[0]}"
fi
echo "Port: $PORT"

MERGED=$(ls "$DIR"/auto-door-esp32c6-*-merged.bin 2>/dev/null | head -n1 || true)

if [[ $ERASE -eq 1 ]]; then
    "${ESPTOOL[@]}" --chip esp32c6 -p "$PORT" -b "$BAUD" erase_flash
fi

COMMON=(--chip esp32c6 -p "$PORT" -b "$BAUD" --before default_reset --after hard_reset write_flash
        --flash_mode dio --flash_freq 80m --flash_size 4MB)
if [[ -n "$MERGED" ]]; then
    echo "Flashing $MERGED at 0x0"
    "${ESPTOOL[@]}" "${COMMON[@]}" 0x0 "$MERGED"
else
    APP=$(ls "$DIR"/auto-door-esp32c6-*-app.bin 2>/dev/null | head -n1 || true)
    for f in "$DIR/bootloader.bin" "$DIR/partition-table.bin" "$DIR/ota_data_initial.bin" "$APP"; do
        [[ -n "$f" && -f "$f" ]] || { echo "Missing file in $DIR: ${f:-auto-door-esp32c6-*-app.bin}" >&2; exit 1; }
    done
    "${ESPTOOL[@]}" "${COMMON[@]}" \
        0x0 "$DIR/bootloader.bin" \
        0xc000 "$DIR/partition-table.bin" \
        0x1d000 "$DIR/ota_data_initial.bin" \
        0x20000 "$APP"
fi

echo "Done."
if [[ $MONITOR -eq 1 ]]; then
    echo "Opening console at 115200 baud (exit: Ctrl+A, K, y)"
    exec screen "$PORT" 115200
fi
