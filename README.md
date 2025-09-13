# ClosestPlane ESP32 (T-Deck Edition)

An Arduino-compatible sketch for the LilyGO T-Deck that connects to a dump1090 server and renders a sweeping radar view on the built-in 320×240 TFT display. Aircraft within the selected range appear as blips and a short tone plays as the sweep crosses each target. Volume, radar range and sweep speed are controlled with the T-Deck keyboard and touchpad and saved to EEPROM.

## Required Libraries

Install the following libraries in the Arduino IDE before compiling:

- **ESP32 board support** (`esp32` by Espressif)
- **TFT_eSPI**
- **TouchDrvGT911**
- **ArduinoJson**

## Hardware Connections

The T-Deck integrates display, keyboard and touchpad, so only external audio hardware is required:

- **MAX98357A I2S amplifier**: BCLK → GPIO17, LRCLK → GPIO16, DIN → GPIO27, SD → GPIO19, 3.3 V and GND.

## Setup
1. Rename `config.h` with your WiFi credentials, dump1090 server address and your latitude/longitude. Adjust I2S pin numbers if required.
2. Ensure the libraries above are installed in the Arduino IDE.
3. Open `closestPlane.ino` in the Arduino IDE, select your ESP32-S3 board (e.g. **esp32s3**) and the correct port.
4. Compile and upload.

## Operation
- The display shows a radar sweep of aircraft within range. Each time the sweep crosses a target a short beep is produced.
- Use **W/S** keys or vertical touchpad swipes to adjust beep volume, sweep speed, alert distance, or rotate the radar when compass mode is selected.
- Press **M** to cycle between control modes.
- Use **A/D** keys or horizontal swipes to change radar range.
- Press **P** to power off.
- Settings persist in EEPROM and a small antenna icon indicates a good data connection.
- Aircraft predicted to pass within the alert radius flash on the radar and trigger a single alert tone. The display shows minutes until the closest inbound aircraft reaches minimum distance.
- Each aircraft is tracked individually in an `alertedFlights` list so it will only trigger the siren once while inbound.
- The alarm sounds again only if that aircraft leaves the inbound zone (bearing difference over 90° or cross-track distance beyond the alert radius) and later re-enters, or if a different aircraft meets the inbound criteria.

## Building in a Codex/Codespace Environment

The following setup script prepares a Codex/Codespace container with all required tools and
libraries for this project. Run it in your container before compiling:
```bash
#!/bin/bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

echo "==> Base packages"
apt-get update
apt-get install -y --no-install-recommends \
  curl ca-certificates git python3 tar xz-utils

# --- Proxy handling ---
# Mirror the proxy you see in Codex logs (http://proxy:8080) for all tools.
HTTP_PROXY="${HTTP_PROXY:-${http_proxy:-}}"
HTTPS_PROXY="${HTTPS_PROXY:-${https_proxy:-$HTTP_PROXY}}"
NO_PROXY="${NO_PROXY:-${no_proxy:-localhost,127.0.0.1,::1}}"
export HTTP_PROXY HTTPS_PROXY NO_PROXY
# Some tools only read lowercase:
export http_proxy="${HTTP_PROXY:-}"
export https_proxy="${HTTPS_PROXY:-}"
export no_proxy="${NO_PROXY:-}"

echo "HTTP_PROXY=${HTTP_PROXY:-<unset>}"
echo "HTTPS_PROXY=${HTTPS_PROXY:-<unset>}"
echo "NO_PROXY=${NO_PROXY}"

# --- Prefer IPv4 to avoid IPv6 'network is unreachable' ---
if ! grep -q '^precedence ::ffff:0:0/96 100' /etc/gai.conf 2>/dev/null; then
  echo 'precedence ::ffff:0:0/96 100' >> /etc/gai.conf
fi

# --- Helper: retry with backoff ---
retry () {
  local attempts="$1"; shift
  local sleep_s="$1"; shift
  local n=1
  until "$@"; do
    if [[ $n -ge $attempts ]]; then
      echo "Command failed after $n attempts: $*" >&2
      return 1
    fi
    echo "Retry $n/$attempts failed. Sleeping ${sleep_s}s…"
    n=$((n+1))
    sleep "$sleep_s"
  done
}

echo "==> Installing arduino-cli"
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
install -m 0755 bin/arduino-cli /usr/local/bin/arduino-cli
rm -rf bin
arduino-cli version

echo "==> arduino-cli config"
arduino-cli config init --overwrite

# Boards index + proxy (this is the only network key we need)
arduino-cli config set board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

if [[ -n "${HTTPS_PROXY}" || -n "${HTTP_PROXY}" ]]; then
  arduino-cli config set network.proxy "${HTTPS_PROXY:-$HTTP_PROXY}"
fi

# Optional: slightly longer socket timeout (older CLI versions may ignore this; safe if it errors)
arduino-cli config set network.socket_timeout 60s || true

echo "==> Updating indexes (via proxy, with retries)"
retry 5 5 arduino-cli core update-index

echo "==> Installing ESP32 core"
retry 5 5 arduino-cli core install esp32:esp32

echo "==> Installing libraries"
retry 5 5 arduino-cli lib install "TFT_eSPI"
retry 5 5 arduino-cli lib install "TouchDrvGT911"
retry 5 5 arduino-cli lib install "ArduinoJson"

echo "✅ Setup complete. Compile with:"
echo "   arduino-cli compile --fqbn esp32:esp32:esp32s3 closestPlane.ino"
```

The command completes successfully when the required libraries (TFT_eSPI, TouchDrvGT911, etc.) are installed.
