# Ultrasound Motion Detector (mdetector)

A background service that polls an ultrasound sensor over Modbus-RTU, detects motion, and writes flag/telemetry files. Designed as a replacement for `motion_detector.py`.

## Features

- Polls the sensor at a configurable interval
- Maintains a moving average over a configurable window
- Detects motion based on distance threshold and jitter
- Writes motion/no-motion flag files
- Writes a live telemetry dump file
- Supports multiple instances on different serial ports

## Requirements

- Linux (tested on Raspberry Pi OS)
- `libmodbus`
- `cmake`, `pkg-config`, `g++`

Install dependencies:

```bash
sudo apt update
sudo apt install libmodbus-dev pkg-config cmake g++
```

## Build

```bash
mkdir build
cd build
cmake ..
make
```

The binary `mdetector` will be created in `build/`.

## Usage

### Manual test

```bash
./mdetector --port /dev/serial0 \
            --flag /tmp/motion.flg \
            --unflag /tmp/no-motion.flg \
            --dump /dev/shm/mdetector.txt
```

### Service options

| Option | Description | Default |
|---|---|---|
| `--port` | Serial device | `/dev/serial0` |
| `--baud` | Baud rate | `115200` |
| `--slave` | Modbus slave ID | `1` |
| `--interval` | Poll interval in ms | `100` |
| `--average` | Averaging window in seconds | `5` |
| `--jitter` | Distance deviation that counts as motion in mm | `50` |
| `--distance` | Distance threshold in mm | `350` |
| `--flag` | Motion flag file | `/tmp/motion.flg` |
| `--unflag` | No-motion flag file | `/tmp/no-motion.flg` |
| `--dump` | Telemetry dump file | `/dev/shm/mdetector.txt` |

## Install as a systemd service

Copy the binary and the service template:

```bash
sudo cp build/mdetector /usr/local/bin/
sudo cp systemd/mdetector@.service /etc/systemd/system/
sudo systemctl daemon-reload
```

Start an instance on a specific port:

```bash
sudo systemctl enable --now mdetector@serial0.service
```

Start another instance on a different port:

```bash
sudo systemctl enable --now mdetector@ttyUSB0.service
```

Check status:

```bash
sudo systemctl status mdetector@serial0.service
```

View logs:

```bash
sudo journalctl -u mdetector@serial0.service -f
```

## How it works

1. Reads the distance register `0x0101` every `--interval` ms.
2. Keeps a sliding window of recent readings sized for `--average` seconds.
3. Compares the current reading to the average.
4. Declares motion when the reading is both:
   - Closer than `--distance`
   - Different from the average by more than `--jitter`
5. Updates flag files and telemetry dump on every state change.

## Troubleshooting

### `Permission denied` on `/dev/serial0`

```bash
sudo usermod -a -G dialout $USER
```

### Port is busy

Stop any other service using the port:

```bash
sudo systemctl stop mdetector-service
```

### No reply from sensor

- Check wiring and power.
- Confirm baud rate and slave ID.
- Verify the sensor is not being used by another process.
