# Automatic Mobile Air Purifier — ESP32-S3 N16R8

Firmware for an ESP32-S3-WROOM-1 N16R8 core board compatible with the
ESP32-S3 DevKitC-1 breakout layout. It uses the module's 16 MB flash and 8 MB
PSRAM configuration, runs at 240 MHz, and drives the mobile air purifier's
L298N motors, x4 quadrature encoders, PMS3003, scanning HC-SR04, SG90 servo,
PWM fan, and 20x4 I2C LCD.

The project defines its own N16R8 PlatformIO board in
`boards/esp32_s3_devkitc_1_n16r8.json`; it is not configured as a 4D Systems
display module.

## Safety first

Every ESP32-S3 GPIO is 3.3 V-only. Never connect a 5 V signal directly:

- place a level converter or resistor divider on HC-SR04 `Echo`;
- ensure PMS3003 TX and each encoder output are 3.3 V; and
- put an I2C level converter between the ESP32-S3 and a typical 5 V LCD
  backpack. Its pull-up resistors otherwise raise SDA/SCL to 5 V.

The standard 0–25 V Arduino voltage-sensor module uses a 30 kOhm/7.5 kOhm
(1:5) divider: connect its `+` terminal to battery positive, `-` to battery
negative/common ground, and `S` to GPIO1. With this 3.3 V ESP32-S3 system,
keep the measured input below 16.5 V; a 12 V SLA charged to 14.4 V is safe.

## Build

```sh
pio run
pio run -t upload
pio device monitor -b 115200
pio test -e native
```

See [ESP32-S3 DevKitC N16R8 wiring](docs/การต่อสาย-ESP32-S3.md) before connecting power. Set `MOTION_ENABLED=false` in
`include/Config.h` until motor direction and encoder polarity are confirmed.

At startup the fan is commanded to 100% immediately for five seconds so its
built-in soft-start can complete with a full-throttle launch burst. It then
returns to LOW while PMS3003 completes its 30-second warm-up; after that,
valid PM2.5 readings select LOW, MEDIUM, or HIGH automatically. The chassis
waits for both the launch burst and sensor warm-up before it can drive.
