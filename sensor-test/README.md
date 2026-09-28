# Sensor test

Standalone diagnostic firmware for the robot's sensors. Motors and fan are not
driven. It reports once per second through UART0 (`/dev/cu.usbserial-*`) at
115200 baud.

It checks:

- PMS3003 frames and PM1/PM2.5/PM10 values (wait 30 seconds for warm-up);
- HC-SR04 distance while its SG90 scanner sweeps 15°–165°;
- both Hall quadrature encoder counts (turn each driven wheel by hand);
- battery voltage through the 1:5 voltage-sensor module on GPIO1; and
- INA219 bus voltage, current, and power on the separate GPIO8/GPIO9 I2C1
  bus at address `0x40`.

## Run

1. Keep the L298N/motor power disabled; this project never drives motors.
2. Upload from this directory.
3. Open the USB-to-UART serial port at 115200 baud.
4. Turn each wheel by hand and place an object in front of the sonar.

```sh
pio run -t upload
pio device monitor -b 115200
```
