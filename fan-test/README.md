# Fan 25% test

This standalone firmware drives only the fan controller PWM input. All robot
motors and other peripherals remain uninitialised.

- PWM pin: GPIO5
- Frequency: 25 kHz
- Resolution: 10 bit
- Command: constant 25% (`256/1023`)
- Serial monitor: 115200 baud

Build and upload from this directory:

```sh
pio run
pio run -t upload
pio device monitor -b 115200
```
