# L298N motor test

Standalone ESP32-S3 test firmware for the L298N and the two drive motors. It
does not require the Hall encoders, sonar, PMS3003, or any other robot module.

## Safety

1. Remove the L298N `ENA` and `ENB` jumpers.
2. Lift both wheels clear of the floor.
3. Make sure ESP32 GND and L298N GND are connected together.
4. Upload, open the 115200-baud serial monitor, then use the control keys.

The automatic sequence is: forward, stop, in-place left turn, stop, in-place
right turn, stop, reverse, stop. Manual movement continues until stopped.

If a wheel moves in the wrong direction, stop power first, then swap that
motor's two wires at the relevant L298N `OUT` terminal. Do not change encoder
wiring during this motor-only test.

The status console is UART0, normally shown as `/dev/cu.usbserial-*` on macOS,
at 115200 baud. It is deliberately not the native-USB `usbmodem` port.

## Commands

- `w`: drive forward continuously
- `s`: reverse continuously
- `a`: turn left continuously in place
- `d`: turn right continuously in place
- `x`: stop immediately
- `t`: run the timed automatic sequence

Adjust `TEST_PWM`, `MOVE_MS`, and `TURN_MS` near the top of `src/main.cpp`
only after confirming the initial low-power test is safe.
