# Ultrasonic quick test

Standalone HC-SR04 and scanner-servo diagnostic. Motors remain disabled. The
servo visits 30, 60, 90, 120 and 150 degrees; each reported distance is the
median of five pings.

Wiring: TRIG GPIO6, ECHO GPIO7 through a 5V-to-3.3V divider, servo signal
GPIO13. Open `/dev/tty.usbserial-*` at 115200 baud.
