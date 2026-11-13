# Intelligent Autonomous Robo Car — Setup Guide

## Firmware

Open `Intelligent_Autonomous_Robo_Car.ino` in Arduino IDE and select **Arduino Nano / ATmega328P**.

## Libraries

Install through Arduino IDE Library Manager:
- Adafruit GFX Library
- Adafruit SSD1306

Provided by the Arduino AVR core:
- Wire
- Servo
- SoftwareSerial

## Fixed pin map

| Module | Signal | Nano pin |
|---|---|---:|
| L298N | ENA | D5 (PWM) |
| L298N | IN1 | D7 |
| L298N | IN2 | D8 |
| L298N | IN3 | D9 |
| L298N | IN4 | D10 |
| L298N | ENB | D6 (PWM) |
| HC-SR04 | TRIG | D11 |
| HC-SR04 | ECHO | D12 |
| SG90 | Signal | D3 |
| Left IR | Digital output | A0 |
| Right IR | Digital output | A1 |
| LDR | Analog output | A2 |
| HC-05 | Arduino SoftwareSerial RX | D2 (connect HC-05 TXD here) |
| HC-05 | Arduino SoftwareSerial TX | D4 (connect to HC-05 RXD through a level divider) |
| HC-05 | STATE, optional | D13 |
| SSD1306 | SDA | A4 |
| SSD1306 | SCL | A5 |

## Bluetooth protocol

Send one character at a time at **9600 baud**:

| Command | Action |
|---|---|
| `F` | Forward |
| `B` | Backward |
| `L` | Pivot left |
| `R` | Pivot right |
| `S` | Stop |
| `0`–`9` | Set speed; `0` is zero speed, `9` is maximum |
| `H` | Heartbeat only; required when STATE-wire detection is disabled |

With `USE_BT_STATE = true`, wire HC-05 STATE to D13. If STATE is unavailable, set it to `false` and make the Bluetooth app send `H` about once per second. Without STATE or periodic heartbeat packets, a Nano cannot reliably distinguish an idle connected phone from a disconnected phone.

## Main configuration values

Edit the configuration section near the top of the sketch:

- `AUTONOMOUS_SPEED`, `AUTONOMOUS_TURN_SPEED`: autonomous motor PWM.
- `LEFT_MOTOR_INVERTED`, `RIGHT_MOTOR_INVERTED`: correct motor direction without changing pins.
- `LEFT_SPEED_TRIM`, `RIGHT_SPEED_TRIM`: compensate for unequal motor speeds.
- `OBSTACLE_DISTANCE_CM`, `OPEN_PATH_DISTANCE_CM`: obstacle and route-clearance thresholds.
- `SERVO_LEFT_DEG`, `SERVO_CENTER_DEG`, `SERVO_RIGHT_DEG`: scan angles.
- `SERVO_SETTLE_MS`: servo/ultrasonic settling time.
- `SIDE_TURN_MS`, `RECOVERY_180_TURN_MS`: timed turns; calibrate for the chassis.
- `IR_ACTIVE_LOW`: choose the sensor's active output level.
- `LDR_DARK_WHEN_BELOW`, `LDR_BLACKOUT_THRESHOLD`, `LDR_WAKE_THRESHOLD`: light-sensor polarity and hysteresis.
- `BLACKOUT_CONFIRM_MS`, `WAKE_CONFIRM_MS`: prevent rapid sleep/wake changes.
- `USE_BT_STATE`, `BT_HEARTBEAT_TIMEOUT_MS`: Bluetooth connection-detection method.

## Calibration sequence

1. First test with the drive wheels lifted clear of the floor. Confirm `F` makes both wheels drive forward. If one wheel is reversed, change that motor's `*_MOTOR_INVERTED` setting.
2. Set `AUTONOMOUS_SPEED` low initially. Adjust `LEFT_SPEED_TRIM` or `RIGHT_SPEED_TRIM` so the car tracks reasonably straight.
3. Confirm each IR sensor's output when clear and when an obstacle is close. Set `IR_ACTIVE_LOW` accordingly and adjust each sensor's onboard potentiometer.
4. Confirm the servo physically points left at 30°, centre at 90°, and right at 150°. Adjust the three angle constants if the bracket is mounted differently. Increase `SERVO_SETTLE_MS` if readings are taken before the servo stops moving.
5. Compare HC-SR04 readings with a few known distances. Adjust the two distance thresholds to suit the car's speed and braking distance.
6. Measure A2 in bright room light and with the LDR fully covered using a temporary analog-read test sketch. Set the polarity and thresholds to your divider arrangement. For a low-reading-in-dark arrangement, the wake threshold should be higher than the blackout threshold. For a high-reading-in-dark arrangement, it should be lower.
7. Calibrate `SIDE_TURN_MS` and `RECOVERY_180_TURN_MS` on the actual floor. With no encoders, these are time estimates, not exact angles.

## Electrical and Nano notes

- Connect Arduino GND, L298N GND, sensor GND, HC-05 GND and the external servo-supply GND together.
- Power the motors from a suitable motor supply through the L298N. Do not route motor current through the Nano 5V pin.
- SG90 servos can draw current spikes. Use a suitable regulated 5V supply if the Nano resets or the OLED flickers; connect grounds together.
- HC-05 RXD is a 3.3V input on common modules. Use a resistor divider/level shifter from Nano D4 to HC-05 RXD. HC-05 TXD to Nano D2 is generally a valid 3.3V-to-5V logic connection.
- Remove the L298N ENA/ENB jumpers so D5/D6 can control speed with PWM.
- Servo uses Timer1 on the classic Nano and disables PWM on D9/D10. This sketch uses D9/D10 only as digital direction pins; motor PWM is on D5/D6.
- The 128×64 OLED framebuffer occupies 1024 bytes, half of the Nano's 2 KB SRAM. The sketch avoids Arduino `String` allocation and large dynamic buffers. Avoid adding large arrays or text-heavy features without checking memory use.

## Known hardware limitations

- A typical SSD1306 0.96-inch display is monochrome. The boot scan beam is drawn in white; it cannot be rendered blue on a monochrome panel.
- The firmware treats a no-echo ultrasonic result as clear by default (`NO_ECHO_MEANS_CLEAR = true`), because an obstacle may be beyond the sensor's useful range. If you prefer a conservative stop on missing echoes, set it to `false`, but a disconnected sensor can then look like a permanently blocked path.
- Timed turns vary with battery voltage, floor friction, motor mismatch and wheel diameter. Tune them on the actual robot.
- When STATE detection is disabled, Bluetooth idle detection depends on the app's periodic `H` heartbeat.
