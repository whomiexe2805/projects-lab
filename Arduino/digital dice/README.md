# S-TECH Digital Dice

A simple push-button digital dice for an **Arduino Nano**, an **I2C SSD1306 OLED (128x64)** and one push button.

> **Important randomness note:** This sketch demonstrates analog-noise sampling and the Von Neumann bit-extraction method. Leaving A0 unconnected does **not** make the Nano a validated True Random Number Generator (TRNG). The noise/bit stream may be biased, correlated, or contain little usable entropy. This project is for learning and experimentation only—not for cryptography, security, gambling, or high-stakes decisions.

## Features

- OLED welcome screen
- Push-button dice roll
- Rolling animation
- Dice face display for values 1–6
- Experimental A0 sampling
- Von Neumann pair extraction (`01` and `10` accepted; `00` and `11` discarded)
- Rejection of 3-bit values 6 and 7, avoiding modulo bias in the final 1–6 mapping
- Timeout if usable bit pairs are not obtained

## Required components

| Component | Quantity |
|---|---:|
| Arduino Nano (ATmega328P) | 1 |
| SSD1306 I2C OLED, 128x64 | 1 |
| Momentary push button | 1 |
| Jumper wires | As required |
| USB cable | 1 |

## Wiring

| Module / component | Arduino Nano |
|---|---|
| OLED SDA | A4 |
| OLED SCL | A5 |
| OLED GND | GND |
| OLED VCC | Supply supported by your OLED breakout |
| Push button terminal 1 | D2 |
| Push button terminal 2 | GND |
| Experimental sample input | A0 (left unconnected for this demonstration) |

The button uses the Nano's internal pull-up resistor (`INPUT_PULLUP`), so it is pressed when D2 reads LOW.

**OLED voltage:** Check the exact OLED breakout board. Some modules accept 5V on VCC because they include a regulator; others are 3.3V-only. Do not assume every OLED module is 5V tolerant.

## Software requirements

Install these libraries from Arduino IDE Library Manager:

1. **Adafruit GFX Library**
2. **Adafruit SSD1306**

Select the correct board and processor for your Nano under **Tools → Board** and **Tools → Processor**. Many classic Nano clones use **ATmega328P (Old Bootloader)**.

Open `S-TECH_Digital_Dice.ino` and upload it.

## How to use

1. Connect the OLED and button according to the wiring table.
2. Connect the Nano to USB.
3. Upload the `.ino` sketch.
4. The OLED displays the welcome screen.
5. Press and release the button to roll.
6. The OLED shows the dice face.
7. Press again for another roll.

## How the experimental bit extraction works

1. The sketch reads the least-significant bit of two A0 ADC samples.
2. It treats the two bits as a pair.
3. `00` and `11` are discarded.
4. `01` and `10` produce one output bit.
5. Three extracted bits form a value from 0 to 7.
6. Values 6 and 7 are rejected; values 0–5 map to dice values 1–6.

The rejection step avoids the small modulo bias that would occur if an arbitrary 32-bit value were simply reduced with `% 6`. However, unbiased mapping does **not** prove the input bits are random.

## Limitations and next engineering step

- A floating analog input is affected by the board, nearby electrical activity, wiring, and environment.
- The Von Neumann method can remove certain forms of bias only when its input assumptions are met; it does not create entropy.
- Passing a few basic statistical checks is not proof of a secure or high-quality TRNG.
- For a serious hardware TRNG, design a suitable physical entropy source, condition its output, and validate it with appropriate tests and health checks. Do not connect an improvised noise circuit directly to A0 without checking its voltage and electrical limits.

## Troubleshooting

### OLED stays blank
- Confirm SDA → A4 and SCL → A5.
- Check the OLED's supported supply voltage.
- Many displays use I2C address `0x3C`; some use `0x3D`. Change `OLED_ADDRESS` in the sketch if required.
- Confirm both Adafruit libraries are installed.

### Nano does not upload
- Try the other Nano processor/bootloader option under **Tools → Processor**.
- Check the selected COM/serial port and USB data cable.

### Screen reports “NO USABLE SAMPLE”
The sampled A0 bit pairs did not yield enough differing pairs before the timeout. This is possible with a floating input and is one reason this sketch is only an experiment, not a dependable TRNG.

## Project details

- Project: S-TECH Digital Dice
- Platform: Arduino Nano
- Display: SSD1306 I2C OLED, 128x64
- Input: One push button
- Randomness approach: Experimental analog sampling + Von Neumann extraction
