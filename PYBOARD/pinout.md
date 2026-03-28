# PYBOARD iNAV Target — Pinout

Pyboard v1.1 · STM32F405RGT6 · iNAV firmware

```
                    ┌──────────────────────────────────────────────────────┐
                    │                  PYBOARD v1.1                        │
                    │                                                      │
         ┌──────────┤  X1  PA0  TIM5_CH1 ── M1 (front-right ESC)          │
         │  MOTORS  │  X2  PA1  TIM5_CH2 ── M2 (rear-right  ESC)          │
         │  PWM /   │  X3  PA2  TIM5_CH3 ── M3 (rear-left   ESC)          │
         │  DShot   │  X4  PA3  TIM5_CH4 ── M4 (front-left  ESC)          │
         └──────────┤                                                      │
                    │  X5  PA4  (free — DAC / ADC)                         │
                    │  X6  PA5  (free — DAC / ADC)                         │
                    │  X7  PA6  (free — ADC / TIM3)                        │
                    │  X8  PA7  (free — ADC / TIM3)                        │
         ┌──────────┤                                                      │
         │  UART1   │  X9  PB6  UART1_TX ── spare serial TX                │
         │  (spare) │  X10 PB7  UART1_RX ── spare serial RX                │
         └──────────┤                                                      │
         ┌──────────┤  X11 PC4  ADC1_IN14 ── VBAT (10k/1k divider to bat) │
         │   ADC    │  X12 PC5  ADC1_IN15 ── current sensor (optional)     │
         └──────────┤                                                      │
          ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─│
         ┌──────────┤  Y1  PC6  UART6_TX ── ELRS RX pin  (optional telem)  │
         │  ELRS /  │  Y2  PC7  UART6_RX ── ELRS TX pin  (CRSF 420k baud) │
         │  CRSF    └──────────────────────────────────────────────────────┘
         └──────────┐
                    │  Y3  PC8  (SDIO_D0 — SD card, free if no SD)
                    │  Y4  PC9  (SDIO_D1 — SD card, free if no SD)
                    │  Y5  PB12 (free — SPI2 / GPIO)
                    │  Y6  PB13 (free — SPI2 / GPIO)
                    │  Y7  PB14 (free — SPI2 / GPIO)
                    │  Y8  PB15 (free — SPI2 / GPIO)
         ┌──────────┤
         │  IMU &   │  Y9  PB10 I2C2_SCL ── MPU-6050 SCL
         │  BARO    │  Y10 PB11 I2C2_SDA ── MPU-6050 SDA + BMP-280 SDA
         │  I2C2    │                       (same bus, addr 0x68 / 0x76)
         └──────────┤
                    │  Y11 PB0  (free — ADC / TIM3_CH3)
                    │  Y12 PB1  (free — ADC / TIM3_CH4 / MPU INT optional)
                    └──────────────────────────────────────────────────────

      Internal (SD card slot — not on headers):
          PC8  SDIO_D0     PC9  SDIO_D1
          PC10 SDIO_D2     PC11 SDIO_D3
          PC12 SDIO_CLK    PD2  SDIO_CMD
      → Blackbox SD logging works automatically via the onboard SD slot.

      USB  (PA11=DM, PA12=DP — internal):
      → USB VCP always available for Configurator / MSP.

      LEDs (onboard):
          PA14  LED1 green  ── iNAV status / disarmed blink
          PB4   LED3 red    ── armed indicator
```

## Sensor Wiring (I2C2 bus)

| Sensor | Pin | Pyboard | Notes |
|--------|-----|---------|-------|
| MPU-6050 SCL | PB10 | Y9 | 3.3V I2C, 4.7kΩ pull-up to 3.3V |
| MPU-6050 SDA | PB11 | Y10 | shared with BMP-280 |
| MPU-6050 VCC | 3.3V | 3V3 | |
| MPU-6050 GND | GND | GND | |
| MPU-6050 INT | PB1 | Y12 | optional — uncomment `MPU_INT_EXTI` in target.h |
| BMP-280 SCL | PB10 | Y9 | same bus |
| BMP-280 SDA | PB11 | Y10 | |
| BMP-280 VCC | 3.3V | 3V3 | |
| BMP-280 GND | GND | GND | |
| BMP-280 SDO | GND | GND | sets I2C address = 0x76 |
| BMP-280 CSB | 3.3V | 3V3 | forces I2C mode |

## Motor Wiring (X-frame QuadX)

| Motor | Pyboard pin | STM32 | Position |
|-------|-------------|-------|----------|
| M1 | X1 | PA0 / TIM5_CH1 | Front-right |
| M2 | X2 | PA1 / TIM5_CH2 | Rear-right |
| M3 | X3 | PA2 / TIM5_CH3 | Rear-left |
| M4 | X4 | PA3 / TIM5_CH4 | Front-left |

Connect signal wire from each ESC to the corresponding pin.
All four channels share TIM5 — DShot and bidirectional DShot work.

## ELRS Receiver (UART6)

| ELRS pad | Pyboard pin | STM32 |
|----------|-------------|-------|
| TX (CRSF out) | Y2 | PC7 / UART6_RX |
| RX (CRSF in)  | Y1 | PC6 / UART6_TX |
| GND | GND | |
| 5V | VIN | 5V from USB or battery regulator |

In iNAV Configurator → Ports tab: set UART6 → Serial RX → CRSF.

## VBAT Sensing (X11)

```
Battery+ ──┤ 10kΩ ├──┬── X11 (PC4)
                     │
                   1kΩ
                     │
                    GND
```
Scale factor in Configurator: set to match your divider ratio (11.0 for 10k/1k).

## Building

```bash
cd /path/to/inav
mkdir build && cd build
cmake .. -DTARGET=PYBOARD
make -j$(nproc)
```

Flash the resulting `PYBOARD.hex` via DFU:
```bash
dfu-util -D PYBOARD.hex -a 0 --dfuse-address 0x08000000
```
Or use STM32CubeProgrammer / iNAV's built-in flashing in the Configurator.
