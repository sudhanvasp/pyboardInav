/*
 * iNAV target for MicroPython pyboard v1.1 (STM32F405RGT6).
 *
 * Motors   : TIM5_CH1-4 on X1-X4 (PA0-PA3) — PWM or DShot
 * IMU      : MPU-6050 on I2C2 (Y9=PB10=SCL, Y10=PB11=SDA)
 * Baro     : BMP-280 on I2C2 (same bus, addr 0x76)
 * ELRS/RX  : UART6 (Y1=PC6=TX, Y2=PC7=RX)
 * Aux UART : UART1 (X9=PB6=TX, X10=PB7=RX)
 * Blackbox : onboard SD card via SDIO (4-bit)
 * VCP      : USB OTG FS (MSP / Configurator)
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "PYBD"
#define USBD_PRODUCT_STRING     "PYBOARD"

// ============================================================
// Status LEDs
// Pyboard v1.1 LEDs: LED1(green)=PA14, LED2(orange)=PA15,
//                    LED3(red)=PB4,     LED4(blue)=PB5
// PA14 = SWCLK — iNAV reconfigures it as GPIO after boot.
// ============================================================
#define LED0                    PA14   // green — iNAV status
#define LED1                    PB4    // red   — armed indicator

// ============================================================
// I2C bus 2  —  Y9 (PB10) = SCL,  Y10 (PB11) = SDA
// IMU (MPU-6050 @ 0x68) and baro (BMP-280 @ 0x76) share bus
// ============================================================
#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                PB10   // Y9
#define I2C2_SDA                PB11   // Y10

// ============================================================
// IMU: MPU-6050 via I2C2
// iNAV's MPU6500 driver is register-compatible with MPU-6050.
// Align = CW0_DEG if the board sits flat with USB pointing right.
// Adjust to CW90_DEG / CW180_DEG / CW270_DEG to match mount.
// ============================================================
#define USE_IMU_MPU6500
#define IMU_MPU6500_ALIGN       CW0_DEG
#define MPU6500_I2C_BUS         BUS_I2C2

// Optional: connect MPU-6050 INT pin to Y12 (PB1) for EXTI sync.
// Solder a wire from sensor INT pad to Y12 on the pyboard header.
// #define MPU_INT_EXTI         PB1    // Y12 (optional)

// ============================================================
// Barometer: BMP-280 via I2C2
// ============================================================
#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C2
#define USE_BARO_BMP280

// ============================================================
// USB VCP  (MSP / Configurator link)
// ============================================================
#define USE_VCP

// ============================================================
// UART1  —  X9 (PB6) = TX,  X10 (PB7) = RX
// General purpose; use for telemetry, MSP passthrough, etc.
// ============================================================
#define USE_UART1
#define UART1_TX_PIN            PB6    // X9
#define UART1_RX_PIN            PB7    // X10

// ============================================================
// UART6  —  Y1 (PC6) = TX,  Y2 (PC7) = RX
// Default assignment: Serial RX (ELRS / CRSF at 420000 baud)
// ============================================================
#define USE_UART6
#define UART6_TX_PIN            PC6    // Y1
#define UART6_RX_PIN            PC7    // Y2

#define SERIAL_PORT_COUNT       3      // VCP + UART1 + UART6

// ============================================================
// ADC
// X11 (PC4) — connect to battery voltage via 10k/1k divider
// X12 (PC5) — optional current sensor output
// ============================================================
#define USE_ADC
#define ADC_CHANNEL_1_PIN               PC4    // X11 — VBAT
#define ADC_CHANNEL_2_PIN               PC5    // X12 — current (optional)
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2

// ============================================================
// Onboard SD card blackbox via SDIO (4-bit)
// Pyboard SD slot uses PC8/PC9/PC10/PC11 (data), PC12 (CLK),
// PD2 (CMD).  PC8/PC9 are also Y3/Y4 — do NOT use those header
// pins for anything else when SD logging is active.
// Default DMA for SDIO on STM32F405: DMA2_Stream3_Channel4.
// ============================================================
#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDCARD_SDIO_4BIT
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

// ============================================================
// Default sensor set and RX type
// ============================================================
#define SENSORS_SET             (SENSOR_ACC | SENSOR_BARO)
#define DEFAULT_RX_TYPE         RX_TYPE_SERIAL
#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_BLACKBOX)

// ============================================================
// DShot / ESC telemetry
// ============================================================
#define USE_DSHOT
#define USE_ESC_SENSOR

// ============================================================
// Motor count
// ============================================================
#define MAX_PWM_OUTPUT_PORTS    4
#define TARGET_MOTOR_COUNT      4

// ============================================================
// IO port masks — allow all pins; iNAV will manage conflicts
// ============================================================
#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))   // PD2 = SDIO_CMD only
