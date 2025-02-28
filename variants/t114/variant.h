#pragma once
#include <Arduino.h>
#define MASTER_CLOCK (64000000ul)
// 32k Crystal
#define USE_LFXO

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define HELTEC_MESH_NODE_T114

// Buttons
#define PIN_BUTTON1 (32 + 10)

// Configurre display
#define USE_ST7789
#define ST7789_BUSY -1
#define ST7789_RS 12  // DC
#define ST7789_NSS 11
#define ST7789_SDA 41 // MOSI
#define ST7789_RESET 2
#define ST7789_SCK 40
#define VTFT_LEDA 15
#define ST7789_MISO -1
#define VTFT_CTRL 3
#define TFT_BACKLIGHT_ON LOW
#define ST7789_SPI_HOST SPI1_HOST
#define SPI_READ_FREQUENCY 16000000
#define SPI_FREQUENCY 40000000
#define TFT_HEIGHT 135
#define TFT_WIDTH 240
#define TFT_OFFSET_X 0
#define TFT_OFFSET_Y 0

#define NUM_DIGITAL_PINS (48)
#define PINS_COUNT (48)
#define NUM_ANALOG_INPUTS (1)
#define NUM_ANALOG_OUTPUTS (0)

// LEDs
// Fakeout for bluefruit
#define LED_BLUE PIN_LED1

// Green LED
#define PIN_LED1 (32 + 3)
#define LED_GREEN PIN_LED1
#define LED_BUILTIN LED_GREEN
#define LED_STATE_ON 0

#define HAS_NEOPIXEL
#define NEOPIXEL_COUNT 2
#define NEOPIXEL_DATA 14
#define NEOPIXEL_TYPE (NEO_GRB + NEO_KHZ800)


// I2C
#define WIRE_INTERFACES_COUNT 2
#define PIN_WIRE_SDA (0 + 26)
#define PIN_WIRE_SCL (0 + 27)
#define PIN_WIRE1_SDA (0 + 16)
#define PIN_WIRE1_SCL (0 + 13)
#define PIN_QSPI_SCK (32 + 14)
#define PIN_QSPI_CS (32 + 15)
#define PIN_QSPI_IO0 (32 + 12)
#define PIN_QSPI_IO1 (32 + 13)
#define PIN_QSPI_IO2 (0 + 7)
#define PIN_QSPI_IO3 (0 + 5)
#define EXTERNAL_FLASH_DEVICES MX25R1635F
#define EXTERNAL_FLASH_USE_QSPI

// LoRa
#define USE_SX1262
#define SX126X_CS (0 + 24)
#define LORA_CS (0 + 24)
#define SX126X_RESET (0 + 25)
#define SX126X_DIO1 (0 + 20)
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8
#define SX126X_BUSY (0 + 17)

#define PIN_SPI1_MISO ST7789_MISO
#define PIN_SPI1_MOSI ST7789_SDA
#define PIN_SPI1_SCK ST7789_SCK

// GPIO
#define PIN_SERIAL2_RX (0 + 9)
#define PIN_SERIAL2_TX (0 + 10)


// GPS
#define GPS_L76K
#define GPS_THREAD_INTERVAL 50
#define GPS_RESET_MODE LOW
#define VEXT_ENABLE (0 + 21)
#define PERIPHERAL_WARMUP_MS 1000
#define VEXT_ON_VALUE HIGH
#define PIN_SERIAL1_RX GPS_TX_PIN
#define PIN_SERIAL1_TX GPS_RX_PIN
#define PIN_GPS_STANDBY (32 + 2)
#define PIN_GPS_PPS (32 + 4)
#define GPS_RX_PIN (32 + 7)
#define GPS_TX_PIN (32 + 5)

// PCF8563
#define PCF8563_RTC 0x51

// SPI
#define SPI_INTERFACES_COUNT 2
#define PIN_SPI_SCK (0 + 19)
#define PIN_SPI_MOSI (0 + 22)
#define PIN_SPI_MISO (0 + 23)

#define ADC_CTRL_ENABLED HIGH
#define ADC_CTRL 6
#define BATTERY_PIN 4
#define ADC_RESOLUTION 14
#define BATTERY_SENSE_RESOLUTION_BITS 12
#define BATTERY_SENSE_RESOLUTION 4096.0

#ifdef AREF_VOLTAGE
#undef AREF_VOLTAGE
#endif
#define AREF_VOLTAGE 3.0

#define VBAT_AR_INTERNAL AR_INTERNAL_3_0
#define ADC_MULTIPLIER (4.90F)

#define HAS_RTC 0
#ifdef __cplusplus
}
#endif