#pragma once

#include <MeshCore.h>
#include <Arduino.h>

// LoRa and SPI pins
#define  P_LORA_DIO_1   (32 + 1)  // P1.1
#define  P_LORA_NSS     (0 + 12)  // P0.12
#define  P_LORA_RESET   (32 + 10) // P1.10
#define  P_LORA_BUSY    (0 + 7)   // P0.7
#define  P_LORA_SCLK    (0 + 11)  // P0.11
#define  P_LORA_MISO    (32 + 8)  // P1.8
#define  P_LORA_MOSI    (32 + 9)  // P0.9

class T114Board : public mesh::MainBoard {
protected:
  uint8_t startup_reason;
  uint8_t btn_prev_state;

public:
  void begin();

  uint16_t getBattMilliVolts() override {
  #ifdef BATTERY_PIN
    analogReadResolution(12);
    float volts = (analogRead(BATTERY_PIN) * ADC_MULTIPLIER * AREF_VOLTAGE) / 4096;
    return volts * 1000;
  #else
    return 0;
  #endif
  }

  uint8_t getStartupReason() const override { return startup_reason; }

  const char* getManufacturerName() const override {
    return "Heltec T114";
  }

  int buttonStateChanged() {
  #ifdef BUTTON_PIN
    uint8_t v = digitalRead(BUTTON_PIN);
    if (v != btn_prev_state) {
      btn_prev_state = v;
      return (v == LOW) ? 1 : -1;
    }
  #endif
    return 0;
  }

  void powerOff() {
    #ifdef HAS_GPS
        digitalWrite(GPS_VRTC_EN, LOW);
        digitalWrite(GPS_RESET, LOW);
        digitalWrite(GPS_SLEEP_INT, LOW);
        digitalWrite(GPS_RTC_INT, LOW);
        pinMode(GPS_RESETB, OUTPUT);
        digitalWrite(GPS_RESETB, LOW);
    #endif
    
    #ifdef BUZZER_EN
        digitalWrite(BUZZER_EN, LOW);
    #endif
    
    #ifdef PIN_3V3_EN
        digitalWrite(PIN_3V3_EN, LOW);
    #endif

    #ifdef LED_PIN
    digitalWrite(LED_PIN, LOW);
    #endif
    #ifdef BUTTON_PIN
    nrf_gpio_cfg_sense_input(digitalPinToInterrupt(BUTTON_PIN), NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
    #endif
    sd_power_system_off();
  }

  void reboot() override {
    NVIC_SystemReset();
  }

//  bool startOTAUpdate() override;
};