#include <Arduino.h>
#include "target.h"

TBeamS3SupremeBoard board;

#ifdef MESH_DEBUG
void scanDevices(TwoWire *w);
uint32_t deviceOnline = 0x00;
#endif

bool pmuIntFlag;

#ifndef LORA_CR
  #define LORA_CR      5
#endif

#if defined(P_LORA_SCLK)
  static SPIClass spi;
  RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
#else
  RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY);
#endif

WRAPPER_CLASS radio_driver(radio, board);

ESP32RTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
SensorManager sensors;

static void setPMUIntFlag(){
  pmuIntFlag = true;
}

bool TBeamS3SupremeBoard::power_init()
{
  bool result = PMU.begin(PMU_WIRE_PORT, I2C_PMU_ADD, PIN_BOARD_SDA1, PIN_BOARD_SCL1);
  if (result == false) {
    MESH_DEBUG_PRINTLN("power is not online..."); while (1)delay(50);
  }
  MESH_DEBUG_PRINTLN("Setting charge led");
  PMU.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

  // Set up PMU interrupts
  MESH_DEBUG_PRINTLN("Setting up PMU interrupts");
  pinMode(PIN_PMU_IRQ, INPUT_PULLUP);
  attachInterrupt(PIN_PMU_IRQ, setPMUIntFlag, FALLING);

  // GPS
  MESH_DEBUG_PRINTLN("Setting and enabling a-ldo4 for GPS");
  PMU.setALDO4Voltage(3300);
  PMU.enableALDO4(); // disable to save power

  // Lora
  MESH_DEBUG_PRINTLN("Setting and enabling a-ldo3 for LoRa");
  PMU.setALDO3Voltage(3300);
  PMU.enableALDO3();

  // To avoid SPI bus issues during power up, reset OLED, sensor, and SD card supplies
  MESH_DEBUG_PRINTLN("Reset a-ldo1&2 and b-ldo1");
  if (ESP_SLEEP_WAKEUP_UNDEFINED == esp_sleep_get_wakeup_cause())
  {
    PMU.enableALDO1();
    PMU.enableALDO2();
    PMU.enableBLDO1();
    delay(250);
  }

  // BME280 and OLED
  MESH_DEBUG_PRINTLN("Setting and enabling a-ldo1 for oled");
  PMU.setALDO1Voltage(3300);
  PMU.enableALDO1();

  // QMC6310U
  MESH_DEBUG_PRINTLN("Setting and enabling a-ldo2 for QMC");
  PMU.setALDO2Voltage(3300);
  PMU.enableALDO2();

  // SD card
  MESH_DEBUG_PRINTLN("Setting and enabling b-ldo2 for SD card");
  PMU.setBLDO1Voltage(3300);
  PMU.enableBLDO1();

  // Out to header pins
  MESH_DEBUG_PRINTLN("Setting and enabling b-ldo2 for output to header");
  PMU.setBLDO2Voltage(3300);
  PMU.enableBLDO2();

  MESH_DEBUG_PRINTLN("Setting and enabling dcdc4 for output to header");
  PMU.setDC4Voltage(XPOWERS_AXP2101_DCDC4_VOL2_MAX); // 1.8V
  PMU.enableDC4();

  MESH_DEBUG_PRINTLN("Setting and enabling dcdc5 for output to header");
  PMU.setDC5Voltage(3300);
  PMU.enableDC5();

  // Other power rails
  MESH_DEBUG_PRINTLN("Setting and enabling dcdc3 for ?");
  PMU.setDC3Voltage(3300); // doesn't go anywhere in the schematic??
  PMU.enableDC3();

  // Unused power rails
  MESH_DEBUG_PRINTLN("Disabling unused supplies dcdc2, dldo1 and dldo2");
  PMU.disableDC2();
  PMU.disableDLDO1();
  PMU.disableDLDO2();

  // Set charge current to 300mA
  MESH_DEBUG_PRINTLN("Setting battery charge current limit and voltage");
  PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
  PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

  // enable battery voltage measurement
  MESH_DEBUG_PRINTLN("Enabling battery measurement");
  PMU.enableBattVoltageMeasure();

  // Reset and re-enable PMU interrupts
  MESH_DEBUG_PRINTLN("Re-enable interrupts");
  PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  PMU.clearIrqStatus();
  PMU.enableIRQ(
      XPOWERS_AXP2101_BAT_INSERT_IRQ | XPOWERS_AXP2101_BAT_REMOVE_IRQ |    // Battery interrupts
      XPOWERS_AXP2101_VBUS_INSERT_IRQ | XPOWERS_AXP2101_VBUS_REMOVE_IRQ |  // VBUS interrupts
      XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ |     // Power Key interrupts
      XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | XPOWERS_AXP2101_BAT_CHG_START_IRQ // Charging interrupts
  );
#ifdef MESH_DEBUG
  printPMU();
  scanDevices(&Wire);
  scanDevices(&Wire1);

#endif

  // Set the power key off press time
  PMU.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
  return true;
}

bool radio_init() {
  fallback_clock.begin();
  Wire1.begin(PIN_BOARD_SDA1,PIN_BOARD_SCL1);
  rtc_clock.begin(Wire1);
  
#ifdef SX126X_DIO3_TCXO_VOLTAGE
  float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 1.6f;
#endif

#if defined(P_LORA_SCLK)
  spi.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
#endif
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, LORA_TX_POWER, 8, tcxo);
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    return false;  // fail
  }

  radio.setCRC(1);
  
  return true;  // success
}

uint32_t radio_get_rng_seed() {
  return radio.random(0x7FFFFFFF);
}

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr) {
  radio.setFrequency(freq);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bw);
  radio.setCodingRate(cr);
}

void radio_set_tx_power(uint8_t dbm) {
  radio.setOutputPower(dbm);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}

// BME280
bool TBeamS3SupremeBoard::bme_init()
{
  bool status;
  status = bme.begin(I2C_BME280_ADD, BME_WIRE_PORT_ADDR);
  if (!status)
  {
    MESH_DEBUG_PRINTLN("Could not find a valid BME280 sensor, check wiring!");
  }
  else
  {
#ifdef MESH_DEBUG
    bme_debug();
#endif
  }
  return status;
}

#ifdef MESH_DEBUG
void TBeamS3SupremeBoard::printPMU()
{
    Serial.print("isCharging:"); Serial.println(PMU.isCharging() ? "YES" : "NO");
    Serial.print("isDischarge:"); Serial.println(PMU.isDischarge() ? "YES" : "NO");
    Serial.print("isVbusIn:"); Serial.println(PMU.isVbusIn() ? "YES" : "NO");
    Serial.print("getBattVoltage:"); Serial.print(PMU.getBattVoltage()); Serial.println("mV");
    Serial.print("getVbusVoltage:"); Serial.print(PMU.getVbusVoltage()); Serial.println("mV");
    Serial.print("getSystemVoltage:"); Serial.print(PMU.getSystemVoltage()); Serial.println("mV");

    // The battery percentage may be inaccurate at first use, the PMU will automatically
    // learn the battery curve and will automatically calibrate the battery percentage
    // after a charge and discharge cycle
    if (PMU.isBatteryConnect()) {
        Serial.print("getBatteryPercent:"); Serial.print(PMU.getBatteryPercent()); Serial.println("%");
    }

    Serial.println();
}

void TBeamS3SupremeBoard::bme_debug()
{
  MESH_DEBUG_PRINTLN("Humidity: %10.1f", humidity());
  MESH_DEBUG_PRINTLN("Pressure: %10.1f\n", pressure());
  MESH_DEBUG_PRINTLN("Temperature: %10.1f\n", temperature());
}

enum {
    POWERMANAGE_ONLINE  = _BV(0),
    DISPLAY_ONLINE      = _BV(1),
    RADIO_ONLINE        = _BV(2),
    GPS_ONLINE          = _BV(3),
    PSRAM_ONLINE        = _BV(4),
    SDCARD_ONLINE       = _BV(5),
    AXDL345_ONLINE      = _BV(6),
    BME280_ONLINE       = _BV(7),
    BMP280_ONLINE       = _BV(8),
    BME680_ONLINE       = _BV(9),
    QMC6310_ONLINE      = _BV(10),
    QMI8658_ONLINE      = _BV(11),
    PCF8563_ONLINE      = _BV(12),
    OSC32768_ONLINE      = _BV(13),
};

void scanDevices(TwoWire *w)
{
    uint8_t err, addr;
    int nDevices = 0;
    uint32_t start = 0;

    Serial.println("I2C Devices scanning");
    for (addr = 1; addr < 127; addr++) {
        start = millis();
        w->beginTransmission(addr); delay(2);
        err = w->endTransmission();
        if (err == 0) {
            nDevices++;
            switch (addr) {
            case 0x77:
            case 0x76:
                Serial.println("\tFind BMX280 Sensor!");
                deviceOnline |= BME280_ONLINE;
                break;
            case 0x34:
                Serial.println("\tFind AXP192/AXP2101 PMU!");
                deviceOnline |= POWERMANAGE_ONLINE;
                break;
            case 0x3C:
                Serial.println("\tFind SSD1306/SH1106 dispaly!");
                deviceOnline |= DISPLAY_ONLINE;
                break;
            case 0x51:
                Serial.println("\tFind PCF8563 RTC!");
                deviceOnline |= PCF8563_ONLINE;
                break;
            case 0x1C:
                Serial.println("\tFind QMC6310 MAG Sensor!");
                deviceOnline |= QMC6310_ONLINE;
                break;
            default:
                Serial.print("\tI2C device found at address 0x");
                if (addr < 16) {
                    Serial.print("0");
                }
                Serial.print(addr, HEX);
                Serial.println(" !");
                break;
            }

        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16) {
                Serial.print("0");
            }
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");

    Serial.println("Scan devices done.");
    Serial.println("\n");
}
#endif