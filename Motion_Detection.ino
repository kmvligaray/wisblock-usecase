/**
 * @file MotionDetection_LoRaWAN.ino
 * @brief Motion detection using RAK12006 PIR sensor with RAK4631 over LoRaWAN
 */

#include <Arduino.h>
#include <LoRaWan-RAK4630.h>
#include <SPI.h>
#include <Wire.h>

// Hardware Definitions
#define PIR_PIN WB_IO6  // PIR sensor input pin
#define LED_PIN LED_BUILTIN  // Optional motion indicator
#define PIN_VBAT A0
uint32_t vbat_pin = PIN_VBAT;

// LoRaWAN Settings
bool doOTAA = true;
#define LORAWAN_DATERATE DR_1
#define LORAWAN_TX_POWER TX_POWER_5
#define JOINREQ_NBTRIALS 3
#define LORAWAN_APP_PORT 2

// VBat Constants
#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER_COMP (1.73)    // (1.403F) // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_EU868;
DeviceClass_t g_CurrentClass = CLASS_A;
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;

// LoRaWAN Buffers
uint8_t m_lora_app_data_buffer[64];
lmh_app_data_t m_lora_app_data = { m_lora_app_data_buffer, 0, 0, 0, 0 };

// PIR Detection Variables
int currentStatus = 1;
int lastStatus = 1;

// OTAA Credentials (Replace with yours)
uint8_t nodeDeviceEUI[8] = {0xAC, 0xF3, 0x29, 0x11, 0xFE, 0x3C, 0x40, 0x54};
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16] = {0xa4, 0x2c, 0xd0, 0x01, 0xc3, 0x92, 0x87, 0x55, 0x74, 0xd2, 0xc6, 0xcf, 0x50, 0x30, 0xcc, 0x8a};
//

// LoRaWAN Callbacks
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);

// Function prototype
float readVBAT(void);
uint8_t mvToPercent(float mvolts);
uint8_t mvToLoRaWanBattVal(float mvolts);


// Setup LoRaWAN parameters
static lmh_param_t lora_param_init = {
  LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK,
  JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF
};

static lmh_callback_t lora_callbacks = {
  BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
  lorawan_rx_handler, lorawan_has_joined_handler,
  lorawan_confirm_class_handler, lorawan_join_failed_handler
};

void setup() {
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(100);
  Serial.println("Starting Motion Detection with LoRaWAN...");

  lora_rak4630_init();

  // Set OTAA credentials before LoRaWAN init
  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);
  uint32_t err_code;
  // Setup LoRaWAN credentials
  err_code = lmh_init(&lora_callbacks, lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  // lmh_setDevEui(nodeDeviceEUI);
  // lmh_setAppEui(nodeAppEUI);
  // lmh_setAppKey(nodeAppKey);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }
  lmh_join();
}

void loop() {
  currentStatus = digitalRead(PIR_PIN);


  if (currentStatus == LOW && lastStatus == HIGH) { // Motion detected
    Serial.println("Motion detected!");
    digitalWrite(LED_PIN, HIGH);
    float vbat_mv = readVBAT();

    // Convert from raw mv to percentage (based on LIPO chemistry)
    uint8_t vbat_per = mvToPercent(vbat_mv);
    uint8_t vbat_lora =mvToLoRaWanBattVal(vbat_mv);

    // Display the results

    Serial.print("LIPO = ");
    Serial.print(vbat_mv);
    Serial.print(" mV (");
    Serial.print(vbat_per);
    Serial.print("%) LoRaWan Batt ");
    Serial.println();
  
    send_motion_packet(vbat_lora);
    delay(2000);  // Debounce time to avoid message spamming
  } else if (currentStatus == HIGH && lastStatus == LOW) {
    Serial.println("No motion.");
    digitalWrite(LED_PIN, LOW);
  }



  lastStatus = currentStatus;
  delay(100);
}

void send_motion_packet(uint8_t loraVbat) {
  if (lmh_join_status_get() != LMH_SET) {
    Serial.println("Not joined to LoRaWAN network yet.");
    return;
  }

  m_lora_app_data.port = LORAWAN_APP_PORT;
  m_lora_app_data.buffer[0] = 0x01;
  m_lora_app_data.buffer[1] = loraVbat;
  m_lora_app_data.buffsize = 2;

  lmh_error_status result = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  Serial.println(result == LMH_SUCCESS ? "Uplink sent." : "Failed to send uplink.");
}

// LoRaWAN Event Handlers
void lorawan_has_joined_handler(void) {
  Serial.println("Successfully joined LoRaWAN network.");
}

void lorawan_join_failed_handler(void) {
  Serial.println("Join failed! Check credentials and coverage.");
}

void lorawan_rx_handler(lmh_app_data_t *app_data) {
  Serial.printf("Received data on port %d: ", app_data->port);
  for (int i = 0; i < app_data->buffsize; i++) {
    Serial.printf("%02X ", app_data->buffer[i]);
  }
  Serial.println();
}

void lorawan_confirm_class_handler(DeviceClass_t Class) {
  Serial.printf("Switched to Class %c\n", "ABC"[Class]);
}


// VBat real battery voltage
float readVBAT(void)
{
  float raw;

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(vbat_pin);

  // // Set the ADC back to the default settings
  // analogReference(AR_DEFAULT);
  // analogReadResolution(10);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB;
}

// Converts the battery voltage into a percentage value
uint8_t mvToPercent(float mvolts)
{
  if (mvolts < 3000)
    return 0;

  if (mvolts < 4200)
  {
    mvolts -= 3000;
    return mvolts * 100 / (4200 - 3000);
  }
}

// Converts the battery voltage to a value between 0 and 255
uint8_t mvToLoRaWanBattVal(float mvolts)
{ // * 2.55
if (mvolts < 3000)
    return 0;

if (mvolts < 4200)
{
  mvolts -= 3000;
  return mvolts * 255 / (4200 - 3000);
}
return 0;
}