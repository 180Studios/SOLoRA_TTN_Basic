/*******************************************************************************
 * SOLoRa ttn test  J. Miller 7/16/18
 *
 * Send TTN packets with a packet counter and temperature and/or accel value if applicable
 *
 * This code runs on the SOLoRa node board github/180Studios/SoLoRa
 * Slow LED blink while joining, blink rate increases upon successfull join
 * 
 * Be sure to update the build configuration with the on-board sensors you installed
 * ...or did not install by updating ttn_test.h
 * 
 * Jumper from DOI1 (JP1.1) to D11 (JP2.6) required for V1.0 SOLoRa board
 *
 *******************************************************************************/

#include <lmic.h>
#include <Wire.h>
#include "ttn_test.h"  // setup compile configuration here condition
#include "sensors.h"

int batteryPin = A7; // V_BATT voltage divider connected to D9/A7

// Init radio
extern osjob_t init_lora_job;
extern void init_lora (osjob_t* j);

// Update the LED every seccond
uint16_t update_LED_interval_ms = 1000; // 1 second toggle while waiting to join network
static osjob_t update_LED_job;
static void update_LED(osjob_t* j)
{
  // Update LED
  if (digitalRead(LED_BUILTIN)){
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED OFF
  }
  else {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED ON
  }
 
  // Reschedule
  os_setTimedCallback(&update_LED_job,
                      os_getTime() + ms2osticks(update_LED_interval_ms),
                      update_LED);
}


uint8_t SOLoRaConfig = 0;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1);    // 2 x blinks to confirm startup
  digitalWrite(LED_BUILTIN, HIGH);delay(200);digitalWrite(LED_BUILTIN, LOW);delay(200);
  digitalWrite(LED_BUILTIN, HIGH);delay(200);digitalWrite(LED_BUILTIN, LOW);delay(200);
  digitalWrite(LED_BUILTIN, LOW);   // turn off the LED

  
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Serial Port Connected");
#endif


#ifdef TEMP
  SOLoRaConfig += 1;
  #define I2C_REQUIRED
#endif

#ifdef ACCEL
  SOLoRaConfig += 2;
  #ifndef I2C_REQUIRED
  #define I2C_REQUIRED
  #endif
#endif

#ifdef I2C_REQUIRED
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(9600);          // start serial communication at 9600bps
#endif

  if(SOLoRaConfig & 0x01) { 
    initTemp();
  }
  if(SOLoRaConfig > 1) {
    initAccel();
  }

  // initialize ADC for Battery 
  init_readBatteryVoltage();
  
  // initialize the scheduler
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Schedule jobs
  // LED updates
  os_setCallback(&update_LED_job, update_LED);
  // Initialize radio
  os_setCallback(&init_lora_job, init_lora);

    // 3xblinks to confirm initialization complete
  digitalWrite(LED_BUILTIN, HIGH);delay(200);digitalWrite(LED_BUILTIN, LOW);delay(200);
  digitalWrite(LED_BUILTIN, HIGH);delay(200);digitalWrite(LED_BUILTIN, LOW);delay(200);
  digitalWrite(LED_BUILTIN, HIGH);delay(200);digitalWrite(LED_BUILTIN, LOW);delay(200);


}

void loop() {

  // Run the scheduler
  os_runloop_once();
}
