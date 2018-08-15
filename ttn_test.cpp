/*
 * ttn_test.cpp - Handles the radio
 *
 * The lmic/config.h is by default configured for the RFM95 radio
 * and the US915 frequency plan. For the EU868 frequency plan you need
 * to update that file and the init_lora function below.
 *
 * Important: see "Pin mapping" below
 *
 */

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "ttn_secrets.h"
#include "ttn_test.h"
#include "sensors.h"


// Port used for the message
// (can be used in the 'Port filter' of the TTN Mapper integration)
const u1_t port = 2;

// Send packet interval (in seconds) -- respect duty cycle!
const uint8_t send_packet_interval = 15;

// LoRaWan keys
static const u1_t app_eui[8]  = SECRET_APP_EUI;
static const u1_t dev_eui[8]  = SECRET_DEV_EUI;
static const u1_t app_key[16] = SECRET_APP_KEY;

// transmission counter
static uint8_t packetCount;

extern uint16_t update_LED_interval_ms;
extern uint8_t SOLoRaConfig;
extern int batteryPin;

// Getters for LMIC
void os_getArtEui (u1_t* buf)
{
  memcpy(buf, app_eui, 8);
}

void os_getDevEui (u1_t* buf)
{
  memcpy(buf, dev_eui, 8);
}

void os_getDevKey (u1_t* buf)
{
  memcpy(buf, app_key, 16);
}

// Pin mapping
// The SOLoRa does not map RFM95 DIO1 to an M0 port.
// LMIC needs this signal in LoRa mode, so you need to bridge IO1 to an
// available port -- I have bridged it to Digital pin #11
// We do not need DIO2 for LoRa communication.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 11, LMIC_UNUSED_PIN},
};

// Init job -- Actual message loop will be initiated when join completes
osjob_t init_lora_job;
void init_lora (osjob_t* j)
{
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Allow 1% error margin on clock
  // Note: this might not be necessary, never had clock problem...
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);


// ********* additional US915 requirements *************
  // TTN uses the second sub-band (1 for SFB2), which are channels 8-15
  LMIC_selectSubBand (1);
  
  //Disable channels 66-72
//   for (int i = 66; i <= 72; i++) 
//   {
//   LMIC_disableChannel(i);
//   }
  
  // Disable Adaptive Datarate
  LMIC_setAdrMode(0);
  
  // Downlink Data Rate in the RX2 window
  LMIC.dn2Dr = DR_SF7;
  
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF10,23);
// ************ end US915 requirements *************


  // start joining
  LMIC_startJoining();
}

void init_readBatteryVoltage(void){
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;  // 1.0V voltage reference
  analogReadResolution(12); // change the resolution to 12 bits
}
int readBatteryVoltage( void )  // this returns milliVolts
{
  #define R5  220   // Resistor value in K-Ohms
  #define RDIV (100.0F / (R5 + 100.0F))
  float _battery = analogRead(batteryPin);
  _battery /= RDIV; // multiply back 1/RDIV (resistor divider)
  _battery *= 1.0;  // Multiply by 1.0V, our reference voltage
  _battery /= 4.096; // convert to millivolts /(4096maxADCcounts/1000mV)
  return (int)_battery;
}

// Send job
static osjob_t send_packet_job;
static void send_packet(osjob_t* j)
{
  static uint8_t payload[32];
  uint8_t idx = 0;
  int batteryVoltage = readBatteryVoltage();
  
  // Check if there is not a current TX/RX job running
  if (!(LMIC.opmode & OP_TXRXPEND)) {

    payload[idx++] = packetCount++;
    payload[idx++] = SOLoRaConfig;

#ifdef TEMP
    float temperature = getTemp();
    // preserve sign and 0.125 degree resolution while packing temperature into 2 bytes 
    payload[idx++] = (uint8_t)((uint16_t)((temperature + 100) * 8)>>8); //msb first
    payload[idx++] = (uint8_t)((temperature + 100) * 8); // LSB
    // inverse must be repeated on TTN application side to extract a signed 0.1 degree
#endif

#ifdef ACCEL
    int8_t accel[3];
    getAccel(accel);
    // transfer signed accel value
    payload[idx++] = accel[0] ;   
    payload[idx++] = accel[1] ;   
    payload[idx++] = accel[2];    
#endif

   // battery Voltage in mV
   payload[idx++] = batteryVoltage >> 8;    // MSB
   payload[idx++] = batteryVoltage;    // LSB

      LMIC_setTxData2(port, payload, idx, 0);
  }

  // Reschedule
  os_setTimedCallback(&send_packet_job,
                      os_getTime()+sec2osticks(send_packet_interval),
                      send_packet);
                       
}

// LoRa event handler
// We look at more events than needed, to track potential issues
void onEvent (ev_t ev) {

    switch(ev) {
        case EV_SCAN_TIMEOUT:
            DP("Scan Timeout");
            break;
        case EV_BEACON_FOUND:
            DP("Beacon Found"); 
            break;
        case EV_BEACON_MISSED:
            DP("Beacon Missed"); 
            break;
        case EV_BEACON_TRACKED:
            DP("Beacon Tracked"); 
            break;
        case EV_JOINING:       
            DP("Joining"); 
            break;
        case EV_JOINED:
            DP("Joined"); 
            update_LED_interval_ms = 250; // speedup LED blink rate
            
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            
            // Start sending packets
            os_setCallback(&send_packet_job, send_packet);
            break;
        case EV_RFU1:
            DP("RFU1"); 
            break;
        case EV_JOIN_FAILED:
            DP("Join Failed"); 
            // Re-try!
            os_setCallback(&init_lora_job, init_lora);   
            break;
        case EV_REJOIN_FAILED:
            DP("Rejoin Failed");  
            break;
        case EV_TXCOMPLETE:
            DP("TX Complete"); 
            if (LMIC.txrxFlags & TXRX_ACK) {
              DP(" - Received ack"); 
            }

 digitalWrite(LED_BUILTIN, LOW);   // turn off the LED
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __DSB();
  __WFI();


            // We could re-schedule from here, but it would break the loop if a
            // TX never completes...
            // os_setTimedCallback(&send_packet_job,
            //                     os_getTime()+sec2osticks(send_packet_interval),
            //                     send_packet);
            break;
        case EV_LOST_TSYNC:
            DP("Lost Sync"); 
            break;
        case EV_RESET:
            DP("Reset"); 
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            DP("RX Complete"); 
            break;
        case EV_LINK_DEAD:
            DP("Link Dead"); 
            break;
        case EV_LINK_ALIVE:
            DP("Link Alive"); 
            break;
         default:
            DP("Unknown event"); 
            break;
    }
}

