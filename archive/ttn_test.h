// ttn_test.h

// if DEBUG is defined, this sketch will not run without a terminal open on the USB serial port
//#define DEBUG  

// Hardware Configuration Switches. Uncomment that you wish to test (and is installed on your SOLoRa board)

//uncomment if on-board MMA accelerometer is installed (did you remember I2C pull-ups?)
//#define ACCEL 

// uncomment if on-board LM75 temperature sensor is installed
//#define TEMP


#ifdef DEBUG
 #define DP(x)  Serial.println (x)
#else
 #define DP(x)
#endif

void init_readBatteryVoltage(void);

