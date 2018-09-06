/*******************************************************************************
* LowPower functions adopted from system_set_sleepmode() in power.h in AdafruitASFcore library
* Modified by Joe Miller 9/3/2018 for SOLoRa board
* Added lowPowerPinMode
*******************************************************************************/

#include "LowPower.h"
#include "Arduino.h"

/*******************************************************************************
* Name: lowPowerPinMode
*
* WARNING:      This should be the first initialization call in you code.
*               If called after anything that sets pinmode, this function will undo it.
*
* Description:  Arduino startup code changes all pins from input mode to output mode.
*               This causes increased leakage current. 
*               This function sets all pins to input mode to reduce power.
* 
*******************************************************************************/
void LP_PinMode() {  
    // later i will substitute pin numbers with names to better keep track which are in use
	//int sen[]={0,1,2,3,4,5,6,7,8,9,10,11,12,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};

    for(int i; i<39; i++) {
    pinMode(i, INPUT_PULLUP);
    }

}


/*******************************************************************************
* Name: idle
* Description: Putting SAMD21G18A into idle mode. This is the lowest current 
*              consumption mode. Requires separate handling of clock and 
* 			   peripheral management (disabling and shutting down) to achieve 
* 			   the desired current consumption.
*
* Argument  	Description
* =========  	===========
* 1. idleMode   Idle mode level (0, 1, 2) where IDLE_2 level provide lowest 
* 				current consumption in this mode.
* 
*******************************************************************************/
void LP_idle(uint8_t idleMode)
{
	if (idleMode > 2) idleMode = 2;
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	PM->SLEEP.reg = idleMode;
	__DSB();
	__WFI();
}

/*******************************************************************************
* Name: standby
* Description: Putting SAMD21G18A into standby mode. This is the lowest current 
*              consumption mode. Use this together with the built-in RTC (use 
*              RTCZero library) or external pin interrupt to wake up through 
*              external event triggering.
*
*******************************************************************************/
void LP_Standby()
{
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
}
