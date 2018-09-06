#ifndef LowPower_h
#define LowPower_h

#include "Arduino.h"

// enum uint8_t
// {
// 	IDLE_0,
// 	IDLE_1,
// 	IDLE_2
// };

extern void LP_PinMode();
extern void	LP_idle(uint8_t idleMode);
extern void	LP_Standby();

#endif
