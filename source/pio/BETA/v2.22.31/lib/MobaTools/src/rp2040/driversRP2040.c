// ESP32 HW-spcific Functions
#ifdef ARDUINO_ARCH_RP2040
// Logging
#include <Arduino.h>
#include "drivers.h"

// ------------------ SOFTLED & Servos ------------------------------------------------------

//===================================================================================

//=======================================================================================

//#define CONFIG_DISABLE_HAL_LOCKS


//#warning "HW specfic drivers.c (direct reg access) - ESP32 --"
// variant with direct acces to ledc PWM register
// APB_CLK must be 80MHz


// There are max 16 pwm ouputs. These are used by servos and softleds
// THE PWM hardware elements are fixed to the gpios

int8_t freePwmNbr( uint8_t pwmNbr ) {
    return true;
}

int8_t initPwmChannel( uint8_t pin, uint8_t timer ) {
    return 0;
}

void setPwmPin( uint8_t pwmNbr ) {
}
    
void setPwmDuty(int8_t pwmNbr, uint32_t duty ){
}
//======================================================================================================
//=======================================================================================================

#endif // End of file