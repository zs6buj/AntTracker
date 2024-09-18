// ESP8266 HW-spcific Functions
#ifdef ARDUINO_ARCH_ESP8266
#include <MobaTools.h>
//#define debugTP
#define debugPrint
#include <utilities/MoToDbg.h>

//#warning "HW specfic - ESP8266 ---"

// definition of gpio ISR's ( there is one ISR entrypoint per gpio )
// at max 10 gpio's can be used at an ESP12: gpio 0,1,2,3,4,5,12,13,14,15
// gpio 6-11 is used for flash
// gpio16 has no interruptcapability ( but can be used as dir- or enable-pin for a stepper)
void ICACHE_RAM_ATTR gpioISR0() { gpioTab[0].MoToISR( gpioTab[0].IsrData ); }
void ICACHE_RAM_ATTR gpioISR1() { gpioTab[1].MoToISR( gpioTab[1].IsrData ); }
void ICACHE_RAM_ATTR gpioISR2() { gpioTab[2].MoToISR( gpioTab[2].IsrData ); }
void ICACHE_RAM_ATTR gpioISR3() { gpioTab[3].MoToISR( gpioTab[3].IsrData ); }
void ICACHE_RAM_ATTR gpioISR4() { gpioTab[4].MoToISR( gpioTab[4].IsrData ); }
void ICACHE_RAM_ATTR gpioISR5() { gpioTab[5].MoToISR( gpioTab[5].IsrData ); }
void ICACHE_RAM_ATTR gpioISR12() { gpioTab[6].MoToISR( gpioTab[6].IsrData ); }
void ICACHE_RAM_ATTR gpioISR13() { gpioTab[7].MoToISR( gpioTab[7].IsrData ); }
void ICACHE_RAM_ATTR gpioISR14() { gpioTab[8].MoToISR( gpioTab[8].IsrData ); }
void ICACHE_RAM_ATTR gpioISR15() { gpioTab[9].MoToISR( gpioTab[9].IsrData ); }

gpioISR_t gpioTab[MAX_GPIO] = {
        &gpioISR0,NULL,NULL
        ,&gpioISR1,NULL,NULL
        ,&gpioISR2,NULL,NULL
        ,&gpioISR3,NULL,NULL
        ,&gpioISR4,NULL,NULL
        ,&gpioISR5,NULL,NULL
        ,&gpioISR12,NULL,NULL
        ,&gpioISR13,NULL,NULL
        ,&gpioISR14,NULL,NULL
        ,&gpioISR15,NULL,NULL
       };

static const uint32_t mask6_11 = 0b00000111111000000;
static uint32_t gpioUsedMask = mask6_11; // gpio 6..11 are always 'in use'
bool gpioUsed( unsigned int gpio ) {
    DB_PRINT( "GpioUsed=%05X, RetVal( %d ) =  %05X ", gpioUsedMask, gpio,  ( gpioUsedMask & ( 1<<gpio )) )
    return ( gpioUsedMask & ( 1<<gpio ) ) ;
}
void setGpio( unsigned int gpio ) {
    gpioUsedMask |= ( 1<<gpio );
    DB_PRINT( "new gpio(%d), Mask=%05X", gpio, gpioUsedMask );
}
void clrGpio( unsigned int gpio ) {
    gpioUsedMask &= ~( 1<<gpio );
    gpioUsedMask |= mask6_11;   // gpio 6...11 must never be cleared
}



#endif // ARDUINO_ARCH_ESP8266
