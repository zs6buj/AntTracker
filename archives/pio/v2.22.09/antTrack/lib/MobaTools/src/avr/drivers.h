#ifndef AVR_DRIVER_H
#define AVR_DRIVER_H
//////////////////////////////////////// processor dependent defines and declarations //////////////////////////////////////////
//vvvvvvvvvvvvvvvvvvvvvvvvvvvvv  AVR ATMega vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#include <avr/interrupt.h>
#define IRAM_ATTR       // delete in .cpp files, because it has no meaning for AVR processors
#define DRAM_ATTR


#define FAST_PORTWRT        // if this is defined, ports are written directly in IRQ-Routines,
                            // not with 'digitalWrite' functions
#define TICS_PER_MICROSECOND (clockCyclesPerMicrosecond() / 8 ) // prescaler is 8 = 0.5us

// check supported AVR Processors
//#if !defined __AVR_MEGA__ && !defined ARDUINO_AVR_ATTINYX4 && !defined ARDUINO_AVR_ATTINYX8
// we need a 16-Bit timer (TCNT1 or TCNT3) and an SPI or USI HW
#if !( ( defined TCNT1H || defined TCNT3H ) && ( defined SPCR || defined USICR ) )
#error "This AVR Processor is not supported"
#endif

// define timer to use
#if defined ( TCNT3 ) && !defined ( NO_TIMER3 )
    // Timer 3 is available, use it
    //#warning "Timer 3 used"
    #define TCNTx       TCNT3
    #define GET_COUNT   TCNT3
    #define TIMERx_COMPB_vect TIMER3_COMPB_vect
    #define TIMERx_COMPA_vect TIMER3_COMPA_vect
    #define OCRxB      OCR3B
    #define OCRxA      OCR3A
    #define TCCRxA     TCCR3A
    #define TCCRxB     TCCR3B
    #define WGMx3      WGM33
    #define WGMx2      WGM32
    #define ICRx       ICR3
    #define OCIExA     OCIE3A
    #define OCIExB     OCIE3B
    #define TIMSKx     TIMSK3
#else
    // Timer 1 benutzen
    #define TCNTx       TCNT1
    #define GET_COUNT   TCNT1
    #define TIMERx_COMPB_vect TIMER1_COMPB_vect
    #define TIMERx_COMPA_vect TIMER1_COMPA_vect
    #define OCRxB      OCR1B
    #define OCRxA      OCR1A
    #define TCCRxA     TCCR1A
    #define TCCRxB     TCCR1B
    #define WGMx3      WGM13
    #define WGMx2      WGM12
    #define ICRx       ICR1
    #define OCIExA     OCIE1A
    #define OCIExB     OCIE1B
    #ifdef TIMSK
        #define TIMSKx     TIMSK
    #else
        #define TIMSKx     TIMSK1
    #endif
#endif    

#define ARCHITECT_INCLUDE <avr/MoToAVR.h>
#endif
