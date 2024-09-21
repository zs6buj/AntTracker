#ifndef MEGAAVR_DRIVER_H
#define MEGAAVR_DRIVER_H
//////////////////////////////////////// processor dependent defines and declarations //////////////////////////////////////////
//vvvvvvvvvvvvvvvvvvvvvvvvvvvvv  megaAVR ATMega4809 (Nano Every) vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
/* MobaTools uses Timer/Counter 0 (TCA0) on NanoEvery. Becuase the prescaler of TCA0 is also used for millis() on Timer TCB3,
   it must not be changed. This leads to 4µs per tic. This is no problem for stepper and Softleds, but the resolution of servos is decreased compared to other AVR processors.
   This would be better with megaCoreX boardmanager ( it doesn't use TCA0 prescaler for millis() ), but this boardmanager doesn't support the standard bootloader of Nano Every.
*/   
//#include <avr/interrupt.h>
#define IRAM_ATTR       // delete in .cpp files, because it has no meaning for megaAVR processors
#define DRAM_ATTR


#define FAST_PORTWRT        // if this is defined, ports are written directly in IRQ-Routines,
                            // not with 'digitalWrite' functions
//#define TICS_PER_MICROSECOND (clockCyclesPerMicrosecond() / 64.0) // prescaler is 64 = 4µs per tic (used by millis() too )
#define TICS_PER_MICROSECOND (F_CPU / 1000000.0 / 64) // the clockCyclesPerMicrosecond() call inhibits compiletime evaluation

// define timer to use
// defines specially für Nano Every ( MegaAVR4808 ).
// Timer TCA0 is used, prescaler must be fixed at 64, because it is used by millis() too
// CMP0: for servos
// CMP1: for stepper and softleds
// CMP2: possible own ISR for softleds? ( not yet implemented )

#define OCRxA   TCA0.SINGLE.CMP0
#define OCRxB   TCA0.SINGLE.CMP1
#define GET_COUNT   TCA0.SINGLE.CNT
#define TIMERx_COMPA_vect   TCA0_CMP0_vect
#define TIMERx_COMPB_vect   TCA0_CMP1_vect
#define TIMERx_COMPC_vect   TCA0_CMP2_vect
#define TIMSKx     TCA0_SINGLE_INTCTRL
#define OCIExB	


#define ARCHITECT_INCLUDE <megaAVR/MoToMegaAVR.h>
#endif
