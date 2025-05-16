#ifndef RP2040_DRIVER_H
#define RP2040_DRIVER_H
//////////////////////////////////////// processor dependent defines and declarations //////////////////////////////////////////
// MobaTools for  RP2040/RP2350 use the low level functionality of the RPi Pico SDK 
// Only cores that allow direct access to this SDK work with MobaTools (Earle Philhowers core )

//--------------------------------------------------------------------------------------------------------------
//vvvvvvvvvvvvvvvvvvvvvvvvvv RP2040 ( and RP2350) processors vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#define RP2040
#define __RP2040__
#define IS_32BIT
#define HAS_PWM_HW
#define IRAM_ATTR       // delete in .cpp files, because it has no meaning for RP2040 processors
#define DRAM_ATTR
#define MOTOSOFTLED32		// use 32-bit version of SoftLed class
#define stepperISR __not_in_flash_func(stepperISR) // stepperISR must be executed in RAM
                                                  // this is only for defining the function. in .cpp
                                                  // where it is called, it must be undef'd
#define softledISR __not_in_flash_func(softledISR) // softledISR must be executed in RAM
                                                  // this is only for defining the function. in .cpp
                                                  // where it is called, it must be undef'd

#if ( F_CPU != 150000000L )
  #undef STP_TIMR_NBR
  #define STP_TIMR_NBR 0         // is obviousl not RP2350, so only timer 0 present
#endif

#include <pico/stdlib.h>		// RP2040 SDK
#include "hardware/resets.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/spi.h"
#include <hardware/pwm.h>

#define CYCLETIME       1     // Cycle count in µs on 32Bit processors
#define TICS_PER_MICROSECOND 1 // RP2040 timer is clocked with 1MHz
#define WITH_PRINTF           // Core supports printf command
#define MAX_JITTER (MIN_STEP_CYCLE/2)

// Definition of used  Hw und IRQ's ( Timer/PWM/SPI )
// by default MoToStepper uses alarm 4 of timer 1
const uint8_t stepTIMER = 0; 		// RP2040 has only timer 0, maybe set to 1 on RP2350
// constexpr uint8_t stepALARM = 3;
extern uint8_t stepperAlarm;
// constexpr uint8_t stepperIRQ_NUM = TIMER_ALARM_IRQ_NUM(stepTIMER, stepALARM);
extern uint8_t stepperIRQNum; // NVIC-IRQ-number of stepper IRQ

// Fast digitalWrite
// cannot be used, because in stepper IRQ the delay between changing dir pin and the next step is too short 
// for stepper driver when using fast digitalWrite.
//#define digitalWrite(pin,state) {if( state ) gpio_set_mask( 1<<pin ); else gpio_clr_mask( 1<<pin ); }


//extern bool timerInitialized;
void seizeTimer1();
//#define USE_SPI2          // Use SPI1 if not defined
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ RP2040 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#define ARCHITECT_INCLUDE <rp2040/MoToRP2040.h>
#endif