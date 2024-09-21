#ifndef STM32F1_DRIVER_H
#define STM32F1_DRIVER_H
//////////////////////////////////////// processor dependent defines and declarations //////////////////////////////////////////
    //--------------------------------------------------------------------------------------------------------------
//vvvvvvvvvvvvvvvvvvvvvvvvvv STM32F1 processors vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#define STM32F1
#define __STM32Fx__
#define IS_32BIT
#define IRAM_ATTR       // delete in .cpp files, because it has no meaning for STM32 processors
#define DRAM_ATTR
#define MOTOSOFTLED32		// use 32-bit version of SoftLed class

#include <libmaple/timer.h>
#include <libmaple/spi.h>
#include <libmaple/nvic.h>

#define CYCLETIME       1     // Cycle count in µs on 32Bit processors

#define TICS_PER_MICROSECOND (CYCLES_PER_MICROSECOND / 36 ) // prescaler is 36 = 0.5us
//#define TICS_PER_MICROSECOND 2 // prescaler is 36 = 0.5us

#define MT_TIMER TIMER4     // Timer used by MobaTools
#define STEP_CHN    1       // OCR channel for Stepper and Leds
#define TIMER_STEPCH_IRQ TIMER_CC1_INTERRUPT
#define SERVO_CHN   2       // OCR channel for Servos
#define TIMER_SERVOCH_IRQ TIMER_CC2_INTERRUPT
#define GET_COUNT timer_get_count(MT_TIMER)

extern bool timerInitialized;
void seizeTimer1();
//#define USE_SPI2          // Use SPI1 if not defined
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ STM32F1 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#define ARCHITECT_INCLUDE <stm32f1/MoToSTM32F1.h>
#endif
