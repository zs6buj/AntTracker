#ifndef MOTOBASE_H
#define MOTOBASE_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2020 All right reserved.

  Base definitions and declarations for all parts of MobaTools
*/
#include <inttypes.h>
#include <Arduino.h>

//architecture specific includes
#ifdef ARDUINO_ARCH_AVR
	#include <avr/drivers.h>
#elif defined ARDUINO_ARCH_MEGAAVR
	#include <megaAVR/drivers.h>
#elif defined ARDUINO_ARCH_STM32F1
	#include <stm32f1/drivers.h>
#elif defined ARDUINO_ARCH_STM32F4
	#include <stm32f4/drivers.h>
#elif defined ARDUINO_ARCH_ESP8266
	#include <esp8266/drivers.h>
#elif defined ARDUINO_ARCH_ESP32
	#include <esp32/drivers.h>
#elif defined ARDUINO_ARCH_RENESAS_UNO 
	#include <ra4m1/drivers.h>
#elif defined ARDUINO_ARCH_RP2040
	#include <rp2040/drivers.h>
#else
    #error "Processor not supported"
#endif


#ifndef IS_ESP
    #define  IRAM_ATTR  // Attribut IRAM_ATTR entfernen wenn nicht definiert
#else
	#ifdef IRAM_ATTR  //IRAM_ATTR
		// don't use ICACHE_RAM_ATTR in newer versions of ESP8266 core
		#undef  ICACHE_RAM_ATTR
		#define ICACHE_RAM_ATTR IRAM_ATTR
	#endif
#endif
////////////////////////////////////////////////// general defines for all plattforms  //////////////////////////////////////////
// Calling compatible architecture specific functions.
// These functions are defined in architecture specific files, but can be called independend from actual architecture

// type definitions ( if they are different for 8bit and 32bit platforms)
#ifdef IS_32BIT
	#define uintxx_t uint32_t
	#define intxx_t	int32_t
	#define uintx8_t uint32_t
	#define intx8_t	int32_t
    #define nextCycle_t int32_t
#else
	#define uintxx_t	uint16_t
	#define  intxx_t	int16_t
	#define uintx8_t uint8_t
	#define intx8_t	int8_t
    #define nextCycle_t uint8_t
    //extern uint8_t nextCycle;
#endif
extern nextCycle_t nextCycle;   // to be used in ISR for stepper and softled

#define ISR_IDLETIME    5000        // max time between two Stepper/Softled ISRs ( µsec )

// old Class names ( for compatibility with former sketches )
#define Stepper4    MoToStepper
#define Servo8      MoToServo  
#define SoftLed     MoToSoftLed
#define EggTimer    MoToTimer

// defines for the stepper motor
/*#define HALFSTEP    1
#define FULLSTEP    2
#define A4988       3   // using motordriver A4988
#define FULL2Wire   4   // not yet used
#define NOSTEP      0   // invalid-flag
*/


// internal defines

#define TIMERPERIODE    20000   // Timer Overflow in µs
//#define TIMER_OVL_TICS  ( TIMERPERIODE*TICS_PER_MICROSECOND )
constexpr uint16_t TIMER_OVL_TICS = ( TIMERPERIODE*TICS_PER_MICROSECOND );


typedef struct {    // portaddress and bitmask for direkt pin set/reset ( only used in AVR )
   volatile uint8_t* Adr;
   volatile uint8_t Mask;
} portBits_t;


#endif



