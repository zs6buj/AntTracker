// ESP32 HW-specific parts for MobaTools
    //--------------------------------------------------------------------------------------------------------------
#ifndef ESP32_DRIVER_H
#define ESP32_DRIVER_H

#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
	#error This ESP32 processor is not supported
#endif
#include "esp32-hal-timer.h"
#include "esp32-hal-spi.h"
#ifdef __cplusplus
extern "C" {
#endif
//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv  ESP32  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#define IS_32BIT
#define IS_ESP  32
#define HAS_PWM_HW

// ----------------   stepper related defines   ---------------------------------
// use of SPI interface
#ifdef USE_VSPI
    #define SPI_USED    VSPI
    #define MOSI        23
    #define SCK         18
    #define SS          5
#else
    // HSPI is used by default
    #define SPI_USED    HSPI
    #define MOSI        13
    #define SCK         14
    #define SS          15
#endif

struct timerConfig_t {
  union {
    struct {
      uint32_t reserved0:   10;
      uint32_t alarm_en:     1;             /*When set  alarm is enabled*/
      uint32_t level_int_en: 1;             /*When set  level type interrupt will be generated during alarm*/
      uint32_t edge_int_en:  1;             /*When set  edge type interrupt will be generated during alarm*/
      uint32_t divider:     16;             /*Timer clock (T0/1_clk) pre-scale value.*/
      uint32_t autoreload:   1;             /*When set  timer 0/1 auto-reload at alarming is enabled*/
      uint32_t increase:     1;             /*When set  timer 0/1 time-base counter increment. When cleared timer 0 time-base counter decrement.*/
      uint32_t enable:       1;             /*When set  timer 0/1 time-base counter is enabled*/
    };
    uint32_t val;
  };
} ;
extern bool timerInitialized;
#define CYCLETIME       1                   // Cycle count in µs on 32Bit processors
// Prescaler for 64-Bit Timer ( input is 
#define DIVIDER     APB_CLK_FREQ/2/1000000  // 0,5µs Timertic ( 80MHz input freq )
#define TICS_PER_MICROSECOND 2              // bei 0,5 µs Timertic
// Mutexes für Zugriff auf Daten, die in ISR verändert werden
extern portMUX_TYPE stepperMux;

#define STEPPER_TIMER     3  // Timer 1, Group 1
extern hw_timer_t * stepTimer;
void seizeTimer1();

// --------------   defines for servo and softled ( ledc pwm hardware on ESP32 is used ) -----------------------
//if  the following line is commented out, direct register access is used ( interrupts are not disabled during flash access )
//#define LEDC_USE_SDK          // use SDK calls für Servo & Led Pulses ( interrupts are disbled during flash access )
                                // !!!!!!! Die SDK-Aufrufe sind NICHT Interruptfest - die SDK-Version darf daher nicht // // !!!!!!! aktiviert werden, da die Interrupts NICHT abgeschaltet werden 

#ifdef COMPILING_MOTOSERVO_CPP
    //#warning compiling servo.cpp for ESP32
    #undef interrupts
    #undef noInterrupts
    #define interrupts()    portEXIT_CRITICAL(&servoMux);
    #define noInterrupts()  portENTER_CRITICAL(&servoMux);
#endif
#ifdef COMPILING_MOTOSOFTLED_CPP
    //#warning compiling softled.cpp for ESP32
    #undef interrupts
    #undef noInterrupts
    #define interrupts()    portEXIT_CRITICAL(&softledMux);
    #define noInterrupts()  portENTER_CRITICAL(&softledMux);
#endif
#ifdef COMPILING_MOTOSTEPPER_CPP
    //#warning compiling stepper.cpp for ESP32
    #undef interrupts
    #undef noInterrupts
    #define interrupts()    portEXIT_CRITICAL(&stepperMux);
    #define noInterrupts()  portENTER_CRITICAL(&stepperMux);
#endif
typedef struct {
    union {
        struct {
            uint32_t pin     :8;     // used pwm HW ( 0... 15
            uint32_t inUse   :1;     // 0 
            uint32_t group   :1;     // leds group ( 0/1 )
            uint32_t timer   :2;     // Timer used ( 2 for servo, 3 for softled, 0/1 unused by MobaTools
            uint32_t channel :3;     // ledc  channel ( 0..7 )
            uint32_t reserved:18;
        };
        uint32_t value;
    };
} pwmUse_t;
extern pwmUse_t pwmUse[16];


#define SERVO_TIMER         2
#define LED_TIMER           3

int8_t initPwmChannel( uint8_t pin, uint8_t timer );
void IRAM_ATTR setPwmDuty(int8_t pwmNbr, uint32_t duty );
void setPwmPin( uint8_t pwmNbr ) ;
int8_t freePwmNbr( uint8_t pwmNbr );

// On ESP32 timer tics and inc are the same. 
#define INC_PER_TIC  1
#define COMPAT_FACT  1 // no compatibility mode for ESP32                

#define SERVO_FREQ  50          // 20ms period
#define SOFTLED_FREQ    100
#define LEDC_BITS  18          // bitresolution for duty cycle of servos and softleds
								// one timertic is 0.07629 µs for servos
#define SERVO_CYCLE ( 1000000L / SERVO_FREQ ) // Servo cycle in uS
#define SOFTLED_CYCLE ( 1000000L / SOFTLED_FREQ ) // softled cycle in uS
#define DUTY100     ( 1<<(LEDC_BITS) )
// compute pulsewidth ( in usec ) to duty ) for Servos
#define time2tic(pulse) ( ( (pulse) *  DUTY100) / SERVO_CYCLE )  
// compute duty to pulsewidth ( in uS )
#define tic2time(duty)  ( ( (duty) * SERVO_CYCLE) / DUTY100 +1 )
#define AS_Speed2Inc(speed)  (speed*1280/763)  // tic == Inc == 0.07629 µs

// compute pulsewidth ( in usec ) to duty ) for Softleds
// all softled pwmValues are in µs
#define slPwm2tic(pulse) ( ( (pulse) *  DUTY100) / SOFTLED_CYCLE )  
// compute duty to pulsewidth ( in uS )
#define tic2slPwm(duty)  ( ( (duty) * SOFTLED_CYCLE) / DUTY100 )

extern portMUX_TYPE softledMux;
extern portMUX_TYPE servoMux;
extern portMUX_TYPE timerMux;

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ end of ESP32 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#ifdef __cplusplus
}
#endif
#define ARCHITECT_INCLUDE <esp32/MoToESP32.h>
#endif /* ESP32_DRIVER_H */

