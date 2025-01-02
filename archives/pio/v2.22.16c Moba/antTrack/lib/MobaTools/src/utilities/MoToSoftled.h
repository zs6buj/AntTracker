#ifndef MOTOSOFTLED_H
#define MOTOSOFTLED_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2020 All right reserved.

  Definitions and declarations for the softled part of MobaTools
*/
// defines for soft-leds
#define MAX_LEDS    16     // Soft On/Off defined for compatibility reasons. There is no fixed limit anymore.


///////////////////////////////////////////////////////////////////////////////////////////////
// 
//////////////////////////////////////////////////////////////////////////////////
// global data for softleds ( used in ISR )
// the PWM pulses are created together with stepper pulses
//
// table of pwm-steps for soft on/off in CYCLETIME units ( bulb simulation). The first value means pwm cycletime
#ifdef IS_32BIT
    #define PWMCYC  10000   // Cycletime in µs ( = 100Hz )
    const uint16_t MIN_PULSE = 50;
    const uint16_t MAX_PULSE = PWMCYC-MIN_PULSE;
    // following values determin characteristics of bulb simulation
    const int  stepRef = 5000;
    const int  stepOfs = 1000;
#else
    const uint8_t iSteps[] PROGMEM = { 80, 1, 4, 6,10,13,15,17,19,21,23,25,27,29,31,33,35,36,37,38,39,
                              40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,
                              60,61,62,63,64,65,65,66,66,67,67,68,68,69,69,70,70,71,71,72,
                              72,73,73,74,74,75,75,76,76,77,77,77,78,78,78,78,79,79,79 };

    #define DELTASTEPS 128  // this MUST be a power of 2
    #define LED_IX_MAX    ((int16_t)sizeof(iSteps) -1) // int16_t to suppress warnings when comparing to aCycle
    #define LED_STEP_MAX    (LED_IX_MAX*DELTASTEPS)     // max value of ledData.aStep
    #define LED_CYCLE_MAX   (iSteps[0])
    #define LED_PWMTIME     LED_CYCLE_MAX * CYCLETIME / 1000  // PWM refreshrate in ms
    
    
#endif
                                        
enum LedStats_t:byte { NOTATTACHED, STATE_OFF, STATE_ON, ACTIVE, INCBULB, DECBULB, INCLIN, DECLIN, STOPPING };
                        // values >= ACTIVE means active in ISR routine ( pulses are generated )
                        
typedef struct ledData_t {          // global led values ( used in IRQ )
  #ifndef ESP8266
      struct ledData_t*  nextLedDataP;  // chaining the active Leds
      struct ledData_t** backLedDataPP; // adress of pointer, that points to this led (backwards reference)
      uint8_t   actPulse;               // PWM pulse is HIGH
  #endif
  #ifdef IS_32BIT
      uint16_t aPwm;                    // actual PWM value ( µs )
      uint16_t tPwmOn;                // target PWM value (µs )
      uint16_t tPwmOff;                 // target PWM value (µs )
      uint16_t stepI;                   // actual step during rising/falling   
      uint16_t stepMax;                 // max nbr of steps between ON/OFF  ( determines rising/falling time )
      //computing of bulb-simulation (hyperbolic ramp): 
      // this values must be recomputed if tPwmon, tPwmoff changes
      // formula: pwm = hypPo + hypB/(stepOfs+(stepRef-stepI))
      int hypB;
      int hypPo;
      int8_t pwmNbr;                    // Number of leds HW ( ESP32 ), same as pin otherwise
  #else
      int16_t speed;                    // > 0 : steps per cycle switching on
                                        // < 0 : steps per cycle switching off
                                        // 0: led is inactive (not attached)
      int16_t   aStep;                  // actual step between on/off or off/on ( always counts up )
      int8_t    aCycle;                 // actual cycle ( =length of PWM pule )
  #endif
  LedStats_t state;	                // actual state: steady or incementing/decrementing
    
  volatile uint8_t invFlg;
  #ifdef FAST_PORTWRT
  portBits_t portPin;               // Outputpin as portaddress and bitmask for faster writing
  #else
  uint8_t pin;                      // Outputpins as Arduino numbers
  #endif
} ledData_t;

void ISR_Softled( void *arg );
//////////////////////////////////////////////////////////////////////////////////////////////
class MoToSoftLed
{ // Switch leds on/off softly.
  // 
  public:
    // don't allow copying and moving of SoftLed objects
    MoToSoftLed &operator= (const MoToSoftLed & )   =delete;
    MoToSoftLed &operator= (MoToSoftLed && )        =delete;
    MoToSoftLed (const MoToSoftLed & )              =delete;
    MoToSoftLed (MoToSoftLed && )                   =delete;

    MoToSoftLed();
    uint8_t attach(uint8_t pinArg, uint8_t invArg = false );     // Led-pin with soft on
    void riseTime( uint16_t );       // in millisec - falltime is the same
    void on();                   // 
    void off();                  // 
    void on(uint8_t);           // pwm value for ON ( in % )
    void off(uint8_t);           //  pwmValue for OFF ( in % )
	void write( uint8_t );			// is ON or OFF
    void write( uint8_t time, uint8_t type  ); //whether it is a linear or bulb type
    void toggle( void ); 
  private:
    void _computeBulbValues();    // used only with ESP8266
    void mount( LedStats_t state );
    ledData_t _ledData;
    uint8_t	_setpoint;
    #define OFF 0
    #define ON  1
    
    uint8_t _ledType;        // Type of lamp (linear or bulb)
    #define LINEAR  0
    #define BULB    1
    int16_t _ledSpeed;       // speed with IRQ based softleds
    
};


#endif