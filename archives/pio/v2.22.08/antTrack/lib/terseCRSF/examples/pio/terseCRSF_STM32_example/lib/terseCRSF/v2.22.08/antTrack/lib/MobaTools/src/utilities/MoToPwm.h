#ifndef MOTOPWM_H
#define MOTOPWM_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2020 All right reserved.

  Definitions and declarations for the pwm part of MobaTools ( only for ESP8266 )
*/
#ifdef ESP8266

///////////////////////////////////////////////////////////////////////////////////////////////
    const uint16_t DEFAULT_PWMCYC = 1000;   // default analogWrite cycletime in µs ( = 1000Hz )
    const uint16_t MIN_PWMPULSE = 40;
    
class MoToPwm
{ // create pwm pulses
  // 
  public:
    MoToPwm();
    uint8_t attach( uint8_t pin );                 // set Gpio to create pulses on
    void detach();
    void analogWrite ( uint16_t duty1000 );     // create pwm pulse with defined dutycycle 0...1000 ( promille)
    void setFreq(uint32_t freq);  // set frequency for following analogWrite commands
    void tone(float freq, uint32_t duration );  // create tone with dutycycle 50%
    void setPwm( uint32_t high, uint32_t low );         // set pem with free defined hig and low times ( in microseconds )
    #define noTone stop
    void stop() ;                               // stop creating pulses
  private:
    uint8_t     _pinNbr;                        // 255 means not attached
    uint32_t    _pwmCycle;                      // cycletime for analogWrite command ( in µs )
    uint32_t    _pwmScale;
    
};
#endif

#endif