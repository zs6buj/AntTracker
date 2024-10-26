/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2020 All right reserved.

  Functions for the stepper part of MobaTools
*/
#include <MobaTools.h>
#include <utilities/MoToDbg.h>

#ifdef ESP8266 // version only for ESP8266

MoToPwm::MoToPwm() {
    _pinNbr = 255;               // not Attached
    _pwmCycle = DEFAULT_PWMCYC;  // only for analogWrite
    _pwmScale = 1000;
}

uint8_t MoToPwm::attach(uint8_t pinArg ){
    // PWM-Ausgang
    // wrong pinnbr or pin in use?
    if ( pinArg >16 || gpioUsed(pinArg ) ) return 0;
    
    setGpio(pinArg);    // mark pin as used
    _pinNbr = pinArg;
    _pwmCycle = DEFAULT_PWMCYC;  // only for analogWrite
    _pwmScale = 1000;
    pinMode( _pinNbr, OUTPUT );
    return true;
}

void MoToPwm::detach( ){
    // Led-Ausgang mit Softstart. 
    
    // not attached
    if (_pinNbr >16 ) return;
    
    clrGpio(_pinNbr);    // mark pin as unused
    stopWaveformMoTo(_pinNbr);
    pinMode( _pinNbr, INPUT );
    _pinNbr = 255;
    return;
}

void MoToPwm::analogWrite ( uint16_t duty1000 ){
    // create pwm pulse with defined dutycycle 0...1000 ( promille)
    if (_pinNbr >16 ) return;
    if (duty1000 > _pwmScale ) duty1000 = _pwmScale;
    
    uint32_t high = duty1000*_pwmCycle/_pwmScale;
    if (high  == 0) {
        stopWaveformMoTo(_pinNbr);
        digitalWrite(_pinNbr, LOW);
    } else if (high == _pwmCycle) {
        stopWaveformMoTo(_pinNbr);
        digitalWrite(_pinNbr, HIGH);
    } else {
        if ( high<MIN_PWMPULSE ) high = MIN_PWMPULSE;   // min pulse high is 40 µs
        startWaveformMoTo(_pinNbr, high, _pwmCycle-high, 0);
    }
}

void MoToPwm::setFreq(uint32_t freq){
    // set frequency for following analogWrite commands
    if (_pinNbr >16 ) return;
    freq = constrain( freq, 1, 10000 );
    _pwmCycle = 1000000L / freq;
}

void MoToPwm::tone(float freq, uint32_t duration){
    // create tone with dutycycle 50%
    if (_pinNbr >16 ) return;
    freq = constrain( freq, 1, 100000 );
    uint32_t cyc2 = 500000.0 / freq;
    cyc2 = constrain( cyc2, MIN_PWMPULSE, 1000000 );
    startWaveformMoTo(_pinNbr, cyc2, cyc2, duration*1000);
}

void MoToPwm::setPwm( uint32_t high, uint32_t low ){
    // set pem with free defined hig and low times ( in microseconds )
    if (_pinNbr >16 ) return;
    high = constrain( high, MIN_PWMPULSE, 1000000L );
    low  = constrain( low, MIN_PWMPULSE, 1000000L );
    startWaveformMoTo(_pinNbr, high, low, 0);
}

void MoToPwm::stop(){
    // stop creating pulses
    if (_pinNbr >16 ) return;
    stopWaveformMoTo(_pinNbr);
}


#endif // ESP
