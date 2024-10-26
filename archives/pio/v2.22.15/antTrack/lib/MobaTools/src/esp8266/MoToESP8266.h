#ifndef MOTOESP8266_H
#define MOTOESP8266_H
// ESP8266 specific declarations for Cpp files
//#warning ESP8266 specific cpp includes

static inline __attribute__((__always_inline__)) void _noStepIRQ() {
			noInterrupts();
}

static inline __attribute__((__always_inline__)) void  _stepIRQ() {
			interrupts();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSERVO_CPP //  inline functions and macros für MoToServo.cpp ---------------------------
void ISR_Servo( void *arg );

static inline __attribute__((__always_inline__)) void startServoPulse(servoData_t *servoDataP, uint32_t pulseWidth ) {
    startWaveformMoTo(servoDataP->pin, pulseWidth/TICS_PER_MICROSECOND, TIMERPERIODE-(pulseWidth/TICS_PER_MICROSECOND),0);
}

static inline __attribute__((__always_inline__)) void servoWrite( servoData_t *servoDataP, uint32_t pulseWidth ) {
    // the same funktion exists for ESP32 ( with another internal call )
    startWaveformMoTo(servoDataP->pin, pulseWidth/TICS_PER_MICROSECOND, TIMERPERIODE-(pulseWidth/TICS_PER_MICROSECOND),0);
}

static inline __attribute__((__always_inline__)) void servoPulseOff( servoData_t *servoDataP ) {
    stopWaveformMoTo( servoDataP->pin );
}
#endif // compiling servo
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef COMPILING_MOTOSOFTLEDESP_CPP  //  inline functions and macros für MoToSoftledESP.cpp ---------------------------

static inline __attribute__((__always_inline__)) uint8_t attachSoftledAS( ledData_t *ledDataP ) {
    gpioTab[gpio2ISRx(ledDataP->pin)].MoToISR = (void (*)(void*))ISR_Softled;
    gpioTab[gpio2ISRx(ledDataP->pin)].IsrData = ledDataP;
    attachInterrupt( ledDataP->pin, gpioTab[gpio2ISRx(ledDataP->pin)].gpioISR, ledDataP->invFlg?RISING:FALLING );
    return ledDataP->pin;
}

static inline __attribute__((__always_inline__)) void startLedPulseAS( uint8_t pin, uint8_t invFlg, uint32_t pulseLen ){
    // start or change the pwmpulses on the led pin.
    // with invFlg set pulseLen is lowtime, else hightime
    if ( pulseLen < 50 ) pulseLen =50;
    if ( invFlg ) {
        startWaveformMoTo(pin, PWMCYC-pulseLen, pulseLen,0);
    } else {
        startWaveformMoTo(pin, pulseLen, PWMCYC-pulseLen,0);
    }

}

static inline __attribute__((__always_inline__)) void softLedOffAS(  uint8_t pin, uint8_t invFlg ){
    stopWaveformMoTo( pin );
    digitalWrite( pin , invFlg );
}

static inline __attribute__((__always_inline__)) void softLedOnAS(  uint8_t pin, uint8_t invFlg ){
    // Bei ESP8266 findet die endgültige Einschaltung erst am Ende des Impulszyklus statt
    attachInterrupt( pin, gpioTab[gpio2ISRx(pin)].gpioISR, invFlg?FALLING:RISING ); // leading edge
}

static inline __attribute__((__always_inline__)) void softLedOn2AS(  uint8_t pin, uint8_t invFlg ){
    // Bei ESP8266 findet die endgültige Einschaltung erst hier am Ende des Impulszyklus statt
    stopWaveformMoTo( pin );
    attachInterrupt( pin, gpioTab[gpio2ISRx(pin)].gpioISR, invFlg?RISING:FALLING ); //trailing edge 
}

static inline __attribute__((__always_inline__)) void attachInterruptAS(  ledData_t *ledDataP ){
    attachInterrupt( ledDataP->pin, gpioTab[gpio2ISRx(ledDataP->pin)].gpioISR, ledDataP->invFlg?RISING:FALLING );
}
#endif // compiling softLed
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSTEPPER_CPP //  inline functions and macros für MoToStepper.cpp ---------------------------
static inline __attribute__((__always_inline__)) void seizeTimerAS() {
    // tis is a dummy function for ESP8266
}
/*
static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
    // initialize ISR-Table and attach interrupt to step-Pin
    // assign an ISR to the pin
    gpioTab[gpio2ISRx(_stepperData.pins[0])].MoToISR = (void (*)(void*))ISR_Stepper;
    gpioTab[gpio2ISRx(_stepperData.pins[0])].IsrData = &_stepperData;
    attachInterrupt( _stepperData.pins[0], gpioTab[gpio2ISRx(_stepperData.pins[0])].gpioISR, RISING );
    setGpio(pins[0]);    // mark pin as used
    setGpio(pins[1]);    // mark pin as used
}*/

#define enableStepperIsrAS()  /* must be defined as macro because local variables of MoToStepper.cpp are used */   \
    /* initialize ISR-Table and attach interrupt to step-Pin                            \
      assign an ISR to the pin   */                                                       \
    gpioTab[gpio2ISRx(_stepperData.pins[0])].MoToISR = (void (*)(void*))ISR_Stepper;    \
    gpioTab[gpio2ISRx(_stepperData.pins[0])].IsrData = &_stepperData;                   \
    attachInterrupt( _stepperData.pins[0], gpioTab[gpio2ISRx(_stepperData.pins[0])].gpioISR, RISING ); \
    setGpio(pins[0]);    /* mark pin as used  */                                         \
    setGpio(pins[1]);    /* mark pin as used  */                                        \

#endif //COMPILING_MOTOSTEPPER_CPP 

#endif