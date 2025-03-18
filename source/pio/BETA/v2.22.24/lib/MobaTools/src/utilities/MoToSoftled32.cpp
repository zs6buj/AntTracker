/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2020 All right reserved.

  Functions for the stepper part of MobaTools
*/
#define COMPILING_MOTOSOFTLED32_CPP

//#define debugPrint
//#define debugTP
#include <MobaTools.h>
#ifdef MOTOSOFTLED32 // version for 32bit controllers ( except ESP8266/32 )

// Global Data for all instances and classes  --------------------------------

// variables for softLeds
// On 32bit controllers all cycle times are measured in µs ( 1 cycle = 1µs )
static ledData_t* ledRootP = NULL; //start of _ledData-chain
static int32_t ledNextCyc = 1;     // next Cycle that is relevant for leds
static int32_t ledCycleCnt = 0;    // count IRQ cycles within PWM cycle

static ledData_t*  ledDataP;              // pointer to active Led in ISR

void softledISR(uint32_t cyclesLastIRQ) { // uint32 for 32-Bit processors
    // ---------------------- softleds -----------------------------------------------
    SET_TP4;
    ledCycleCnt += cyclesLastIRQ;
    if ( ledCycleCnt >= ledNextCyc ) {
        // this IRQ is relevant for softleds
        ledNextCyc = PWMCYC; // there must be atleast one IRQ per PWM Cycle
        if ( ledCycleCnt >= PWMCYC ) {
            // start of a new PWM Cycle - switch all leds with rising/falling or active state to on
            // When rising, check if full on is reached
            ledCycleCnt = 0;
            for ( ledDataP=ledRootP; ledDataP!=NULL; ledDataP = ledDataP->nextLedDataP ) {
                //SET_TP1;
                // loop over led-objects
                ledDataP->actPulse = true;  // start a new pulse
                int changePulse = BULB; // change LINEAR or BULB ( -1: don't change )
                if (ledDataP->invFlg  ) {
                    digitalWrite( ledDataP->pin, LOW );
                } else { 
                    digitalWrite( ledDataP->pin, HIGH );
                }
                switch ( ledDataP->state ) {
                  case INCLIN:
                    changePulse = LINEAR;
					[[fallthrough]];    // supress warning
                  case INCBULB:
                    // led with rising brightness
                    // check if led on is reached
                    if ( ++ledDataP->stepI >= ledDataP->stepMax ) {
                        // full on is reached
                        changePulse = -1; // nothing to change
                        ledDataP->stepI = ledDataP->stepMax;
                        if ( ledDataP->tPwmOn == PWMCYC ) {
                            // switch on static after pulse end (leading edge)
                            SET_TP4;
                            ledDataP->state = STATE_ON;
                            *ledDataP->backLedDataPP = ledDataP->nextLedDataP;
                            if ( ledDataP->nextLedDataP ) ledDataP->nextLedDataP->backLedDataPP = ledDataP->backLedDataPP;
                            ledDataP->aPwm = 0;
                            CLR_TP4;
                        } else {
                            // pwm constant with tPwmOn
                            ledDataP->aPwm = ledDataP->tPwmOn;
                            ledDataP->state = ACTIVE;
                        }
                    }
                    break;
                  case DECLIN:
                    changePulse = LINEAR;
					[[fallthrough]];    // supress warning
                  case DECBULB:
                    // led with falling brightness
                    // check if led off is reached
                    if ( ledDataP->stepI > 0 ) --ledDataP->stepI;
                    if ( ledDataP->stepI == 0 ) {
                        // full off is reached
                         changePulse = -1; // nothing to change
                       if ( ledDataP->tPwmOff == 0 ) {
                            // led is off -> remove from chain
                            ledDataP->state = STATE_OFF;
                            *ledDataP->backLedDataPP = ledDataP->nextLedDataP;
                            if ( ledDataP->nextLedDataP ) ledDataP->nextLedDataP->backLedDataPP = ledDataP->backLedDataPP;
                            digitalWrite( ledDataP->pin , ledDataP->invFlg );
                            ledDataP->state = STATE_OFF;
                        } else {
                            // pwm constant with tPwmOff
                            ledDataP->aPwm = ledDataP->tPwmOff;
                            ledDataP->state = ACTIVE;
                        }
                    }
                    break;
                  case ACTIVE:
                    // led with constant PWM-brightness
                    changePulse = -1; // nothing to change
					break;
                  default: ;
                } // end of 'switch'
                // set time for next IRQ to min of all aPwm
                if ( changePulse >= 0 ) {
                    // inc- / decreasing, set pwm according to actual step
                    uint16_t pOff = max( MIN_PULSE, ledDataP->tPwmOff );
                    uint16_t pOn = min ( MAX_PULSE, ledDataP->tPwmOn );
                    if ( changePulse == LINEAR ) {
                        ledDataP->aPwm = pOff + ( (pOn - pOff) * ledDataP->stepI / ledDataP->stepMax );
                    } else {
                        ledDataP->aPwm = ledDataP->hypPo + ledDataP->hypB/(stepOfs+stepRef - (stepRef * ledDataP->stepI / ledDataP->stepMax) );
                    }
                }  
                if ( ledDataP->aPwm > 0 && ledNextCyc > ledDataP->aPwm ) ledNextCyc = ledDataP->aPwm; 
                //CLR_TP1;
            } // end of led loop
        } else { // is switchofftime within PWM cycle
            //SET_TP3;
            for ( ledDataP=ledRootP; ledDataP!=NULL; ledDataP = ledDataP->nextLedDataP ) {
                //SET_TP4;
                if ( ledDataP->actPulse ) {
                    // led is within PWM cycle with output high
                    if ( ledDataP->aPwm <= ledCycleCnt ) {
                        // End of ON-time is reached
                        digitalWrite( ledDataP->pin , ledDataP->invFlg );
                        ledDataP->actPulse = false; // Led pulse is LOW now
                    } else { 
                       // End of ON-time not yet reached, compute next necessary step
                       ledNextCyc = min( ledDataP->aPwm, ledNextCyc);
                    }
                }
                //CLR_TP4;
            }
            //CLR_TP3;
        }
        //CLR_TP3;
     } // end of softleds 
    //CLR_TP3;
    nextCycle = min( nextCycle, ( ledNextCyc-ledCycleCnt ) );
    //SET_TP3;
    CLR_TP4;
} //=============================== End of softledISR ========================================
/////////////////////////////////////////////////////////////////////////////
//Class MoToSoftLed - for Led with soft on / soft off ---------------------------
// Version with Software PWM

void MoToSoftLed::_computeBulbValues() {
     // recompute parameter for bulb simulation ( hyperbolic approximation of pwm ramp )
    int hypB;
    int hypPo;
    //hypB = ( _ledData.tPwmOn - _ledData.tPwmOff )*_ledData.stepOfs*(_ledData.hypF*_ledData.stepRef+_ledData.stepOfs)/(_ledData.hypF*_ledData.stepRef);
    //hypPo = _ledData.tPwmOn - hypB/_ledData.stepOfs;
    hypB = (stepOfs*(stepRef+stepOfs)/(stepRef)) * ( _ledData.tPwmOn - _ledData.tPwmOff );
    //hypPo = _ledData.tPwmOn - hypB/stepOfs;
    hypPo = _ledData.tPwmOff - hypB/(stepOfs+stepRef);
    noInterrupts();
    _ledData.hypPo = hypPo;
    _ledData.hypB = hypB;
    interrupts();
    #if 0 //def debugPrint
    // print pwmval for step= 0, step=max/2, step=max
    int pwmOff = hypPo + hypB/(stepOfs+(stepRef - 0 ));
    int pwm2 = hypPo + hypB/(stepOfs+(stepRef/2 ));
    int pwmOn = hypPo + hypB/(stepOfs);
    DB_PRINT("Hyperbolic ramp ( pwmOff=%d, pwmOn=%d, stepRef=%d  .. B=%d, Po=%d", _ledData.tPwmOff,_ledData.tPwmOn,stepRef, hypB, hypPo);
    DB_PRINT("Pwm-0=%d,  Pwm1/2=%d, PwmMax=%d", pwmOff, pwm2, pwmOn);
    for( byte ix=0; ix<=20; ix++ ) {
        SET_TP2;CLR_TP2;SET_TP2;
        int step = stepRef * ix / 20;
        int pwm = hypPo + hypB/(stepOfs+stepRef - step );
        CLR_TP2;
        DB_PRINT(" Step %5d - pwm = %6d", step, pwm );
    }
    #endif
}

void MoToSoftLed::mount( LedStats_t stateVal ) {
    // mount softLed to ISR chain ( if not already in )
    // new active Softleds are always inserted at the beginning of the chain
    // only leds in the ISR chain are processed in ISR
    noInterrupts();
    //SET_TP2;
    // check if it's not already active (mounted)
    // Leds must not be mounted twice!
    if ( _ledData.state < ACTIVE ) {
        // write backward reference into the existing first entry 
        // only if the chain is not empty
        if ( ledRootP ) ledRootP->backLedDataPP = &_ledData.nextLedDataP;
        //CLR_TP2;
        _ledData.nextLedDataP = ledRootP;
        ledRootP = &_ledData;
        _ledData.backLedDataPP = &ledRootP;
        //SET_TP2;
    }
    _ledData.state = stateVal;
    //CLR_TP2;
    interrupts();
}   
    

MoToSoftLed::MoToSoftLed() {
    _ledData.aPwm     = 0 ;          // initialize to OFF
    _ledData.tPwmOn = PWMCYC;      // target PWM value (µs )
    _ledData.tPwmOff  = 0;           // target PWM value (µs )
    _ledData.stepI    = 0;           // start of rising
    _ledData.stepMax  = LED_DEFAULT_RISETIME*1000/PWMCYC; // total steps for rising/falling ramp
    _ledData.state    = NOTATTACHED; // initialize 
    _setpoint = OFF ;                // initialize to off
    _ledType = LINEAR;
    _ledData.invFlg = false;
}

   

uint8_t MoToSoftLed::attach(uint8_t pinArg, uint8_t invArg ){
    // Led-Ausgang mit Softstart. 
    
    _ledData.invFlg  = invArg;
    pinMode( pinArg, OUTPUT );
    DB_PRINT( "Led attached, ledIx = 0x%08lx, Pin=%d", (uint32_t)this, pinArg );
    _ledData.state   = STATE_OFF ;   // initialize 
    riseTime( LED_DEFAULT_RISETIME );
    _ledData.stepMax  = LED_DEFAULT_RISETIME*1000/PWMCYC; // total steps for rising/falling ramp
    if ( _ledData.invFlg ) { 
        digitalWrite( pinArg, HIGH );
    } else {
        digitalWrite( pinArg, LOW );
    }
    _ledData.pin=pinArg ;      // Pin-Nbr 
    _computeBulbValues();
    
    seizeTimerAS();
    enableSoftLedIsrAS();
     //DB_PRINT("IX_MAX=%d, CYCLE_MAX=%d, PWMTIME=%d", LED_IX_MAX, LED_CYCLE_MAX, LED_PWMTIME );
    return true;
}

void MoToSoftLed::on(uint8_t brightness ){
    // set brightness for on ( in percent ) and switch on
    // this brightness will stay for all succeding 'on'
    if ( _ledData.state ==  NOTATTACHED ) return;  // this is not a valid instance
    uint16_t tmp;
    if ( brightness > 100 ) brightness = 100;
    tmp = PWMCYC * brightness / 100 ;
    if ( tmp <= _ledData.tPwmOff ) {
        // must be higher than value for 'off'
        _ledData.tPwmOn =_ledData.tPwmOff + MIN_PULSE;
    } else {
        _ledData.tPwmOn = tmp;
    }
    _computeBulbValues();
    on();
    DB_PRINT("On: Br=%d, PwmOn=%d ( %d ), PwmOff=%d", brightness, _ledData.tPwmOn, tmp, _ledData.tPwmOff);
}

void MoToSoftLed::off(uint8_t brightness ){
    // set brightness for off ( in percent ) and switch off
    // this brightness will stay for all succeding 'off'
    if ( _ledData.state ==  NOTATTACHED ) return;  // this is not a valid instance
    uint16_t tmp;
    if ( brightness > 100 ) brightness = 100;
    tmp = PWMCYC * brightness / 100 ;
    if ( tmp >= _ledData.tPwmOn ) {
        // must be lower than value for 'on'
        _ledData.tPwmOff =_ledData.tPwmOn - MIN_PULSE;
    } else {
        _ledData.tPwmOff = tmp;
    }
    _computeBulbValues();
    off();
}

void MoToSoftLed::on(){
    LedStats_t stateT;
    if ( _ledData.state ==  NOTATTACHED ) return;  // this is not a valid instance
    // Don't do anything if its already ON 
    if ( _setpoint != ON  ) {
        _setpoint        = ON ;
        if ( _ledType == LINEAR ) {
            stateT          = INCLIN;
        } else { // is bulb simulation
            stateT          = INCBULB;
        }
        mount( stateT ); // mount into chain if not already mounted
    }
    //DB_PRINT( "Led %08lx On, stepI=%d, state=%d", (uint32_t)this, _ledData.stepI, _ledData.state);
}

void MoToSoftLed::off(){
    LedStats_t stateT;
    if ( _ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    // Dont do anything if its already OFF 
    if ( _setpoint != OFF ) {
        //SET_TP3;
        _setpoint            = OFF;
        if ( _ledType == LINEAR ) {
            stateT          = DECLIN;
        } else { // is bulb simulation
            stateT          = DECBULB;
        }
        mount( stateT ); // mount into chain if not already mounted
        //CLR_TP3;
    }
    //DB_PRINT( "Led %08lx On, stepI=%d, state=%d", (uint32_t)this, _ledData.stepI, _ledData.state);
}

void MoToSoftLed::toggle( void ) {
    if ( _ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    if ( _setpoint == ON  ) off();
    else on();
}

void MoToSoftLed::write( uint8_t setpntVal, uint8_t ledPar ){
    if ( _ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    _ledType = ledPar;
    write( setpntVal ) ;
}

void MoToSoftLed::write( uint8_t setpntVal ){
    //DB_PRINT( "LedWrite ix= %0x8lx, valid= 0x%x, sp=%d, lT=%d", (uint32_t)this), ledValid, setpntVal, _ledType );
    if ( _ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    if ( setpntVal == ON ) on(); else off();
    #ifdef debug
    // im Debugmode hier die Led-Daten ausgeben
    //DB_PRINT( "_ledData[%d]\n\speed=%d, Type=%d, aStep=%d, stpCnt=%d, state=%d, _setpoint= %d", ledValid, _ledSpeed, _ledType, _ledData.aStep, _ledData.stpCnt, _ledData.state, _setpoint);
    //DB_PRINT( "ON=%d, NextCyc=%d, CycleCnt=%d, StepIx=%d, NextStep=%d", 
    //         ON, ledNextCyc, ledCycleCnt, ledStepIx, ledNextStep);
    #endif
}

void MoToSoftLed::riseTime( uint16_t riseTime ) {
    if ( _ledData.state ==  NOTATTACHED ) return;
    // length of startphase in ms (min 20ms, max 65000ms )
    // 
    uint16_t stepMax;
    if ( riseTime <= 20 ) riseTime = 20;
    _ledSpeed = riseTime;
    stepMax = riseTime * 1000L / PWMCYC; // Nbr of pwm steps from ON to OFF and vice versa
    // adjust stepnumbers for ISR
    _computeBulbValues();
    noInterrupts();
    _ledData.stepI = (long)_ledData.stepI * stepMax / _ledData.stepMax; // adjust actual position to new risetime
    _ledData.stepMax = stepMax;
    interrupts();
    DB_PRINT( "_ledSpeed = %d ( risetime=%d ), StepMax=%d", _ledSpeed, riseTime, stepMax );
}

#endif // IS_32BIT exept ESP
