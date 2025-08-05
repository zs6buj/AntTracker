/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2020 All right reserved.

  Functions for the stepper part of MobaTools
*/

#if defined ARDUINO_ARCH_AVR || defined ARDUINO_ARCH_MEGAAVR //this is only for 8Bit AVR controllers
#define COMPILING_MOTOSOFTLED_CPP


#include <MobaTools.h>
//#define debugPrint
//#define debugTP
#include <utilities/MoToDbg.h>

// Global Data for all instances and classes  --------------------------------

// variables for softLeds
static ledData_t* ledRootP = NULL; //start of _ledData-chain
static uint8_t ledNextCyc = TIMERPERIODE  / CYCLETIME;     // next Cycle that is relevant for leds
static uint8_t ledCycleCnt = 0;    // count IRQ cycles within PWM cycle

static ledData_t*  ledDataP;              // pointer to active Led in ISR
void softledISR(uintx8_t cyclesLastIRQ) { // uint8 for AVR, uint32 for 32-Bit processors
    // ---------------------- softleds -----------------------------------------------
    SET_TP2;
    ledCycleCnt += cyclesLastIRQ;
    if ( ledCycleCnt >= ledNextCyc ) {
        // this IRQ is relevant for softleds
        ledNextCyc = LED_CYCLE_MAX; // there must be atleast one IRQ per PWM Cycle
        if ( ledCycleCnt >= LED_CYCLE_MAX ) {
            // start of a new PWM Cycle - switch all leds with rising/falling state to on
            // When rising, check if full on is reached
            ledCycleCnt = 0;
            for ( ledDataP=ledRootP; ledDataP!=NULL; ledDataP = ledDataP->nextLedDataP ) {
                //SET_TP1;
                // loop over led-objects
                switch ( ledDataP->state ) {
                  case INCBULB:
                  case INCLIN:
                    // led with rising brightness
                    noInterrupts();
                    if (ledDataP->invFlg  ) {
                        #ifdef FAST_PORTWRT
                        *ledDataP->portPin.Adr &= ~ledDataP->portPin.Mask;
                        #else
                        digitalWrite( ledDataP->pin, LOW );
                        #endif
                    } else { 
                        #ifdef FAST_PORTWRT
                        *ledDataP->portPin.Adr |= ledDataP->portPin.Mask;
                        #else
                        digitalWrite( ledDataP->pin, HIGH );
                        #endif
                    }
                    interrupts();
                    // check if led on is reached
                    if ( ledDataP->aCycle >=  LED_CYCLE_MAX ) {    // led is full on, remove from active-chain
                        SET_TP4;
                        ledDataP->state = STATE_ON;
                        *ledDataP->backLedDataPP = ledDataP->nextLedDataP;
                        if ( ledDataP->nextLedDataP ) ledDataP->nextLedDataP->backLedDataPP = ledDataP->backLedDataPP;
                        ledDataP->aCycle = 0;
                        CLR_TP4;
                    } else { // set off-time
                        if ( ledNextCyc > ledDataP->aCycle ) ledNextCyc = ledDataP->aCycle;
                        ledDataP->actPulse = true;
                    }
                    break;
                  case DECBULB:
                  case DECLIN:
                    // led with falling brightness
                    if (ledDataP->invFlg  ) {
                        #ifdef FAST_PORTWRT
                        *ledDataP->portPin.Adr &= ~ledDataP->portPin.Mask;
                        #else
                        digitalWrite( ledDataP->pin, LOW );
                        #endif
                    } else {
                        #ifdef FAST_PORTWRT
                        *ledDataP->portPin.Adr |= ledDataP->portPin.Mask;
                        #else
                        digitalWrite( ledDataP->pin, HIGH );
                        #endif
                    }
                    // set off-time 
                    if ( ledNextCyc > ledDataP->aCycle ) ledNextCyc = ledDataP->aCycle;
                    ledDataP->actPulse = true;
                    break;
                  default: ;
                } // end of 'switch'
                //CLR_TP1;
            } // end of led loop
        } else { // is switchofftime within PWM cycle
            SET_TP3;
            for ( ledDataP=ledRootP; ledDataP!=NULL; ledDataP = ledDataP->nextLedDataP ) {
                //SET_TP4;
                if ( ledDataP->actPulse ) {
                    // led is within PWM cycle with output high
                    if ( ledDataP->aCycle <= ledCycleCnt ) {
                        uint8_t tmpIx;
                        // End of ON-time is reached
                        SET_TP4;
                        if (ledDataP->invFlg  ) {
                            #ifdef FAST_PORTWRT
                            *ledDataP->portPin.Adr |= ledDataP->portPin.Mask;
                            #else
                            digitalWrite( ledDataP->pin, HIGH );
                            #endif
                        } else {
                            #ifdef FAST_PORTWRT
                            *ledDataP->portPin.Adr &= ~ledDataP->portPin.Mask;
                            #else
                            digitalWrite( ledDataP->pin, LOW );
                            #endif
                        }
                        CLR_TP4;
                        ledDataP->actPulse = false; // Led pulse is LOW now
                        // determine length of next PWM Cyle
                        //SET_TP1;
                        SET_TP4;
                        ledDataP->aStep += ledDataP->speed;
                        tmpIx = (ledDataP->aStep/DELTASTEPS);
                        if ( tmpIx > LED_IX_MAX ) {
                            // the end is reached
                            CLR_TP4;
                            switch ( ledDataP->state ) {
                              case DECBULB:
                              case DECLIN:
                                // led is off -> remove from chain
                                ledDataP->state = STATE_OFF;
                                *ledDataP->backLedDataPP = ledDataP->nextLedDataP;
                                if ( ledDataP->nextLedDataP ) ledDataP->nextLedDataP->backLedDataPP = ledDataP->backLedDataPP;
                                break;
                              case INCBULB:
                              case INCLIN:
                                // switch permanetly on wirh next cycle
                                ledDataP->aCycle = LED_CYCLE_MAX;
                                break;
                              default:
                                ;
                            }
                            SET_TP4;
                        } else {
                            // we are still in up/down
                            CLR_TP4;
                            switch ( ledDataP->state ) {
                              case INCBULB:
                                ledDataP->aCycle = pgm_read_byte(&(iSteps[tmpIx]));
                                break;
                              case DECBULB:
                                //CLR_TP1;
                                ledDataP->aCycle = LED_CYCLE_MAX-pgm_read_byte(&(iSteps[tmpIx]));
                                //SET_TP1;
                                break;
                              case INCLIN:
                                ledDataP->aCycle = tmpIx;
                                break;
                              case DECLIN:
                                ledDataP->aCycle = LED_CYCLE_MAX - tmpIx;
                                break;
                              default:
                                // no action if state is one of NOTATTACHED, STATE_ON, STATE_OFF
                                break;
                            }
                            SET_TP4;
                        }
                        CLR_TP4;
                    } else { 
                       // End of ON-time not yet reached, compute next necessary step
                       CLR_TP3;
                       ledNextCyc = min( ledDataP->aCycle, ledNextCyc);
                       SET_TP3;
                    }
                }
                //CLR_TP4;
            }
            CLR_TP3;
        }
        //CLR_TP3;
     } // end of softleds 
    //CLR_TP3;
    nextCycle = min( nextCycle, ( ledNextCyc-ledCycleCnt ) );
    //SET_TP3;
    CLR_TP2;
} //=============================== End of softledISR ========================================
/////////////////////////////////////////////////////////////////////////////
//Class MoToMoToSoftLed - for Led with soft on / soft off ---------------------------
// Version with Software PWM
MoToSoftLed::MoToSoftLed() {
    _ledData.speed    = 0;           // defines rising/falling timer
    _ledData.aStep    = DELTASTEPS ;          // actual PWM step
    _ledData.aCycle   = 0;           // actual cycle ( =length of PWM pule )
    _ledData.actPulse = false;       // PWM pulse is active
    _ledData.state    = NOTATTACHED; // initialize 
    _setpoint = OFF ;                // initialize to off
    _ledType = LINEAR;
    _ledData.nextLedDataP = NULL;    // don't put in ISR chain
    _ledData.invFlg = false;
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
    
uint8_t MoToSoftLed::attach(uint8_t pinArg, uint8_t invArg ){
    // Led-Ausgang mit Softstart. 
    
    _ledData.invFlg  = invArg;
    pinMode( pinArg, OUTPUT );
    //DB_PRINT( "Led attached, ledIx = 0x%x, Count = %d", ledIx, ledCount );
    _ledData.state   = STATE_OFF ;   // initialize 
    riseTime( LED_DEFAULT_RISETIME );
    if ( _ledData.invFlg ) { 
        digitalWrite( pinArg, HIGH );
    } else {
        digitalWrite( pinArg, LOW );
    }
    
    #ifdef FAST_PORTWRT
    _ledData.portPin.Adr = portOutputRegister(digitalPinToPort(pinArg));
    _ledData.portPin.Mask = digitalPinToBitMask(pinArg);
    #else
    _ledData.pin=pinArg ;      // Pin-Nbr 
    #endif
    
    seizeTimerAS();
    // enable compareB- interrupt
    #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
        TIMSK |= ( _BV(OCIExB) );    // enable compare interrupts
    #else
        _stepIRQ() ; // Softled uses same IRQ as steppers
    #endif
    DB_PRINT("IX_MAX=%d, CYCLE_MAX=%d, PWMTIME=%d", LED_IX_MAX, LED_CYCLE_MAX, LED_PWMTIME );
    return true;
}


void MoToSoftLed::on(){
    if ( _ledData.state ==  NOTATTACHED ) return;  // this is not a valid instance
    LedStats_t stateT;
    // Don't do anything if its already ON 
    if ( _setpoint != ON  ) {
        _setpoint        = ON ;
        /*if ( _ledData.state < ACTIVE ) {
            // is full off*/
            _ledData.aStep   = DELTASTEPS;
        /*} else {
            // is counting up
            _ledData.aStep = LED_STEP_MAX - _ledData.aStep;
        }*/
        _ledData.speed   = _ledSpeed;
        if ( _ledType == LINEAR ) {
            stateT          = INCLIN;
            _ledData.aCycle  = 1;
        } else { // is bulb simulation
            stateT          = INCBULB;
            _ledData.aCycle  = iSteps[1];
        }
        mount(stateT);
    }
    DB_PRINT( "Led %04X On, state=%d, ledRoot=%04X", (uint32_t)this, _ledData.state, (uintxx_t)ledRootP);
}

void MoToSoftLed::off(){
    if ( _ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    LedStats_t stateT;
    // Dont do anything if its already OFF 
    if ( _setpoint != OFF ) {
        //SET_TP3;
        _setpoint            = OFF;
        /*if ( _ledData.state < ACTIVE ) {
            // is full on*/
            _ledData.aStep   = DELTASTEPS;
        /*} else {
            // is counting up
            _ledData.aStep = LED_STEP_MAX -_ledData.aStep;
        }*/
        _ledData.speed   = _ledSpeed;
        if ( _ledType == LINEAR ) {
            stateT          = DECLIN;
            _ledData.aCycle  = LED_IX_MAX;
        } else { // is bulb simulation
            //CLR_TP3;
            stateT = DECBULB;
            _ledData.aCycle = LED_CYCLE_MAX  - iSteps[1];
        }
        //CLR_TP3;
        mount(stateT);
    }
    DB_PRINT( "Led %04X Off, state=%d", (uint32_t)this, _ledData.state);
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
    //DB_PRINT( "LedWrite ix= %d, valid= 0x%x, sp=%d, lT=%d", ledIx, ledValid, setpntVal, _ledType );
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
    // length of startphase in ms (min 20ms )
    // The max value ist slower, if CYCLETIME is reduced.
    // risetime is computed to a 'speed' Value with 16 beeing the slowest 
    // with speed value = 16 means risetime is (LED_CYCLE_MAX * LED_PWMTIME * DELTATIME / 16)
    // risetime = (LED_CYCLE_MAX * LED_PWMTIME * DELTATIME) / speed
    // 
    long riseMax = ((long) LED_CYCLE_MAX * DELTASTEPS * LED_PWMTIME );
    if ( riseTime <= 20 ) riseTime = 20;
    if ( riseTime >= riseMax/16 ) riseTime = riseMax/16;
    int tmp = ( ((long)riseMax  *10) / ( riseTime  ) +5 ) /10;
    _ledSpeed = tmp;
    DB_PRINT( "_ledSpeed = %d ( risetime=%d, riseMax=%d, PWMTIME=%d )", _ledSpeed, riseTime, riseMax, LED_PWMTIME );
}

// For compatibility with code written for other platforms too 
// The avr implementation ignores parameters in the on and off method
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

void MoToSoftLed::on(uint8_t value){
    on();
}
void MoToSoftLed::off(uint8_t value){
    off();
}
#pragma GCC diagnostic pop


#endif // ARDUINO_ARCH_AVR
