// MoToStepper: ISR and SPI functions and Methods that are not used in ESP8266
// The difference in these functions is too big to do it with function internal ifdef's

//Variables not used with ESP8266
extern bool timerInitialized;

// constants
static stepperData_t *stepperRootP = NULL;    // start of stepper data chain ( NULL if no stepper object )
uint8_t spiStepperData[2]; // step pattern to be output on SPI
                            // low nibble of spiStepperData[0] is SPI_1
                            // high nibble of spiStepperData[1] is SPI_4
                            // spiStepperData[1] is shifted out first

bool IRAM_ATTR setStepperPins( stepperData_t *stepperDataP, uint8_t stepPattern ) {
	// setting 4-pin stepperdata to enable/disable the stepper
	//CLR_TP2;SET_TP2;
	// store pattern data
	bool spiChanged = false;
	switch ( stepperDataP->output ) {
		// V2.6: PIN8_11/PIN4_7 not allowed anymore ( wasn't described in Doku since V0.8
	  case SPI_1:
		//SET_TP2;
		spiStepperData[0] = (spiStepperData[0] & 0xf0) | ( stepPattern );
		spiChanged = true; 
		//CLR_TP2;
		break;
	  case SPI_2:
		spiStepperData[0] = (spiStepperData[0] & 0x0f) | ( stepPattern <<4 );
		spiChanged = true;
		//CLR_TP2;
		break;
	  case SPI_3:
		spiStepperData[1] = (spiStepperData[1] & 0xf0) | ( stepPattern );   
		spiChanged = true;
		break;
	  case SPI_4:
		spiStepperData[1] = (spiStepperData[1] & 0x0f) | ( stepPattern <<4 );
		spiChanged = true;
		break;
	#ifdef ESP32 // !!! Auf ESP32 darf ein switch in ISR max 4 case-Blöcke enthalten !!!!
	}
	switch ( stepperDataP->output ) {
	#endif
	  case SINGLE_PINS : // Outpins are individually defined
		for ( uint8_t bitNr = 0; bitNr < 4; bitNr++ ) {
			// setStepperPinsAS( bitNr, stepPattern & (1<<bitNr) );
			if ( stepPattern & (1<<bitNr) ) {
				#ifdef FAST_PORTWRT
				*stepperDataP->portPins[bitNr].Adr |= stepperDataP->portPins[bitNr].Mask;
				#else
				digitalWrite( stepperDataP->pins[bitNr], HIGH );
				#endif
			} else {
				#ifdef FAST_PORTWRT
				*stepperDataP->portPins[bitNr].Adr &= ~stepperDataP->portPins[bitNr].Mask;
				#else    
				digitalWrite( stepperDataP->pins[bitNr], LOW );
				#endif    
			}
		}
		break;
	  default:
		// should never be reached
		break;
	}
	return spiChanged;
}

#pragma GCC optimize "O3"   // optimize ISR for speed
void IRAM_ATTR stepperISR(nextCycle_t cyclesLastIRQ) {
    //SET_TP4;
    static const int DRAM_ATTR stepPattern[8] = {0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,0b0001 };
    stepperData_t *stepperDataP;         // actual stepper data in IRQ
    uint8_t spiChanged, changedPins, bitNr;
    //SET_TP1;SET_TP4; // Oszimessung Dauer der ISR-Routine
    spiChanged = false;
    #ifdef __AVR_MEGA__
    nestedInterrupts(); // allow nested interrupts, because this IRQ may take long
    #endif
    stepperDataP = stepperRootP;
    // ---------------Stepper motors ---------------------------------------------
    while ( stepperDataP != NULL ) {
        #ifdef IS_32BIT
        bool resetPulse = false; // for 32Bit reset pulse und setting new pulse is never done in same IRQ
        if ( stepperDataP->output == A4988_PINS && digitalRead(stepperDataP->pins[0]) ) {
        #else
        if ( stepperDataP->output == A4988_PINS  ) {
        #endif
            // reset step pulse - pulse is max one cycle length
            #ifdef FAST_PORTWRT
            noInterrupts();
            *stepperDataP->portPins[0].Adr &= ~stepperDataP->portPins[0].Mask;
            interrupts();
            #else
            digitalWrite( stepperDataP->pins[0], LOW );
			#ifdef IS_32BIT
            resetPulse = true;
			#endif
            #endif
        } // end of resetting step pulse
		
        if ( stepperDataP->rampState >= rampStat::CRUISING &&  stepperDataP->speedZero != ZEROSPEEDACTIVE ) {
            // only active motors with speed > 0
            //if ( stepperDataP->aCycSteps ) stepperDataP->cycCnt+=cyclesLastIRQ;
            if ( stepperDataP->aCycSteps ) stepperDataP->cycCnt=stepperDataP->cycCnt+cyclesLastIRQ;
            #ifdef IS_32BIT
            if ( stepperDataP->cycCnt >= ( stepperDataP->aCycSteps ) && !resetPulse ) {
                //SET_TP2;
                stepperDataP->cycCnt = stepperDataP->cycCnt - stepperDataP->aCycSteps ;
                // 'Aufholen' zu langsamer Interrupts begrenzen
                // cycCnt darf nie größer als aCycSteps werden ( maximale Steprate )!
                if ( stepperDataP->cycCnt >= stepperDataP->aCycSteps+MAX_JITTER ) { 
                    //SET_TP4; 
                    stepperDataP->cycCnt = stepperDataP->aCycSteps+MAX_JITTER; 
                }
                //CLR_TP4;
            #else // not 32bit ( never use a jitter computing )
            if ( stepperDataP->cycCnt >= ( stepperDataP->aCycSteps ) ) {
                //SET_TP2;
                stepperDataP->cycCnt = 0 ;
            #endif
                //SET_TP2;
                // Do one step
                // update position for absolute positioning
                //stepperDataP->stepsFromZero += stepperDataP->patternIxInc;
                stepperDataP->stepsFromZero = stepperDataP->stepsFromZero +stepperDataP->patternIxInc;
                
                // sign of patternIxInc defines direction
                int8_t _patIx;
                _patIx = stepperDataP->patternIx + stepperDataP->patternIxInc;
                if ( _patIx > 7 ) _patIx = 0;
                if ( _patIx < 0 ) _patIx += 8;
                stepperDataP->patternIx = _patIx;
                //CLR_TP2;SET_TP2;
                // store pattern data
                #ifdef __AVR_MEGA__
                noInterrupts(); // because of read modify write actions in setting outputs
                #endif
                switch ( stepperDataP->output ) {
					// V2.6: PIN8_11/PIN4_7 not allowed anymore ( wasn't described in Doku since V0.8
                  case SPI_1:
                    //SET_TP2;
                    spiStepperData[0] = (spiStepperData[0] & 0xf0) | ( stepPattern[ _patIx ] );
                    spiChanged = true; 
                    //CLR_TP2;
                    break;
                  case SPI_2:
                    spiStepperData[0] = (spiStepperData[0] & 0x0f) | ( stepPattern[ _patIx ] <<4 );
                    spiChanged = true;
                    //CLR_TP2;
                    break;
                  case SPI_3:
                    spiStepperData[1] = (spiStepperData[1] & 0xf0) | ( stepPattern[ _patIx ] );   
                    spiChanged = true;
                    break;
                  case SPI_4:
                    spiStepperData[1] = (spiStepperData[1] & 0x0f) | ( stepPattern[ _patIx ] <<4 );
                    spiChanged = true;
                    break;
                #ifdef ESP32 // !!! Auf ESP32 darf ein switch in ISR max 4 case-Blöcke enthalten !!!!
                }
                switch ( stepperDataP->output ) {
                #endif
                  case SINGLE_PINS : // Outpins are individually defined
                    changedPins = stepPattern[ _patIx ] ^ stepperDataP->lastPattern;
                    for ( bitNr = 0; bitNr < 4; bitNr++ ) {
                        if ( changedPins & (1<<bitNr ) ) {
                            // bit Changed, write to pin
                            if ( stepPattern[ _patIx ] & (1<<bitNr) ) {
                                #ifdef FAST_PORTWRT
                                *stepperDataP->portPins[bitNr].Adr |= stepperDataP->portPins[bitNr].Mask;
                                #else
                                digitalWrite( stepperDataP->pins[bitNr], HIGH );
                                #endif
                            } else {
                                #ifdef FAST_PORTWRT
                                *stepperDataP->portPins[bitNr].Adr &= ~stepperDataP->portPins[bitNr].Mask;
                                #else    
                                digitalWrite( stepperDataP->pins[bitNr], LOW );
                                #endif    
                            }
                        }
                    }
                    stepperDataP->lastPattern = stepPattern[ _patIx ];
                    break;
                  case A4988_PINS : // output step-pulse and direction
                    // direction first
                    //SET_TP2;
                    if ( stepperDataP->patternIxInc > 0 ) {
                        // turn forward 
                        #ifdef FAST_PORTWRT
                        *stepperDataP->portPins[1].Adr |= stepperDataP->portPins[1].Mask;
                        #else
                        digitalWrite( stepperDataP->pins[1], HIGH );
                        #endif
                    } else {
                        // turn backwards
                        #ifdef FAST_PORTWRT
                        *stepperDataP->portPins[1].Adr &= ~stepperDataP->portPins[1].Mask;
                        #else
                        digitalWrite( stepperDataP->pins[1], LOW );
                        #endif
                    }    
                    // Set step pulse 
                    nextCycle = MIN_STEP_CYCLE/2; // will be resettet in half of min steptime
                    #ifdef FAST_PORTWRT
                    *stepperDataP->portPins[0].Adr |= stepperDataP->portPins[0].Mask;
                    #else
					digitalWrite( stepperDataP->pins[0], HIGH );
                    //SET_TP4;
                    #endif
                    break;
                  default:
                    // should never be reached
                    break;
                }
                #ifdef __AVR_MEGA__
                interrupts();
                #endif
                //CLR_TP2;
                // ------------------ check if last step -----------------------------------
                //if ( --stepperDataP->stepCnt == 0 ) {
				stepperDataP->stepCnt = stepperDataP->stepCnt	-1;
                if ( stepperDataP->stepCnt == 0 ) {
                    // this was the last step.
                    if (stepperDataP->stepCnt2 > 0 ) { // check if we have to start a movement backwards
                        // yes, change Direction and go stpCnt2 Steps
                        stepperDataP->patternIxInc = -stepperDataP->patternIxInc;
                        stepperDataP->stepCnt = stepperDataP->stepCnt2;
                        stepperDataP->stepCnt2 = 0;
                        stepperDataP->rampState = rampStat::RAMPACCEL;
                    } else {
                        stepperDataP->stepsInRamp = 0;      // we cannot be in ramp when stopped
                        if (stepperDataP->enableOn) {
                            // enable is active, wait for disabling
                            stepperDataP->aCycSteps = stepperDataP->cycDelay;
							nextCycle = MIN_STEP_CYCLE/2; // step pulse will be resettet in half of min steptime
                            stepperDataP->rampState = rampStat::STOPPING;
                        } else {    
                        stepperDataP->aCycSteps = ISR_IDLETIME;    // no more Interrupts for this stepper needed
                        stepperDataP->rampState = rampStat::STOPPED;
                        //CLR_TP2;
                        }
                    }
                }
                // --------------- compute nexte steplength ------------------------------------
                //SET_TP2;
                // ramp state machine
                // !!! Auf ESP32 darf ein switch in ISR max 4 case-Blöcke enthalten !!!!
                switch ( stepperDataP->rampState ) {
                  case  rampStat::RAMPACCEL:
                    // we are accelerating the motor
                    if (stepperDataP->stepsInRamp >= stepperDataP->stepRampLen ) {
                        // we reached the end of the ramp
                        stepperDataP->aCycSteps = stepperDataP->tCycSteps;
                        #ifndef IS_32BIT
                        stepperDataP->aCycRemain = stepperDataP->tCycRemain;
                        #endif
                        stepperDataP->stepsInRamp = stepperDataP->stepRampLen;
                        stepperDataP->rampState = rampStat::CRUISING;
                    } else {
                        stepperDataP->aCycSteps = stepperDataP->cyctXramplen / (stepperDataP->stepsInRamp + RAMPOFFSET) ;//+1;
                        #ifndef IS_32BIT
                        stepperDataP->aCycRemain += stepperDataP->cyctXramplen % (stepperDataP->stepsInRamp + RAMPOFFSET);
                       if ( stepperDataP->aCycRemain > (stepperDataP->stepsInRamp + RAMPOFFSET) ) {
                            stepperDataP->aCycSteps++;
                            stepperDataP->aCycRemain -= (stepperDataP->stepsInRamp + RAMPOFFSET);
                        }
                        #endif
                        // do we have to start deceleration ( remaining steps < steps in ramp so far )
                        // Ramp must be same length in accelerating and decelerating!
                        if ( stepperDataP->stepCnt <= ( stepperDataP->stepsInRamp+1U  ) ) {
                            //CLR_TP2;
                            stepperDataP->rampState = rampStat::RAMPDECEL;
                            //DB_PRINT( "scnt=%ld, sIR=%u\n\r", stepperDataP->stepCnt, stepperDataP->stepsInRamp );
                            //SET_TP2;
                        } else {
                            // still in ramp
                            stepperDataP->stepsInRamp ++;
                        }    
                    }
                    break;
                  case rampStat::RAMPDECEL:
                  case rampStat::SPEEDDECEL:
                    if ( stepperDataP->stepsInRamp <= stepperDataP->stepRampLen ) {
                        // we are stopping the motor
                        if ( stepperDataP->stepCnt > (uint32_t)( stepperDataP->stepsInRamp ) ) {
                            //CLR_TP2; // ToDo: check whether this in necessary ( schould be done in method that changes steps to  move)
                            //steps to move has changed, accelerate again with next step
                            stepperDataP->rampState = rampStat::RAMPACCEL;
                            //DB_PRINT( "scnt=%ld, sIR=%u\n\r", stepperDataP->stepCnt, stepperDataP->stepsInRamp );
                            //SET_TP2;
                        }
						// check if we can decrement steps in Ramp
 						if ( stepperDataP->stepsInRamp > 0 ) --stepperDataP->stepsInRamp;
 						#ifdef debugPrint
 						//else stepperDataP->errorFlg = 1; // must not be reached!!
 						#endif
                         stepperDataP->aCycSteps = stepperDataP->cyctXramplen / ( stepperDataP->stepsInRamp + RAMPOFFSET ) ;// +1 ;
                          #ifndef IS_32BIT
                        stepperDataP->aCycRemain += stepperDataP->cyctXramplen % (stepperDataP->stepsInRamp + RAMPOFFSET);
                        if ( stepperDataP->aCycRemain > (stepperDataP->stepsInRamp + RAMPOFFSET) ) {
                            stepperDataP->aCycSteps++;
                            stepperDataP->aCycRemain -= (stepperDataP->stepsInRamp + RAMPOFFSET);
                        }
                        #endif
                    } else {
                        // lower speed to new value 
                        if ( (stepperDataP->stepsInRamp-stepperDataP->stepRampLen) > stepperDataP->deltaSteps ) {
                            // steps in ramp still greater than delta
                            stepperDataP->stepsInRamp -=stepperDataP->deltaSteps;
                            stepperDataP->aCycSteps = stepperDataP->cyctXramplen / ( stepperDataP->stepsInRamp + RAMPOFFSET ) ;//+1 ;
                            #ifndef IS_32BIT
                            stepperDataP->aCycRemain += stepperDataP->cyctXramplen % (stepperDataP->stepsInRamp + RAMPOFFSET);
                            if ( stepperDataP->aCycRemain > (stepperDataP->stepsInRamp + RAMPOFFSET) ) {
                                stepperDataP->aCycSteps++;
                                stepperDataP->aCycRemain -= (stepperDataP->stepsInRamp + RAMPOFFSET);
                            }
                            #endif
                        } else {
                            // new targetspeed reached
                            //SET_TP2;
                            stepperDataP->rampState = rampStat::CRUISING;
                            stepperDataP->stepsInRamp =  stepperDataP->stepRampLen;
                            stepperDataP->aCycSteps = stepperDataP->tCycSteps;
                            #ifndef IS_32BIT
                            stepperDataP->aCycRemain = 0;
                            #endif
							if ( stepperDataP->speedZero == DECELSPEEDZERO ) {
								//target speed was 0, create no more steps
								stepperDataP->speedZero = ZEROSPEEDACTIVE;
								// if enabe is active, start Time until disabling the motor
								if (stepperDataP->enableOn) {
									// enable is active, wait for disabling
									stepperDataP->aCycSteps = stepperDataP->cycDelay;
									stepperDataP->rampState = rampStat::STOPPING;
								}   
							}
                            //CLR_TP2;
                        }
                    }
                    break;
                    
                  case rampStat::CRUISING:
                    // Not in ramp, targetspeed reached - or without ramp at all
                    //CLR_TP2;
                    stepperDataP->aCycSteps = stepperDataP->tCycSteps;
                    #ifndef IS_32BIT
                    stepperDataP->aCycRemain += stepperDataP->tCycRemain;
                    if  ( stepperDataP->aCycRemain > CYCLETIME ) {
                        stepperDataP->aCycRemain -= CYCLETIME;
                        stepperDataP->aCycSteps++;
                    }
                    #endif
                    // do we have to start the deceleration
                    if ( stepperDataP->stepCnt <= stepperDataP->stepRampLen+1U && stepperDataP->stepRampLen != 0 ) {
                        // in mode without ramp ( stepRampLen == 0 ) , this can never be true
						// V2.6.2 - added explicit check of ramplen to be 0
                        stepperDataP->rampState = rampStat::RAMPDECEL;
                    }
                    
                    break;
                    
                  default:
                    //stepper does not move -> nothing to do
                    //CLR_TP2;
                    break;
                } // End of ramp-statemachine
                //CLR_TP2;
            } // End of do one step
			//CLR_TP2;
            #ifdef IS_32BIT
            // check if max possible speed - is made at the beginning of the ISR
            if ( stepperDataP->aCycSteps+MAX_JITTER < stepperDataP->cycCnt ) {
                stepperDataP->cycCnt = stepperDataP->aCycSteps+MAX_JITTER ;
                //SET_TP2; 
            }
            #endif
            nextCycle = (nextCycle_t)min ( (uintxx_t)nextCycle, (uintxx_t)stepperDataP->aCycSteps+MAX_JITTER-stepperDataP->cycCnt );
            CLR_TP2;
        } // end of 'if stepper active AND moving'
		
        else if ( stepperDataP->rampState == rampStat::STARTING && stepperDataP->speedZero != ZEROSPEEDACTIVE ) {
            // we start with enable function active ( cycCnt is already set to 0 resp. MAX_JITTER )
			// enable the motor and wait delaytime (cycDelay) before executing first step.
			// if no enablepin is defined, the steppattern must be set to the last active pattern.
			if ( stepperDataP->enablePin == NO_ENABLEPIN ) {
				// set motorwires to last pattern ( as it was when disabling the outputs )
				spiChanged = setStepperPins( stepperDataP, stepPattern[ stepperDataP->patternIx ] );
			}
			else {
				// set enablePin to active
				digitalWrite( stepperDataP->enablePin, stepperDataP->enable );
			}
            stepperDataP->aCycSteps = stepperDataP->cycDelay;
            if ( stepperDataP->stepRampLen > 0 ) stepperDataP->rampState = rampStat::RAMPACCEL;
            else                                stepperDataP->rampState = rampStat::CRUISING;
            nextCycle = (nextCycle_t)min ( (uintxx_t)nextCycle, (uintxx_t)stepperDataP->aCycSteps );
        } 
		else if ( stepperDataP->rampState == rampStat::STOPPING  ) {
			// Check if time between last step and disabling the motor has elapsed
            //stepperDataP->cycCnt+=cyclesLastIRQ;
            stepperDataP->cycCnt=stepperDataP->cycCnt+cyclesLastIRQ;
            if ( stepperDataP->cycCnt >= stepperDataP->aCycSteps+MAX_JITTER ) {
                stepperDataP->cycCnt = MAX_JITTER;
				if ( stepperDataP->enablePin == NO_ENABLEPIN ) {
					// set motorwires to 0 to disable the motor
					spiChanged = setStepperPins( stepperDataP, 0 );
				} 
				else {
					// set enablePin to inactive
					digitalWrite( stepperDataP->enablePin, !stepperDataP->enable );
				}
                stepperDataP->rampState = rampStat::STOPPED;
            }
			nextCycle = (nextCycle_t)min ( (uintxx_t)nextCycle, (uintxx_t)stepperDataP->aCycSteps+MAX_JITTER-stepperDataP->cycCnt );

        }

        stepperDataP = stepperDataP->nextStepperDataP;
        //SET_TP1; //CLR_TP2;
    } // end of stepper-loop
    
    // shift out spiStepperData, if SPI is active
    //SET_TP2;
    if ( spiInitialized && spiChanged ) {
        //SET_TP4;
        startSpiWriteAS( spiStepperData );
        //CLR_TP4;
    }
    //CLR_TP4;
} // ==================== End of stepper ISR ======================================
#pragma GCC optimize "Os"

uintxx_t MoToStepper::setSpeedSteps( uintxx_t speed10, intxx_t rampLen ) {
    // Set speed and length of ramp to reach speed ( from stop )
    // neagtive ramplen means it was set automatically
     SET_TP4;
    rampStat newRampState;      // State of acceleration/deceleration
    uintxx_t tCycSteps;         // nbr of IRQ cycles per step ( new target value of motorspeed  )
	#ifndef IS_32BIT
    uint16_t tCycRemain;        // Remainder of division when computing tCycSteps
    long     tMicroSteps;       // Microseconds per step
	#endif
    uintxx_t newCyctXramplen;      // precompiled  tCycSteps*rampLen*RAMPOFFSET
    uintxx_t newRampLen;         // new ramplen
    uintxx_t newStepsInRamp;     // new stepcounter in ramp - according to new speed and ramplen
    intxx_t  newDeltaSteps = 1;  //  only for SPEEDDECEL
    uintxx_t newSpeed10;        // new target speed

    if ( _stepperData.output == NO_OUTPUT ) return 0; // --------------->>>>>>>>>>>>>>>>not attached
    // compute new speed values, adjust length of ramp if necessary
    //actSpeed10 = oldSpeed10 = _stepSpeed10;
        
    newRampLen = abs(rampLen);    // negative values are invalid ( indicate automatically adjusted length )
    if (rampLen<0) newRampLen--;
    if (newRampLen > MAXRAMPLEN ) newRampLen = MAXRAMPLEN;
    newSpeed10 = min( uintxx_t(1000000L / MIN_STEPTIME * 10), speed10 );
	_noStepIRQ();
	if ( newSpeed10 == 0) {
		if ( _stepperData.speedZero == NORMALSPEED ) {
			// We are not yet in ZEROSPEED-mode so stop the stepper to standstill
			// Because ramplen has been set to 0 when already in ZEROSPEED mode we must not do that again if
			// already active.
			if ( _chkRunning() ) { 
				// stepper is moving, we hahe to stop it
				if ( _stepperData.stepRampLen > 0) {
					_stepperData.speedZero = DECELSPEEDZERO;	// there is a ramp, flag for ramping down to speed 0
				} else {
					// no ramp, simply inhibit creating pulses, disable stepper if needed.
					_stepperData.speedZero = ZEROSPEEDACTIVE;				
					if (_stepperData.enableOn ) {
						_stepperData.aCycSteps = _stepperData.cycDelay;
						_stepperData.rampState = rampStat::STOPPING;
					}
				}
			} else { 
			  // Stepper doesn't move, only inhibit starting
				_stepperData.speedZero = ZEROSPEEDACTIVE;	// set stepper inactive ( no ISR action )
			}  
		}
		newSpeed10 = MINSPEEDZERO; // minimum speed in ramp before stopping
	} else {
		if (_stepperData.enableOn && _stepperData.speedZero == ZEROSPEEDACTIVE ) {
			// We are starting from zero speed and enable is active, wait for enabling
			_stepperData.aCycSteps = _stepperData.cycDelay;
			_stepperData.rampState = rampStat::STARTING;
		}   
		_stepperData.speedZero = NORMALSPEED;
	}
	_stepIRQ(true);
	
    
    // compute target steplength and check whether speed and ramp fit together: 
	#ifdef IS_32BIT
        tCycSteps = ( 1000000L * 10  / newSpeed10 );
        newCyctXramplen = tCycSteps * ( newRampLen + RAMPOFFSET );
	#else
        tMicroSteps = ( 1000000L * 10  / newSpeed10 );
        tCycSteps = tMicroSteps / CYCLETIME; 
        tCycRemain = tMicroSteps % CYCLETIME; 
        // tcyc * (rapmlen+RAMPOFFSET) must be less then 65000, otherwise ramplen is adjusted accordingly
        long tmp =  tMicroSteps * ( newRampLen + RAMPOFFSET ) / CYCLETIME ;
        if ( tmp > 65000L ) {
            // adjust ramplen
            newRampLen = 65000L * CYCLETIME / tMicroSteps;
            if( newRampLen > RAMPOFFSET ) newRampLen -= RAMPOFFSET; else newRampLen = 0;
            newCyctXramplen = tMicroSteps * ( newRampLen + RAMPOFFSET ) / CYCLETIME;
        } else {
            newCyctXramplen = tmp;
        }
	#endif
    if (rampLen >= 0) {
        // ramplength was set by user, update reference-values
        _lastRampSpeed = newSpeed10;
        _lastRampLen   = newRampLen;
    }
    
    // recompute all relevant rampvalues according to actual speed and ramplength
    // This needs to be done only, if a ramp is defined, the stepper is moving
    // and the speed and ramp values changed
    // In all other cases the new speed/ramp values will get active immediately
    _noStepIRQ();
    if ( (_stepperData.stepRampLen + newRampLen ) != 0
        && _chkRunning() 
        &&  newCyctXramplen != _stepperData.cyctXramplen ) {
        // local variables to hold data that might change in IRQ:
        // If there was a step during recomputing the rampvalues, we must recompute again
        // recomputing the rampvalues lasts too long to stop the IRQ during the whole time
        CLR_TP4;
        uint32_t        __stepCnt;
        uint32_t        __newStepCnt;
        uint32_t        __newStepCnt2;
        uint8_t loopCnt = 5;
        do { CLR_TP4;
            // read actual ISR values
            newRampState = _stepperData.rampState;
            __stepCnt       =  _stepperData.stepCnt; 
            __newStepCnt    = 0;    // if stepcnt is to be changed
            __newStepCnt2   = 0;
            if ( --loopCnt ) _stepIRQ(true); //CLR_TP2;
            //with ramp and ramp or speed changed 
            // compute new 'steps in Ramp' according to new speed and ramp values. This maybe greater
            // than ramplen, if speed changed to slower
			#ifdef IS_32BIT
            newStepsInRamp = ( (int64_t)newCyctXramplen * (_stepperData.stepsInRamp + RAMPOFFSET ) / _stepperData.cyctXramplen );
            newStepsInRamp = newStepsInRamp<RAMPOFFSET? 0 : newStepsInRamp-RAMPOFFSET;
			#else
            newStepsInRamp = ( (long)newCyctXramplen * (_stepperData.stepsInRamp + RAMPOFFSET ) / _stepperData.cyctXramplen );
            if ( newStepsInRamp > RAMPOFFSET ) newStepsInRamp -= RAMPOFFSET; else newStepsInRamp = 0; 
			#endif
            
            if ( newSpeed10 != _stepSpeed10 ) {
                // speed changed!
                if ( newStepsInRamp > newRampLen ) {
                    //  ==========  we are too fast ============================
                        //Serial.print(" --");
                        DB_PRINT ( "Slower: %u/%u -> %u/%u", _stepSpeed10,_stepperData.stepRampLen,  newSpeed10, newRampLen );
                        newRampState = rampStat::SPEEDDECEL;
                        if ( _stepperData.stepsInRamp == 0 ) {
                            newDeltaSteps = ((10L * newStepsInRamp)+5 )/10L;
                        } else {
                            newDeltaSteps = ((10L * newStepsInRamp / _stepperData.stepsInRamp)+5 )/10L;
                        }
                        if ( newDeltaSteps < 1 ) newDeltaSteps = 1;
                        DB_PRINT ( "Slower: %u/%u -> %u/%u, delat=%u", _stepSpeed10,_stepperData.stepRampLen,  newSpeed10, newRampLen, newDeltaSteps );
                } else  {
                    //  ==========  we are too slow ============================
                    //Serial.print(" ++"); 
                    //DB_PRINT ( "Faster: %u/%u -> %u/%u", _stepSpeed10,_stepperData.stepRampLen, newSpeed10 , newRampLen );
                    newRampState = rampStat::RAMPACCEL;
                }
            } else {
                //Serial.print(" ==");
            }

            // Check whether we can reach targetposition with new values
            if ( newStepsInRamp/newDeltaSteps > (__stepCnt - _stepperData.stepCnt2) ) {
                // we cannot reach the tagetposition, so we go beyond the targetposition and than back.
                // This works even if we are already beyond the target position
                //Serial.print( " ><");
                __newStepCnt2 = newStepsInRamp - (__stepCnt - _stepperData.stepCnt2);
                __newStepCnt = newStepsInRamp;
                newRampState = rampStat::RAMPDECEL;
            }
            if (loopCnt ) _noStepIRQ(); //SET_TP2;
            SET_TP4;
            //Serial.print(" ) ");Serial.print(_stepperData.rampState);
        } while ( __stepCnt != _stepperData.stepCnt && loopCnt ); // if there was a step during computing, do it again
        _stepperData.rampState = newRampState;
        _stepperData.stepsInRamp = newStepsInRamp;
        _stepperData.deltaSteps = newDeltaSteps;
        if ( __newStepCnt != 0 ) {
            _stepperData.stepCnt = __newStepCnt;
            _stepperData.stepCnt2 = __newStepCnt2;
        }
    } 
    
    _stepperData.tCycSteps = tCycSteps;
	#ifndef IS_32BIT
    _stepperData.tCycRemain = tCycRemain;
	#endif
    _stepperData.cyctXramplen = newCyctXramplen;
    _stepperData.stepRampLen = newRampLen;
    _stepIRQ(true); CLR_TP4;
    _stepSpeed10 = speed10 == 0? 0 : newSpeed10;
    CLR_TP4;
    prDynData();
    return _stepperData.stepRampLen;
}

extern nextCycle_t nextCycle;
//static nextCycle_t cyclesLastIRQ = 1;  // µsec since last IRQ

void MoToStepper::prDynData() {             // print actual Stepperdata
#undef debugPrint
#ifdef debugPrint
    #warning "should not be executed"
    stepperData_t printData;  // vor Ausdruck eine lokale Kopie erstellen
	intxx_t prNextCycle;
    _noStepIRQ();
    printData.stepCnt   = _stepperData.stepCnt;             // nmbr of steps to take
    printData.stepCnt2  = _stepperData.stepCnt2;            // nmbr of steps to take after automatic reverse
	printData.tCycSteps =   _stepperData.tCycSteps;         // nbr of IRQ cycles per step ( target value of motorspeed  )
	#ifndef IS_32BIT
	printData.tCycRemain =  _stepperData.tCycRemain;        // Remainder of division when computing tCycSteps
	printData.aCycRemain =  _stepperData.aCycRemain;        // accumulate tCycRemain when cruising
	#endif
	printData.aCycSteps =   _stepperData.aCycSteps;         // nbr of IRQ cycles per step ( actual motorspeed  )
	printData.cyctXramplen =_stepperData.cyctXramplen;     // precompiled  tCycSteps*(rampLen+RAMPOFFSET)
    printData.cycCnt =       _stepperData.cycCnt;             // counting cycles until cycStep
	printData.cycDelay =    _stepperData.cycDelay;          // delay time enable -> stepping

    printData.stepRampLen = _stepperData.stepRampLen;       // Length of ramp in steps
    printData.stepsInRamp = _stepperData.stepsInRamp;       // stepcounter within ramp ( counting from stop ( = 0 ): incrementing in startramp, decrementing in stopramp
    printData.rampState = _stepperData.rampState;           // State of acceleration/deceleration
    printData.stepsFromZero = _stepperData.stepsFromZero;   // distance from last reference point 
    printData.deltaSteps = _stepperData.deltaSteps;         // number of computed steps per real step in SPEEDDECEL
	printData.speedZero  = _stepperData.speedZero;
    prNextCycle = nextCycle;
    _stepIRQ();
    uint32_t time = micros();
    uint16_t usec = time%1000; time /=1000;
    uint16_t msec = time%1000; time /= 1000;
    DB_PRINT("Time:%5lu:%03u,%03u\t stepCnt=%5lu\t stepCnt2=%5lu\t sFZ=%5ld", time, msec,usec,printData.stepCnt, printData.stepCnt2, printData.stepsFromZero );
	
    #ifdef IS_32BIT
    DB_PRINT("tCySteps=%5u\t # aCySteps=%5u\t ", (unsigned int)printData.tCycSteps,(unsigned int)printData.aCycSteps);
	#else
    DB_PRINT("tCySteps=%5u\t tCyRemain=%5u\t aCySteps=%5u\t aCyRemain=%5u", printData.tCycSteps,printData.tCycRemain,printData.aCycSteps,printData.aCycRemain);
	#endif
    DB_PRINT(" XrampL=%5u\t rampLen=%4u\t stepsInRamp=%4u\t, rampState=%s(%u)",printData.cyctXramplen,printData.stepRampLen,printData.stepsInRamp,rsC[(int)printData.rampState],(int)printData.rampState);
    DB_PRINT("deltaStp=%4d,\t speedFlg=%d cycnt=%4ld, nextCyc=%4d", printData.deltaSteps, printData.speedZero, printData.cycCnt, prNextCycle );

    DB_PRINT("^^^^^^^^^^^^^^ISR-Data^^^^^^^^^^^^^^^^");
    #endif
}
