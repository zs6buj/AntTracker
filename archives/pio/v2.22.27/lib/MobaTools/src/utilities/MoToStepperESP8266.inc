// MobaTools for ESP8266
// This file contains functions that are special to the ESP Processor and are very different
// from the AVR-Versions
// Stepper ISR
// Enable is controlled by means of pulses in the dir pin ( without any step pulses )
// This is done because we have no direct time-ISR on the ESP. The dir output is always
// LOW if there are no step pulses to be created. What we need is a dedicated ISR for the dir-pin if the
// enable function is active
void ICACHE_RAM_ATTR ISR_StepperEnable( stepperData_t *stepperDataP) {
    //GPOS = (1<<2);
    if ( stepperDataP->delayActiv ) {
        // end of enable delay - start moving or switch off enable
        stepperDataP->delayActiv = false;
        if ( stepperDataP->rampState >= rampStat::CRUISING ) {
            // start moving, start creating pulses;
            digitalWrite( stepperDataP->pins[1], (stepperDataP->patternIxInc < 0) ); //set dir pin accordingly
            //delayMicroseconds( 10 );
            startWaveformISR( stepperDataP->pins[0], CYCLETIME, stepperDataP->aCycSteps-CYCLETIME, 0 ); 
        } else {
            // no more moving, switch off enable pin
            digitalWrite( stepperDataP->enablePin, !stepperDataP->enable );
        }
    }
    //GPOC = (1<<2);
}
void ICACHE_RAM_ATTR ISR_Stepper(stepperData_t *stepperDataP) {
    SET_TP1;
    //GPOS = (1<<0);

    // ---------------Stepper motors ---------------------------------------------
    // Usually the ISR is fired at the start of the step pulse
    //CLR_TP1;    // spike for recognizing start of each stepper
    //CLR_TP2;
    if ( stepperDataP->dirChange ) {
        // was falling edge, we have to change the direction.
        digitalWrite( stepperDataP->pins[1], (stepperDataP->patternIxInc < 0) );
        // next ISR at rising edge again
        stepperDataP->dirChange = false;
        attachInterrupt( stepperDataP->pins[0],gpioTab[gpio2ISRx(stepperDataP->pins[0])].gpioISR, RISING );    
    } else {
        // we did a step, update position counter
        stepperDataP->stepsFromZero += stepperDataP->patternIxInc;
        //if ( digitalRead( stepperDataP->pins[1]) ) stepperDataP->stepsFromZero--;
        //else stepperDataP->stepsFromZero++;
        // ------------------ check if last step -----------------------------------
        if ( --stepperDataP->stepCnt == 0 || stepperDataP->speedZero == ZEROSPEEDACTIVE ) {
            // this was the last step to position or speed 0 reached.
            if ( stepperDataP->stepCnt == 0 && stepperDataP->stepCnt2 > 0 ) { // check if we have to start a movement backwards
                // yes, change Direction and go stpCnt2 Steps
                stepperDataP->patternIxInc = -stepperDataP->patternIxInc;
                // change Direction at the end of the pulse
                attachInterrupt( stepperDataP->pins[0],gpioTab[gpio2ISRx(stepperDataP->pins[0])].gpioISR, FALLING ); 
                stepperDataP->dirChange = true;
                stepperDataP->stepCnt = stepperDataP->stepCnt2;
                stepperDataP->stepCnt2 = 0;
				if ( stepperDataP->speedZero == ZEROSPEEDACTIVE ) {
					// Zero-speed has been reached, so dont create steppulses anymore
					stepperDataP->rampState = rampStat::SPEED0;
					// stop creating pulses after the current pulse
					startWaveformISR(stepperDataP->pins[0], CYCLETIME, 10, CYCLETIME );
					if (stepperDataP->enablePin < 254) {
						// enable is active, wait for disabling 
						// timing is done by a pulse on the dir output ( with IRQ at the end )
						startWaveformISR( stepperDataP->pins[1], 1000*stepperDataP->cycDelay, 10 , (1000*stepperDataP->cycDelay)); 
						stepperDataP->delayActiv = true;
					}  
				} else {
					// ramp up in reverse direction
					stepperDataP->rampState = rampStat::RAMPACCEL;
				}
            } else {    
                stepperDataP->stepsInRamp = 0;      // we cannot be in ramp when stopped
                stepperDataP->aCycSteps = 0;    // no more Interrupts for this stepper needed
				if ( stepperDataP->speedZero == ZEROSPEEDACTIVE ) 	stepperDataP->rampState = rampStat::STOPPED;
				else												stepperDataP->rampState = rampStat::SPEED0;
				// stop creating pulses after the current pulse
				startWaveformISR(stepperDataP->pins[0], CYCLETIME, 10, CYCLETIME );
                if (stepperDataP->enablePin < 254) {
                    // enable is active, wait for disabling 
					// timing is done by a pulse on the dir output ( with IRQ at the end )
                    startWaveformISR( stepperDataP->pins[1], 1000*stepperDataP->cycDelay, 10 , (1000*stepperDataP->cycDelay)); 
                    stepperDataP->delayActiv = true;
                }  
                //CLR_TP2;
            }
        }
        // --------------- compute nexte steplength ------------------------------------
        //SET_TP2;
        // ramp state machine
        switch ( stepperDataP->rampState ) {
          case  rampStat::RAMPACCEL:
            //digitalWrite(16,1);
            CLR_TP1;
            // we are accelerating the motor
            if (stepperDataP->stepsInRamp >= stepperDataP->stepRampLen ) {
                // we reached the end of the ramp
                stepperDataP->aCycSteps = stepperDataP->tCycSteps;
                stepperDataP->stepsInRamp = stepperDataP->stepRampLen;
                stepperDataP->rampState = rampStat::CRUISING;
                //digitalWrite(16,0);
            } else {
                stepperDataP->aCycSteps = stepperDataP->cyctXramplen / (stepperDataP->stepsInRamp + RAMPOFFSET) ;//+1;
                // do we have to start deceleration ( remaining steps <= steps in ramp so far )
                // Ramp must be same length in accelerating and decelerating!
                if ( stepperDataP->stepCnt <= ( stepperDataP->stepsInRamp+1  ) ) {
                    //CLR_TP2;
                    stepperDataP->rampState = rampStat::RAMPDECEL;
                    //SET_TP2;
                } else {
                    // still in ramp
                    stepperDataP->stepsInRamp ++;
                }
            }
            SET_TP1;
            break;
          case rampStat::RAMPDECEL:
          case rampStat::SPEEDDECEL:
            SET_TP2;
            if ( stepperDataP->stepsInRamp <= stepperDataP->stepRampLen ) {
                // we are stopping the motor
                if ( stepperDataP->stepCnt > ( stepperDataP->stepsInRamp ) ) {
                    //CLR_TP2; // ToDo: check whether this in necessary ( schould be done in method that changes steps to  move)
                    //steps to move has changed, accelerate again with next step
                    stepperDataP->rampState = rampStat::RAMPACCEL;
                    //DB_PRINT( "scnt=%ld, sIR=%u\n\r", stepperDataP->stepCnt, stepperDataP->stepsInRamp );
                    //SET_TP2;
                }
				// check if we can decrement steps in Ramp
 				if ( stepperDataP->stepsInRamp > 0 ) --stepperDataP->stepsInRamp;  // issue#34
                stepperDataP->aCycSteps = stepperDataP->cyctXramplen / ( stepperDataP->stepsInRamp + RAMPOFFSET ) ;// +1 ;
                CLR_TP2;
                //break;
            } else {
                // lower speed to new value 
                //digitalWrite(16,1);
                if ( (stepperDataP->stepsInRamp-stepperDataP->stepRampLen) > stepperDataP->deltaSteps ) {
                    // steps in ramp still greater than delta
                    stepperDataP->stepsInRamp -=stepperDataP->deltaSteps;
                    stepperDataP->aCycSteps = stepperDataP->cyctXramplen / ( stepperDataP->stepsInRamp + RAMPOFFSET ) ;//+1 ;
					if ( stepperDataP->speedZero == DECELSPEEDZERO ) {
						//target speed was 0, check if we are creating the last step
						if ( (stepperDataP->stepsInRamp-stepperDataP->stepRampLen) <= stepperDataP->deltaSteps ) {
							stepperDataP->speedZero = ZEROSPEEDACTIVE;
							stepperDataP->rampState = rampStat::SPEED0;
						}
					}
                } else {
                    // new targestspeed reached
                    //SET_TP3;
                    stepperDataP->rampState = rampStat::CRUISING;
                    stepperDataP->stepsInRamp =  stepperDataP->stepRampLen;
                    stepperDataP->aCycSteps = stepperDataP->tCycSteps;                //CLR_TP3;
                    //digitalWrite(16,0);
                }
                //ToDo - do we have to stop the motor
            }
            break;
            
          case rampStat::CRUISING:
            // Not in ramp, targetspeed reached - or without ramp at all
            //CLR_TP2;
            stepperDataP->aCycSteps = stepperDataP->tCycSteps;
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
		// set new steplength if there are steps to do
		if ( stepperDataP->aCycSteps > 0 ) {
			startWaveformISR(stepperDataP->pins[0], CYCLETIME, stepperDataP->aCycSteps - CYCLETIME, 0 );
		}
        
    }
     CLR_TP1;
    //GPOC = (1<<0);

}    

uintxx_t MoToStepper::setSpeedSteps( uint32_t speed10, int rampLen ) {
    // Set speed and length of ramp to reach speed ( from stop )
    // neagtive ramplen means it was set automatically
    rampStat newRampState;      // State of acceleration/deceleration
    uint32_t tCycSteps;         // nbr of IRQ cycles per step ( new target value of motorspeed  )
    uint32_t newUstXramplen;      // precompiled  tCycSteps*rampLen*RAMPOFFSET
    uint32_t newRampLen;         // new ramplen
    uint32_t newStepsInRamp;     // new stepcounter in ramp - according to new speed and ramplen
    int32_t newDeltaSteps = 1;  //  only for SPEEDDECEL
    uint32_t newSpeed10;        // new target speed
    //digitalWrite( 16,HIGH );
    if ( _stepperData.output == NO_OUTPUT ) return 0; // --------------->>>>>>>>>>>>>>>>not attached
    DB_PRINT(">>>>>>>>>>sSS(%u,%d)", speed10, rampLen );
    // compute new speed values, adjust length of ramp if necessary
    //actSpeed10 = oldSpeed10 = _stepSpeed10;
        
    newRampLen = abs(rampLen);    // negative values are invalid ( indicate automatically adjusted length )
    if (rampLen<0) newRampLen--;
    if (newRampLen > MAXRAMPLEN ) newRampLen = MAXRAMPLEN;
    newSpeed10 = min( (uint32_t)1000000 / MIN_STEPTIME * 10, (uint32_t)speed10 );
	if ( newSpeed10 == 0) {
		if ( _chkRunning() ) { 
			// stepper is moving, we hahe to stop it
			if ( _stepperData.stepRampLen > 0) {
				_stepperData.speedZero = DECELSPEEDZERO;	// there is a ramp, flag for ramping down to speed 0
			} else {
				// no ramp, simply inhibit creating pulses, disable stepper if needed.
				_stepperData.speedZero = ZEROSPEEDACTIVE;				
			}
		} else { 
		  // Stepper doesn't move, only inhibit starting
			_stepperData.speedZero = ZEROSPEEDACTIVE;	// set stepper inactive ( no ISR action )
			_stepperData.rampState = rampStat::SPEED0;
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
	_stepIRQ();
    
    //DB_PRINT( "rampLen-new=%u, ramplenParam=%u", newRampLen, rampLen );
    // compute target steplength and check whether speed and ramp fit together: 
    tCycSteps = ( 1000000L * 10  / newSpeed10 );
    // tcyc * (rapmlen+RAMPOFFSET) must be less then 65000, otherwise ramplen is adjusted accordingly
    newUstXramplen = tCycSteps * ( newRampLen + RAMPOFFSET );

    if (rampLen >= 0) {
        // ramplength was set by user, update reference-values
        _lastRampSpeed = newSpeed10;
        _lastRampLen   = newRampLen;
    }
    DB_PRINT( "rampLen-new=%u, ramplenParam=%d, _lastrampSpeed=%d, _lastRmapLen=%d", newRampLen, rampLen, _lastRampSpeed, _lastRampLen );
    
    // recompute all relevant rampvalues according to actual speed and ramplength
    // This needs to be done only, if a ramp is defined, the stepper is moving
    // and the speed an ramp values changed
    // In all other cases the new speed/ramp values will get active immediately
    DB_PRINT( "actRampLen=%u, cXr-new=%u, xCr-old=%u", newRampLen, newUstXramplen, _stepperData.cyctXramplen );
    _noStepIRQ(); SET_TP2;
    if ( (_stepperData.stepRampLen + newRampLen ) != 0
        && _chkRunning() 
        &&  newUstXramplen != _stepperData.cyctXramplen ) {
        // local variables to hold data that might change in IRQ:
        // If there was a step during recomputing the rampvalues, we must recompute again
        // recomputing the rampvalues lasts too long to stop the IRQ during the whole time
        uint32_t        __stepCnt;
        uint32_t        __newStepCnt;
        uint32_t        __newStepCnt2;
        
        DB_PRINT("Speed changed! New: tUs=%u, ramp=%u, cXr=%u",tCycSteps,newRampLen,newUstXramplen );
        //Serial.print(_stepperData.rampState); Serial.print(" ( ");Serial.print( _stepperData.stepsInRamp );Serial.print("->");
         do {
            // read actual ISR values
            newRampState = _stepperData.rampState;
            __stepCnt       =  _stepperData.stepCnt; 
            __newStepCnt    = 0;    // if stepcnt is to be changed
            __newStepCnt2   = 0;
            _stepIRQ(); CLR_TP2;
            //with ramp and ramp or speed changed 
            // compute new 'steps in Ramp' according to new speed and ramp values. This maybe greater
            // than ramplen, if speed changed to slower
            newStepsInRamp = ( (int64_t)newUstXramplen * (_stepperData.stepsInRamp + RAMPOFFSET ) / _stepperData.cyctXramplen );
            newStepsInRamp = newStepsInRamp<RAMPOFFSET? 0 : newStepsInRamp-RAMPOFFSET;
            
            if ( newSpeed10 != _stepSpeed10 ) {
                // speed changed!
                if ( newStepsInRamp > newRampLen ) {
                    //  ==========  we are too fast ============================
                        //Serial.print(" --");
                        //DB_PRINT ( "Slower: %u/%u -> %u/%u", _stepSpeed10,_stepperData.stepRampLen,  newSpeed10, newRampLen );
                        newRampState = rampStat::SPEEDDECEL;
                        if ( _stepperData.stepsInRamp == 0 ) {
                            newDeltaSteps = ((10L * newStepsInRamp)+5 )/10L;
                        } else {
                            newDeltaSteps = ((10L * newStepsInRamp / _stepperData.stepsInRamp)+5 )/10L;
                        }
                        if ( newDeltaSteps < 1 ) newDeltaSteps = 1;
                    
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
            if ( newStepsInRamp > (__stepCnt - _stepperData.stepCnt2) ) {
                // we cannot reach the tagetposition, so we go beyond the targetposition and than back.
                // This works even if we are already beyond the target position
                //Serial.print( " ><");
                __newStepCnt2 = newStepsInRamp - (__stepCnt - _stepperData.stepCnt2);
                __newStepCnt = newStepsInRamp;
                newRampState = rampStat::RAMPDECEL;
            }
            _noStepIRQ(); SET_TP2;
            //Serial.print(" ) ");Serial.print(_stepperData.rampState);
        } while ( __stepCnt != _stepperData.stepCnt ); // if there was a step during computing, do it again
        _stepperData.rampState = newRampState;
        _stepperData.stepsInRamp = newStepsInRamp;
        _stepperData.deltaSteps = newDeltaSteps;
        if ( __newStepCnt != 0 ) {
            _stepperData.stepCnt = __newStepCnt;
            _stepperData.stepCnt2 = __newStepCnt2;
        }
    }
    _stepperData.tCycSteps = tCycSteps;
    _stepperData.cyctXramplen = newUstXramplen;
    _stepperData.stepRampLen = newRampLen;
    _stepIRQ(); CLR_TP2;
	DB_PRINT( "_sSp10=%d, newSpd=%d speed10=%d",  _stepSpeed10, newSpeed10, speed10 );
    if ( _stepSpeed10 == 0 && speed10 > 0 && _stepperData.stepCnt > 0) {
		// stepper was stopped by speed - start again if there are still steps to do
		if ( newRampLen > 0 ) {
			// starting with ramp
			_stepperData.rampState = rampStat::RAMPACCEL;
		} else {
			// starting without ramp
			_stepperData.rampState = rampStat::CRUISING;
		}
        // check if enable Pin must be activated
        if ( _stepperData.enableOn ) {
			DB_PRINT("sss starting, delayActive=%d", _stepperData.delayActiv );
            if ( !_stepperData.delayActiv ) {
                // enable must be set and delaytime is not yet running
                digitalWrite( _stepperData.enablePin, _stepperData.enable );
                // create a singlepulse on dir-output to measure delaytime
                //delayMicroseconds( 10 );
                startWaveformMoTo( _stepperData.pins[1], 1000*_stepperData.cycDelay, 10000 , 1000*_stepperData.cycDelay); 
                _stepperData.delayActiv = true;
            }
        } else {
            // no enable control, start movement directly
            // set dir output
			DB_PRINT("sss starting enable , delayActive=%d", _stepperData.delayActiv );
            digitalWrite( _stepperData.pins[1], (_stepperData.patternIxInc < 0) );
            startWaveformMoTo( _stepperData.pins[0], CYCLETIME, _stepperData.aCycSteps-CYCLETIME, 0 ); 
        }
    } else {
        // direction may have changed, so set it here
		DB_PRINT("sss direction changed?" );
        noInterrupts(); // with ramp direction may change in ISR
        digitalWrite( _stepperData.pins[1], (_stepperData.patternIxInc < 0) );
        interrupts();
    }
    _stepSpeed10 = speed10 == 0? 0 : newSpeed10;
    //digitalWrite(16,LOW );
    DB_PRINT("nSiR=%d, nDs=%d, _Spd10=%d",newStepsInRamp, newDeltaSteps, _stepSpeed10);
    prDynData();
    return _stepperData.stepRampLen;
}
 
void MoToStepper::prDynData() {             // print actual Stepperdata
 #if 0//def debugPrint
    stepperData_t printData;  // vor Ausdruck eine lokale Kopie erstellen
    noInterrupts();
    printData.stepCnt   = _stepperData.stepCnt;             // nmbr of steps to take
    printData.stepCnt2  = _stepperData.stepCnt2;            // nmbr of steps to take after automatic reverse
    printData.tCycSteps  = _stepperData.tCycSteps;            // µseconds per step ( target value of motorspeed  )
    printData.aCycSteps  = _stepperData.aCycSteps;            // nµseconds per step ( actual motorspeed  )
    printData.cyctXramplen = _stepperData.cyctXramplen;       // precompiled  tCycSteps*(rampLen+RAMPOFFSET)
    printData.cycDelay   = _stepperData.cycDelay;             // delay time: enable -> stepping
    printData.dirChange = _stepperData.dirChange;           // Flag: Dir has to be changed ( at falling edge )
    printData.stepRampLen = _stepperData.stepRampLen;       // Length of ramp in steps
    printData.stepsInRamp = _stepperData.stepsInRamp;       // stepcounter within ramp ( counting from stop ( = 0 ): incrementing in startramp, decrementing in stopramp
    printData.rampState = _stepperData.rampState;           // State of acceleration/deceleration
    printData.stepsFromZero = _stepperData.stepsFromZero;   // distance from last reference point 
    printData.deltaSteps = _stepperData.deltaSteps;         // number of computed steps per real step in SPEEDDECEL

    interrupts();
    Serial.printf("stepCnt=%5d\t stepCnt2=%5d\t sFZ=%5d\n\r", printData.stepCnt, printData.stepCnt2, printData.stepsFromZero );
    Serial.printf("tCycSteps=%5d\t aCycSteps=%5d\t XrampL=%5d\n\r", printData.tCycSteps,printData.aCycSteps,printData.cyctXramplen);
    Serial.printf("rampLen=%4d\t stepsInRamp=%4d\t, rampState=%s(%d)\n\r",printData.stepRampLen,printData.stepsInRamp,rsC[(int)printData.rampState],printData.rampState);
    Serial.printf("dltaStp=%4d\n\r", printData.deltaSteps );
    Serial.println("^^^^^^^^^^^^^^ISR-Data^^^^^^^^^^^^^^^^");
    #endif
}

