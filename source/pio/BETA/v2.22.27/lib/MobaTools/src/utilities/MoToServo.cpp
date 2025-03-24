/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2020 All rights reserved.

  Functions for the servo part of MobaTools
*/
#define COMPILING_MOTOSERVO_CPP  // this allows servo-specific defines in includefiles

//#define debugTP
//#define debugPrint
#include <MobaTools.h>

// Global Data for all instances and classes  --------------------------------
// Variables for servos
static byte servoCount = 0;


#ifdef IS_ESP //------------------- Servo Interrupt für ESP8266 und ESP32 ----------------------
static bool speedV08 = false;    // Compatibility-Flag for speed method

/////////////////////////////  Pulse-interrupt for ESP8266 and ESP 32  /////////////////////////////////////////
// This ISR is fired at the falling edge of the servo pulse. It is specific to every servo Objekt and
// computes the length of the next pulse. The pulse itself is created by the core_esp8266_waveform routines or by ledPWM HW ( ESP32 )
void IRAM_ATTR ISR_Servo( void *arg ) {
    servoData_t *_servoData = static_cast<servoData_t *>(arg);
	// On ESP32 the IRQ fires at start AND end of the pulse, ignore leading edge
	if ( digitalRead( _servoData->pin) == HIGH ) return ;
    portENTER_CRITICAL_ISR(&servoMux);
    SET_TP2;
    if ( _servoData->ist != _servoData->soll ) {
        //SET_TP1;
        _servoData->offcnt = 50;
        if ( _servoData->ist > _servoData->soll ) {
            _servoData->ist -= _servoData->inc;
            if ( _servoData->ist < _servoData->soll ) _servoData->ist = _servoData->soll;
        } else {
            _servoData->ist += _servoData->inc;
            if ( _servoData->ist > _servoData->soll ) _servoData->ist = _servoData->soll;
        }
        //CLR_TP1;
        //Serial.println(_servoData->ist );
            servoWrite( _servoData, _servoData->ist/INC_PER_TIC ); 
        //SET_TP1;
        //CLR_TP1;
    } else if ( !_servoData->noAutoff ) { // no change in pulse length, look for autooff
        if ( --_servoData->offcnt == 0 ) {
            SET_TP3;
            servoPulseOff( _servoData );
            //CLR_TP3;
        }
    }
    portEXIT_CRITICAL_ISR(&servoMux);
    CLR_TP2;
}

#elif defined RP2040 //---------------- for RP2040/RP2350 processor ----------------------------
  //#pragma message "Servo for RP2040"
  static servoData_t* lastServoDataP = NULL; //start of ServoData-chain
  static bool speedV08 = false;    // Compatibility-Flag for speed method
  void __not_in_flash_func(servoISR)(void) {
  SET_TP1;
  // IRQ fired when a PWM counter wraps
  servoData_t *sPtr = lastServoDataP;
  int thisSlice = 0;
  while ( sPtr != NULL ) {
    // Cycle through all servos
    if ( pwm_get_irq_status_mask() & 1 << (sPtr->pwmNbr >> 1) ) {
      CLR_TP1;
      thisSlice = sPtr->pwmNbr >> 1;
      // got one for this slice
      if ( sPtr->ist != sPtr->soll ) {
        sPtr->offcnt = 50;
        // servo is not at target position
        if ( sPtr->ist > sPtr->soll ) {
          sPtr->ist -= sPtr->inc;
          if ( sPtr->ist < sPtr->soll ) sPtr->ist = sPtr->soll; // if it was overshoot;
        } else {
          sPtr->ist += sPtr->inc;
          if ( sPtr->ist > sPtr->soll ) sPtr->ist = sPtr->soll; // if it was overshoot;
        }
        // set new duty
        pwm_set_chan_level(sPtr->pwmNbr >> 1, sPtr->pwmNbr & 1, tic2time(sPtr->ist));
      } else { // no change in pulse length, look for autooff
		if ( sPtr->offcnt == 0 ) { 
		  // is autooff and off time has elapsed
          pwm_set_chan_level(sPtr->pwmNbr >> 1, sPtr->pwmNbr & 1, 0);
		} else {
			// still create the pulse
			if ( !sPtr->noAutoff ) --sPtr->offcnt;
            pwm_set_chan_level(sPtr->pwmNbr >> 1, sPtr->pwmNbr & 1, tic2time(sPtr->ist));
		}
      } 
      SET_TP1;
    }
    // try next ( there maybe 2 servos(channels) for this IRQ
    sPtr = sPtr->prevServoDataP;
  }
  pwm_clear_irq (thisSlice);
  CLR_TP1;
}



#else //---------------------- Timer-interrupt for non ESP / non RP2040 -----------------------------
static servoData_t* lastServoDataP = NULL; //start of ServoData-chain
static servoData_t* pulseP = NULL;         // pulse Ptr in IRQ
static servoData_t* activePulseP = NULL;   // Ptr to pulse to stop
static servoData_t* stopPulseP = NULL;     // Ptr to Pulse whose stop time is already in OCR1
static servoData_t* nextPulseP = NULL;
static enum { PON, POFF } IrqType = PON; // Cycle starts with 'pulse on'
static uint16_t activePulseOff = 0;     // OCR-value of pulse end 
static uint16_t nextPulseLength = 0;
static bool speedV08 = true;    // Compatibility-Flag for speed method
// create overlapping servo pulses
// Positions of servopulses within 20ms cycle are variable, max 2 pulses at the same time
// 27.9.15 with variable overlap, depending on length of next pulse: 16 Servos
// 2.1.16 Enable interrupts after timecritical path (e.g. starting/stopping servo pulses)
//        so other timecritical tasks can interrupt (nested interrupts)
// 6.6.19 Because stepper IRQ now can last very long, it is disabled during servo IRQ
static bool searchNextPulse() {
    //SET_TP4;
    while ( pulseP != NULL && pulseP->soll < 0 ) {
        //SET_TP4;
        pulseP = pulseP->prevServoDataP;
        //CLR_TP4;
    }
    //CLR_TP2;
    if ( pulseP == NULL ) {
        // there is no more pulse to start, we reached the end
        //CLR_TP4;
        return false;
    } else { // found pulse to output
        //SET_TP2;
        if ( pulseP->ist == pulseP->soll ) {
            // no change of pulselength
            if ( pulseP->offcnt > 0 ) pulseP->offcnt--;
        } else if ( pulseP->ist < pulseP->soll ) {
            pulseP->offcnt = OFF_COUNT;
            if ( pulseP->ist < 0 ) pulseP->ist = pulseP->soll; // first position after attach
            else pulseP->ist += pulseP->inc;
            if ( pulseP->ist > pulseP->soll ) pulseP->ist = pulseP->soll;
        } else {
            pulseP->offcnt = OFF_COUNT;
            pulseP->ist -= pulseP->inc;
            if ( pulseP->ist < pulseP->soll ) pulseP->ist = pulseP->soll;
        } 
        //CLR_TP4;
        return true;
    } 
} //end of 'searchNextPulse'

// ---------- OCRxA Compare Interrupt used for servo motor (overlapping pulses) ----------------
// not for ESP processors
#if defined ( ARDUINO_ARCH_AVR ) 
ISR ( TIMERx_COMPA_vect) {
#elif defined  (ARDUINO_ARCH_MEGAAVR )
ISR (TCA0_CMP0_vect) {
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;	// Reset IRQ-flag
#else // STM32Fx and Renesas RA4M1
void ISR_Servo( void) {
    uint16_t OCRxA = 0;
#endif
    SET_TP2;
    CLR_TP4;
    // Timer1 Compare A, used for servo motor
    if ( IrqType == POFF ) { // Pulse OFF time
        //SET_TP2; // Oszimessung Dauer der ISR-Routine OFF
        //SET_TP3; // Oszimessung Dauer der ISR-Routine
        IrqType = PON ; // it's (nearly) always alternating
        // switch off previous started pulse
        #ifdef FAST_PORTWRT
        *stopPulseP->portAdr &= ~stopPulseP->bitMask;
        #else
        digitalWrite( stopPulseP->pin, LOW );
        #endif
        if ( nextPulseLength > 0 ) {
            // there is a next pulse to start, compute starttime 
            // set OCR value to next starttime ( = endtime of running pulse -overlap )
            // next starttime must behind actual timervalue and endtime of next pulse must
            // lay after endtime of runningpuls + safetymargin (it may be necessary to start
            // another pulse between these 2 ends)
            long tmpTCNT1 = GET_COUNT + MARGINTICS/2;
            //CLR_TP3 ;
            OCRxA = max ( (long)((long)activePulseOff + (long) MARGINTICS - (long) nextPulseLength), tmpTCNT1 );
        } else {
            // we are at the end, no need to start another pulse in this cycle
            if ( activePulseOff ) {
                // there is still a running pulse to stop
                //SET_TP1; // Oszimessung Dauer der ISR-Routine
                OCRxA = activePulseOff;
                IrqType = POFF;
                stopPulseP = activePulseP;
                activePulseOff = 0;
                //CLR_TP1; // Oszimessung Dauer der ISR-Routine
            } else { // was last pulse, start over
                pulseP = lastServoDataP;
                nextPulseLength = 0;
                OCRxA = FIRST_PULSE;
            }
        }
        //CLR_TP2; // Oszimessung Dauer der ISR-Routine OFF
    } else { // Pulse ON - time
        //SET_TP2; // Oszimessung Dauer der ISR-Routine ON
        //if ( pulseP == lastServoDataP ) SET_TP3;
        // look for next pulse to start
        // do we know the next pulse already?
        if ( nextPulseLength > 0 ) {
            // yes we know, start this pulse and then look for next one
            uint16_t tmpTCNT1= GET_COUNT-4; // compensate for computing time
            if ( nextPulseP->on && (nextPulseP->offcnt+nextPulseP->noAutoff) > 0 ) {
                // its a 'real' pulse, set output pin
                //CLR_TP1;
                #ifdef FAST_PORTWRT
                *nextPulseP->portAdr |= nextPulseP->bitMask;
                #else
                digitalWrite( nextPulseP->pin, HIGH );
                #endif
            }
            //SET_TP3;
            // the 'nextPulse' we have started now, is from now on the 'activePulse', the running activPulse is now the
            // pulse to stop next.
            stopPulseP = activePulseP; // because there was a 'nextPulse' there is also an 'activPulse' which is the next to stop
            OCRxA = activePulseOff;
            activePulseP = nextPulseP;
            activePulseOff = activePulseP->ist/INC_PER_TIC + tmpTCNT1; // end of actually started pulse
            nextPulseLength = 0;
            //SET_TP1;
        }
        if ( searchNextPulse() ) {
            // found a pulse
            if ( activePulseOff == 0 ) {
                // it is the first pulse in the sequence, start it
                //TOG_TP2;
                SET_TP3;
                activePulseP = pulseP; 
                activePulseOff = pulseP->ist/INC_PER_TIC + GET_COUNT - 4; // compensate for computing time
                if ( pulseP->on && (pulseP->offcnt+pulseP->noAutoff) > 0 ) {
                    // its a 'real' pulse, set output pin
                    #ifdef FAST_PORTWRT
                    *pulseP->portAdr |= pulseP->bitMask;
                    #else
                    digitalWrite( pulseP->pin, HIGH );
                    #endif
                }
                int32_t tmpTCNT1 = GET_COUNT+ MARGINTICS/2;
                //SET_TP3;
                // look for second pulse
                //SET_TP4;
                pulseP = pulseP->prevServoDataP;
                //CLR_TP4;
                if ( searchNextPulse() ) {
                    // there is a second pulse - this is the 'nextPulse'
                    nextPulseLength = pulseP->ist/INC_PER_TIC;
                    nextPulseP = pulseP;
                    //SET_TP4;
                    pulseP = pulseP->prevServoDataP;
                    //CLR_TP4;
                    // set Starttime for 2. pulse in sequence
                    OCRxA = max ( (long)((long)activePulseOff + (long) MARGINTICS - (long) nextPulseLength), tmpTCNT1  );
                } else {
                    // no next pulse, there is only one pulse
                    OCRxA = activePulseOff;
                    activePulseOff = 0;
                    stopPulseP = activePulseP;
                    IrqType = POFF;
                }
                CLR_TP3;
            } else {
                // its a pulse in sequence, so this is the 'nextPulse'
                nextPulseLength = pulseP->ist/INC_PER_TIC;
                nextPulseP = pulseP;
                //SET_TP4;
                pulseP = pulseP->prevServoDataP;
                //CLR_TP4;
                IrqType = POFF;
            }
        } else {
            // found no pulse, so the last one is running or no pulse at all
            
            if ( activePulseOff == 0 ) {
                // there wasn't any pulse, restart
                pulseP = lastServoDataP;
                nextPulseLength = 0;
                OCRxA = FIRST_PULSE;
            } else {
                // is last pulse, don't start a new one
                IrqType = POFF;
            }
        }
        //CLR_TP2; CLR_TP3; // Oszimessung Dauer der ISR-Routine ON
    } //end of 'pulse ON'
	setServoCmpAS(OCRxA);	// set Servo-compare register
    //CLR_TP1; CLR_TP3; // Oszimessung Dauer der ISR-Routine
    CLR_TP2;
}

#endif // not ESP or RP2040
// ------------ end of Interruptroutines ------------------------------
///////////////////////////////////////////////////////////////////////////////////
// --------- Class MoToServo ---------------------------------
// Class-specific Variables

const byte NO_ANGLE = 0xff;
const byte NO_PIN = 0xff;
const int8_t NOT_ATTACHED = -1;

MoToServo::MoToServo() //: _servoData.pin(NO_PIN),_angle(NO_ANGLE),_min16(1000/16),_max16(2000/16)
{   _servoData.servoIx = servoCount++;
    _servoData.soll = -1;    // = not initialized
    _servoData.pin = NO_PIN;
    _servoData.pwmNbr = NOT_ATTACHED;
    _minPw = MINPULSEWIDTH ;
    _maxPw = MAXPULSEWIDTH ;
    #ifndef IS_ESP // there is no servochain on ESP
    noInterrupts(); // Add to servo-chain
    _servoData.prevServoDataP = lastServoDataP;
    lastServoDataP = &_servoData;
    interrupts();
    #endif
    MODE_TP1;   // set debug-pins to Output
    MODE_TP2;
    MODE_TP3;
    MODE_TP4;
}

void MoToServo::setMinimumPulse(uint16_t t)
{   _minPw = constrain( t, MINPULSEWIDTH,MAXPULSEWIDTH);
}

void MoToServo::setMaximumPulse(uint16_t t)
{   _maxPw = constrain( t, MINPULSEWIDTH,MAXPULSEWIDTH);
}


uint8_t MoToServo::attach(int pinArg) {
    return attach( pinArg, MINPULSEWIDTH, MAXPULSEWIDTH, false );
}
uint8_t MoToServo::attach(int pinArg, bool autoOff ) {
    return attach( pinArg, MINPULSEWIDTH, MAXPULSEWIDTH, autoOff );
}
uint8_t MoToServo::attach(int pinArg, uint16_t pmin, uint16_t pmax ) {
    return attach( pinArg, pmin, pmax, false );
}

uint8_t MoToServo::attach( int pinArg, uint16_t pmin, uint16_t pmax, bool autoOff ) {
    // return false if already attached or too many servos
    DB_PRINT("Servoattach: pwmNbr=%d, servoIx=%d, Pin=%d", _servoData.pwmNbr, _servoData.servoIx, pinArg );
    if ( _servoData.pwmNbr >= 0  ||  _servoData.servoIx >= MAX_SERVOS ) return 0;
    #ifdef ESP8266 // check pinnumber
        if ( pinArg <0 || pinArg >15 || gpioUsed(pinArg ) ) return 0;
        setGpio(pinArg);    // mark pin as used
    #endif   
    // set pulselength for angle 0 and 180
    _minPw = constrain( pmin, MINPULSEWIDTH, MAXPULSEWIDTH );
    _maxPw = constrain( pmax, MINPULSEWIDTH, MAXPULSEWIDTH );
	DB_PRINT( "pin: %d, pmin:%d pmax%d autoOff=%d", pinArg, pmin, pmax, autoOff);
    
    // intialize objectspecific data
    _lastPos = 1500*TICS_PER_MICROSECOND*INC_PER_TIC ;    // initalize to middle position
    _servoData.soll = -1;  // invalid position -> no pulse output
    _servoData.ist = -1;   
    _servoData.inc = 8000;  // means immediate movement
    _servoData.pin = pinArg;
    _servoData.on = false;  // create no pulses until next write
    _servoData.noAutoff = autoOff?0:1 ;  
    #ifdef FAST_PORTWRT
    // compute portaddress and bitmask related to pin number
    _servoData.portAdr = portOutputRegister(digitalPinToPort(pinArg));
    _servoData.bitMask = digitalPinToBitMask(pinArg);
    DB_PRINT( "Idx: %d Portadr: 0x%x, Bitmsk: 0x%x", _servoData.servoIx, _servoData.portAdr, _servoData.bitMask );
	#endif
    pinMode (_servoData.pin,OUTPUT);
    digitalWrite( _servoData.pin,LOW);
    _servoData.pwmNbr = 0; // >= 0 means 'successfully attached' on all architectures ( maybe ocerriten later in this function )
    #ifdef ESP8266
        // assign an ISR to the pin
        gpioTab[gpio2ISRx(_servoData.pin)].MoToISR = (void (*)(void*))ISR_Servo;
        gpioTab[gpio2ISRx(_servoData.pin)].IsrData = &_servoData;
        attachInterrupt( _servoData.pin, gpioTab[gpio2ISRx(_servoData.pin)].gpioISR, FALLING );
    #elif defined HAS_PWM_HW // ( ESP32 and RP2040/RP3250 so far )
        // pwmNbr will be negative if there are no free pwm channels
        _servoData.pwmNbr =  servoPwmSetup( &_servoData );
        DB_PRINT("pwmNbr=%d, Pin=%d", _servoData.pwmNbr, _servoData.pin );
        
    #else // create servo pulses in timer ISR
        seizeTimerAS();
        // initialize servochain pointer and ISR if not done already
        noInterrupts();
        if ( pulseP == NULL ) {
            pulseP = lastServoDataP;
            enableServoIsrAS();
        }
         interrupts();
    #endif // no ESP8266
    DB_PRINT("OVLMARGIN=%d, OVL_TICS=%d, MARGINTICS=%d, SPEEDRES=%d, TPM4=%d", (int16_t) OVLMARGIN, (int16_t)OVL_TICS, (int16_t)MARGINTICS, (int16_t)INC_PER_TIC, (int)(TICS_PER_MICROSECOND*4) );
    //return ( _servoData.pwmNbr >= 0 );
    return ( _servoData.pwmNbr +1 );
}

void MoToServo::detach()
{
    if ( _servoData.pwmNbr == NOT_ATTACHED ) return; // only if servo is attached
    byte tPin = _servoData.pin;
    while( digitalRead( _servoData.pin ) ); // don't detach during an active pulse
    noInterrupts();
    _servoData.on = false;  
    _servoData.soll = -1;  
    _servoData.ist = -1;  
    interrupts();
    #ifdef ESP8266
        stopWaveformMoTo(tPin); //stop creating pulses
        clrGpio(tPin);
        detachInterrupt( tPin );
    #endif
    #ifdef HAS_PWM_HW
        servoDetach( &_servoData );
    #endif
    pinMode( tPin, INPUT );
    _servoData.pwmNbr = NOT_ATTACHED;  
    _servoData.pin = NO_PIN;  
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"  // because of startPulse in NON esp8266 use
void MoToServo::write(uint16_t angleArg)
{   // set position to move to
    // values between 0 and 180 are interpreted as degrees,
    // values between MINPULSEWIDTH and MAXPULSEWIDTH are interpreted as microseconds
    static int newpos;
    bool startPulse = false;    // only for esp8266
    //SET_TP1;
    #ifdef ARDUINO_ARCH_AVR
        //DB_PRINT( "Write: angleArg=%d, Soll=%d, OCR=%u", angleArg, _servoData.soll, OCRxA );
    #endif
    if ( _servoData.pwmNbr != NOT_ATTACHED ) { // only if servo is attached
        //Serial.print( "Pin:" );Serial.print (_servoData.pin);Serial.print("Wert:");Serial.println(angleArg);
        #ifdef ARDUINO_ARCH_AVR
		//DB_PRINT( "Stack=0x%04x, &sIx=0x%04x", ((SPH&0x7)<<8)|SPL, &_servoData.servoIx );
        #endif
        if ( angleArg <= 255) {
            // pulse width as degrees (byte values are always degrees) 09-02-2017
            angleArg = min( 180,(int)angleArg);
            newpos = time2tic( map( angleArg, 0,180, _minPw, _maxPw ) );
        } else {
            // pulsewidth as microseconds
            newpos = time2tic( constrain( angleArg, _minPw, _maxPw ) );
            
        }
        if ( _servoData.soll < 0 ) {
            // Serial.println( "first write");
            // this is the first pulse to be created after attach
            _servoData.on = true;
            startPulse = true;      // only for esp8266
            _lastPos = newpos;
            noInterrupts();
            _servoData.soll= newpos ; 
            _servoData.ist= newpos ; // .ist =.soll  -> will jump to .soll immediately
            interrupts();
            
        }
        else if ( newpos != _servoData.soll ) {
            // position has changed, store old position, set new position
            _lastPos = _servoData.soll;
            noInterrupts();
            _servoData.soll= newpos ;
            interrupts();
        }
        #ifdef IS_ESP // start creating pulses?
            if ( (startPulse) || (_servoData.offcnt+_servoData.noAutoff) == 0  ) {
                SET_TP3;
                // first pulse after attach, or pulses have been switch off by autoff
                startServoPulse( &_servoData, _servoData.ist/INC_PER_TIC);
                DB_PRINT( "start pulses at pin %d, ist=%d, soll=%d", _servoData.pin, _servoData.ist, _servoData.soll );
                CLR_TP3;
            }
        #endif
        _servoData.offcnt = OFF_COUNT;   // auf jeden Fall wieder Pulse ausgeben
    }
    //DB_PRINT( "Soll=%d, Ist=%d, Ix=%d, inc=%d, SR=%d, Duty100=%d, LEDC_BITS=%d", _servoData.soll,_servoData.ist, _servoData.servoIx, _servoData.inc, INC_PER_TIC, DUTY100, LEDC_BITS );
    DB_PRINT( "Soll=%d, Ist=%d, Ix=%d, inc=%d, SR=%d", _servoData.soll,_servoData.ist, _servoData.servoIx, _servoData.inc, (int)INC_PER_TIC );
	DB_PRINT( "t2tic=%d, tic2t=%d", (int)time2tic(map( angleArg, 0,180, _minPw, _maxPw)), (int)tic2time(_servoData.soll ) );
    //delay(2);
    //CLR_TP1;
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void MoToServo::setSpeedTime(uint16_t minMaxTime ) {
	// Set speed as time (in milliseconds) needed when moving from 0° ... 180°
	uint16_t maxTics = 8* ( _maxPw - _minPw );	//	tics are counted in 0.125 µs	
	uint16_t speedCycles = minMaxTime / 20;	// Nbr of pulses needed from 0° to 180°
	if ( speedCycles == 0 ) speedCycles = 1;	// Avoid divide by zero
	uint16_t speedTics = maxTics / speedCycles;
	setSpeed( speedTics, HIGHRES );	// no compatibility mode, when new speed method is used
	DB_PRINT(" IPM=%d, TPM=%d", INC_PER_MICROSECOND, TICS_PER_MICROSECOND );
}
	
	
	
void MoToServo::setSpeed( int speed, bool compatibility ) {
    // set global compatibility-Flag
    #ifndef IS_ESP
    speedV08 = compatibility;   // not on ESP8266/ESP32
    #endif
    setSpeed( speed );
}
#pragma GCC diagnostic pop

void MoToServo::setSpeed( int speed ) {
    // Set increment value for movement to new angle
    // 'speed' is 0,125µs increment per 20ms
    if ( _servoData.pwmNbr != NOT_ATTACHED ) { // only if servo is attached
        if ( speedV08 ) speed *= COMPAT_FACT;
        speed = constrain(  speed, 0, 8000 );  // 8000 means immediate movement, greater values make no sense
                                        // Greater Values will also lead to an overflow on ESP32
        noInterrupts();
        if ( speed == 0 )
            _servoData.inc = AS_Speed2Inc(8000);  // means immediate movement
        else
            _servoData.inc = AS_Speed2Inc(speed);
        interrupts();
    }
}

uint8_t MoToServo::read() {
    // get position in degrees
    int offset;
    if ( _servoData.pwmNbr == NOT_ATTACHED ) return -1; // Servo not attached
    offset = (_maxPw - _minPw)/180/2;
    return map( readMicroseconds() + offset, _minPw, _maxPw, 0, 180 );
}

uint16_t MoToServo::readMicroseconds() {
    // get position in microseconds
    int value;
    if ( _servoData.pwmNbr == NOT_ATTACHED ) return -1; // Servo not attached
    noInterrupts();
    value = _servoData.ist;
    interrupts();
    if ( value < 0 ) value = _servoData.soll; // there is no valid actual vlaue
    //DB_PRINT( "Ist=%d, Soll=%d, TpM=%d, SR=%d", value, _servoData.soll, TICS_PER_MICROSECOND, INC_PER_TIC );
    return tic2time( value );   
}

uint8_t MoToServo::moving() {
    // return how much still to move (percentage)
    if ( _servoData.pwmNbr == NOT_ATTACHED ) return 0; // Servo not attached
    long total , remaining;
    total = abs( _lastPos - _servoData.soll );
    noInterrupts(); // disable interrupt, because integer _servoData.ist is changed in interrupt
    remaining = abs( _servoData.soll - _servoData.ist );
    interrupts();  // allow interrupts again
    if ( remaining == 0 ) return 0;
    return ( remaining * 100 ) /  total +1;
}

uint8_t MoToServo::attached()
{
    return ( _servoData.pwmNbr != NOT_ATTACHED );
}


