// AVR HW-spcific Functions
#ifdef ARDUINO_ARCH_AVR
#include <MobaTools.h>
#define debugTP
//#define debugPrint
#include <utilities/MoToDbg.h>

//#warning "HW specfic - avr ---"

uint8_t noStepISR_Cnt;   // Counter for nested StepISr-disable

//void ISR_Stepper(void);     // defined in MoToISR.cpp
nextCycle_t nextCycle;
static nextCycle_t cyclesLastIRQ = 1;  // cycles since last IRQ
// ---------- OCRxB Compare Interrupt used for stepper motor and Softleds ----------------
void stepperISR(uint8_t cyclesLastIRQ) __attribute__ ((weak));
void softledISR(uint8_t cyclesLastIRQ) __attribute__ ((weak));
ISR ( TIMERx_COMPB_vect) {
    uint16_t tmp;
  // Timer1 Compare B, used for stepper motor, starts every CYCLETIME us
    // 26-09-15 An Interrupt is only created at timeslices, where data is to output
    SET_TP1;
    nextCycle = ISR_IDLETIME  / CYCLETIME ;// min ist one cycle per IDLETIME
    if ( stepperISR ) stepperISR(cyclesLastIRQ);
    //============  End of steppermotor ======================================
   if ( softledISR ) softledISR(cyclesLastIRQ);
    // ======================= end of softleds =====================================
    // set compareregister to next interrupt time;
    // compute next IRQ-Time in us, not in tics, so we don't need long
    noInterrupts(); // when manipulating 16bit Timerregisters IRQ must be disabled
    if ( nextCycle == 1 )  {
        //CLR_TP1;
        // this is timecritical: Was the ISR running longer then CYCELTIME?
        // compute length of current IRQ ( which startet at OCRxB )
        // we assume a max. runtime of 1000 Tics ( = 500µs , what nevver should happen )
        tmp = GET_COUNT - OCRxB ;
        if ( tmp > 1000 ) tmp += TIMER_OVL_TICS; // there was a timer overflow
        if ( tmp > (CYCLETICS-10) ) {
            // runtime was too long, next IRQ mus be started immediatly
            //SET_TP3;
            tmp = GET_COUNT+10; 
        } else {
            tmp = OCRxB + CYCLETICS;
        }
        OCRxB = ( tmp > TIMER_OVL_TICS ) ? tmp -= TIMER_OVL_TICS : tmp ;
        //SET_TP1;
    } else {
        // time till next IRQ is more then one cycletime
        // compute next IRQ-Time in us, not in tics, so we don't need long
        tmp = ( OCRxB / TICS_PER_MICROSECOND + nextCycle * CYCLETIME );
        if ( tmp > TIMERPERIODE ) tmp = tmp - TIMERPERIODE;
        OCRxB = tmp * TICS_PER_MICROSECOND;
    }
    interrupts();
    cyclesLastIRQ = nextCycle;
    CLR_TP1; // Oszimessung Dauer der ISR-Routine
}
////////////////////////////////////////////////////////////////////////////////////////////

void seizeTimerAS() {
    static bool timerInitialized = false;
    if ( !timerInitialized ) {
        uint8_t oldSREG = SREG;
        cli();
        
        TCCRxA =0; /* CTC Mode, ICRx is TOP */
        TCCRxB = _BV(WGMx3) | _BV(WGMx2) /* CTC Mode, ICRx is TOP */
      | _BV(CS11) /* div 8 clock prescaler */
      ;
        ICRx = TIMERPERIODE * TICS_PER_MICROSECOND;  // timer periode is 20000us 
        OCRxA = FIRST_PULSE;
        OCRxB = 400;
        // Serial.print( " Timer initialized " ); Serial.println( TIMSKx, HEX );
        SREG = oldSREG;  // undo cli() 
        timerInitialized = true;  
        MODE_TP1;   // set debug-pins to Output
        MODE_TP2;
        MODE_TP3;
        MODE_TP4;
        DB_PRINT("Testpins initialisiert");
    }
}

extern uint8_t spiStepperData[2]; // step pattern to be output on SPI
extern uint8_t spiByteCount;

#ifdef SPCR
// use an ISR only if we have a 'real' SPI Hardware
ISR ( SPI_STC_vect ) { 
    //SET_TP4;
    // output step-pattern on SPI, set SS when ready
    if ( spiByteCount++ == 0 ) {
        // end of shifting out high Byte, shift out low Byte
        SPDR = spiStepperData[0];
    } else {
        // end of data shifting
        //digitalWrite( SS, HIGH );
        SET_SS;
        spiByteCount = 0;
    }
    //CLR_TP4;
    
}
#endif

void enableSoftLedIsrAS() {
}

#endif