// AVR HW-spcific Functions
#ifdef ARDUINO_ARCH_MEGAAVR
#include <MobaTools.h>
#define debugTP
//#define debugPrint
#include <utilities/MoToDbg.h>

//#warning "HW specfic - megaavr ( Mega4809 ) ---"

uint8_t noStepISR_Cnt;   // Counter for nested StepISr-disable

nextCycle_t nextCycle;
static nextCycle_t cyclesLastIRQ = 1;  // cycles since last IRQ
// ---------- OCRxB Compare Interrupt used for stepper motor and Softleds ----------------
void stepperISR(uint8_t cyclesLastIRQ) __attribute__ ((weak));
void softledISR(uint8_t cyclesLastIRQ) __attribute__ ((weak));

// reenabling interrupts within an ISR
__attribute(( naked, noinline )) void isrIrqOn () { asm("reti"); }


ISR ( TCA0_CMP1_vect) {
    uint16_t tmp;
  // Timer TCA0 Compare 1, used for stepper motor, starts every CYCLETIME us
    // 26-09-15 An Interrupt is only created at timeslices, where data is to output
    SET_TP1;
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP1_bm;	// Reset IRQ-flag

    nextCycle = ISR_IDLETIME  / CYCLETIME ;// min ist one cycle per IDLETIME
    if ( stepperISR ) stepperISR(cyclesLastIRQ);
    //============  End of steppermotor ======================================
   if ( softledISR ) softledISR(cyclesLastIRQ);
    // ======================= end of softleds =====================================
    // set compareregister to next interrupt time;
    // compute next IRQ-Time in us, not in tics, so we don't need long
    noInterrupts(); // when manipulating 16bit Timerregisters IRQ must be disabled (mandatory for Mega4809!!!)
    if ( nextCycle == 1 )  {
        CLR_TP1;
        //noInterrupts();
        // This is timecritical: Was the ISR running longer then CYCELTIME?
        // compute length of current IRQ ( which startet at OCRxB )
        // Tic-Time is 4µs on 4809!!! ( because of millis() using the TCA0 prescaler )
        // We assume a max. runtime of 120 Tics ( = 480µs , what never should happen )
		// so if the difference is > 120 tics, we assume an timer overflow
        tmp = GET_COUNT - OCRxB ;
        if ( tmp > 120 ) tmp += TIMER_OVL_TICS; // there was a timer overflow
        if ( tmp > (CYCLETICS-2) ) {
            // runtime was too long, next IRQ mus be started immediatly
            //SET_TP3;
            tmp = GET_COUNT+2; 
        } else {
            tmp = OCRxB + CYCLETICS;
        }
        OCRxB = ( tmp > TIMER_OVL_TICS ) ? tmp - TIMER_OVL_TICS : tmp ;
        //interrupts();
        SET_TP1;
    } else {
        // time till next IRQ is more then one cycletime
        tmp = ( OCRxB + (nextCycle * CYCLETICS) );
        if ( tmp >= TIMER_OVL_TICS ) tmp = tmp - TIMER_OVL_TICS;
        OCRxB = tmp ;
    }
	interrupts();
    cyclesLastIRQ = nextCycle;
    CLR_TP1; // Oszimessung Dauer der ISR-Routine
}
////////////////////////////////////////////////////////////////////////////////////////////

void seizeTimerAS() {
    static bool timerInitialized = false;
    if ( !timerInitialized ) {
        // using timer TCA0 in normal mode.
        // CMP0 register used for steppers and softleds
        // CMP1 register used for servos
		// CMP2	register (not yet )used for  softleds
        noInterrupts();
        TCA0_SINGLE_CTRLESET = TCA_SINGLE_CMD_RESET_gc;     // hard reset timer
        //TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc;      // 0,5µs per tic, timer disabled - not possible because it influences millis()
        TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc;      // 4µs per tic ( also used by millis() ), timer disabled
		//+-> Arduino default of prescaler TCA0 is 64, changes would influence millis()
		// this means 4µs per tic for TCA0
        TCA0_SINGLE_CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;    // normal mode, no autolockUpdate, no pins active
        TCA0_SINGLE_CTRLC = 0;
        TCA0_SINGLE_CTRLD = 0;                              // no split mode ( user 16bit timer )
        //TCA0_SINGLE_CTRLECLR
        TCA0_SINGLE_CTRLESET = TCA_SINGLE_LUPD_bm;          // don't use buffered compare registes
        //TCA0_SINGLE_CTRLFCLR
        //TCA0_SINGLE_CTRLFSET
        //TCA0_SINGLE_INTCTRL = TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm; // enable cmp0 and cmp1 interrupt
        TCA0_SINGLE_INTFLAGS = 0;   // clear all interrupts
        TCA0_SINGLE_PER  = TIMERPERIODE * TICS_PER_MICROSECOND;  // timer periode is 20000us 
        TCA0_SINGLE_CMP0 = FIRST_PULSE;   
        TCA0_SINGLE_CMP1 = 400;  
        TCA0_SINGLE_CTRLA |= TCA_SINGLE_ENABLE_bm;          // Enable the timer
        interrupts();
        timerInitialized = true;  
        MODE_TP1;   // set debug-pins to Output
        MODE_TP2;
        MODE_TP3;
        MODE_TP4;
        DB_PRINT("CYCLETICS=%d, TIMER_OVL_TICS=%d", CYCLETICS, TIMER_OVL_TICS );
    }
}

extern uint8_t spiStepperData[2]; // step pattern to be output on SPI

ISR ( SPI0_INT_vect ) { 
    //SET_TP4;
    // Because of buffered SPI, both bytes have already been written to SPI HW
	// This IRQ fires, if both bytes have been shifted out
    SET_SS;
	SPI0_INTFLAGS = SPI_TXCIF_bm;     // Clear transfer complete flag
    //CLR_TP4;
    
}


void enableSoftLedIsrAS() {
}

#endif