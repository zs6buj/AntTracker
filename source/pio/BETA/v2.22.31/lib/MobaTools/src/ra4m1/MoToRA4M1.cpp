// RA4M1 HW-spcific Functions
#ifdef ARDUINO_ARCH_RENESAS_UNO 
#include <MobaTools.h>
#define debugTP
//#define debugPrint
#include <utilities/MoToDbg.h>
#ifdef debugIRQ
#warning debugIRQ aktiv
	uint8_t dbgIx;	// index of irq values
	irqValues_h dbgTimer[dbgIxMax];
#endif
#ifdef debugSvIRQ
	#warning debug Servo IRQ active
	uint8_t dbSvIx;
	irqSvVal_t irqSvVal[dbSvIxMax];
#endif

//#warning "HW specfic - RA4M1 ---"
// RA4M1 specific variables
// Create timer related structures
timer_cfg_t timer_cfg;		// base structure for timers
TimerIrqCfg_t timerIrqCfg;	// used by IRQManager
GPTimer MoToGPT(timer_cfg); // create all extended structures for GPT.

R_GPT0_Type *gptRegP = (R_GPT0_Type *)R_GPT0_BASE;   // Pointer to active HW timer registers
R_ICU_Type *icuRegP = (R_ICU_Type *)R_ICU_BASE;      // Pointer to ICU Interrupt registers
// Event nbr for ICU table
// Index in ICU table for this event ( = NVIC-IRQ-Number )
IRQn_Type IRQnStepper = FSP_INVALID_VECTOR;     // NVIC-IRQ number for stepper IRQ ( cmpA )
IRQn_Type IRQnServo = FSP_INVALID_VECTOR ;      // NVIC-IRQ number for servo IRQ    (cmpB )

uint8_t noStepISR_Cnt = 0;   // Counter for nested StepISr-disable

void stepperISR(int32_t cyclesLastIRQ)  __attribute__ ((weak));
void softledISR(uint32_t cyclesLastIRQ)  __attribute__ ((weak));
nextCycle_t nextCycle;
static nextCycle_t cyclesLastIRQ = 1;  // cycles since last IRQ

void ISR_Stepper() {
    // GPT Timer CCMPA, used for stepper motor and softleds, starts every nextCycle us
	// Quit irq-flag
	icuRegP->IELSR_b[IRQnStepper].IR = 0;
#ifdef debugIRQ
	if( dbgIx < dbgIxMax ) {
		dbgTimer[dbgIx].timestamp = micros();
		dbgTimer[dbgIx].preTimerCnt = gptRegP->GTCNT;
		dbgTimer[dbgIx].preTimerCmp = gptRegP->GTCCR[0];
		dbgTimer[dbgIx].cyclesLastIRQ = cyclesLastIRQ;
	}
#endif
 

 // nextCycle ist set in stepperISR and softledISR
    SET_TP1;
    nextCycle = ISR_IDLETIME  / CYCLETIME ;// min ist one cycle per IDLETIME
    if ( stepperISR ) stepperISR(cyclesLastIRQ);
    //============  End of steppermotor ======================================
    if ( softledISR ) softledISR(cyclesLastIRQ);
    // ======================= end of softleds =====================================
    // set compareregister to next interrupt time;
	// next ISR must be at least MIN_STEP_CYCLE/4 beyond actual counter value ( time between to ISR's )
	uint32_t minOCR = gptRegP->GTCNT;
	uint32_t nextOCR = gptRegP->GTCCR[0];  // CCRA = Step cmp
	if ( minOCR < nextOCR ) minOCR += TIMER_OVL_TICS; // timer had overflow already
    minOCR = minOCR + ( (MIN_STEP_CYCLE/4) * TICS_PER_MICROSECOND ); // minimumvalue for next OCR
	nextOCR = nextOCR + ( nextCycle * TICS_PER_MICROSECOND );
	if ( nextOCR < minOCR ) {
		// time till next ISR ist too short, set to mintime and adjust nextCycle
        SET_TP2;
		nextOCR = minOCR;
		nextCycle = ( nextOCR - gptRegP->GTCCR[0]  ) / TICS_PER_MICROSECOND;
        CLR_TP2;
	}
    if ( nextOCR > (uint16_t)TIMER_OVL_TICS ) nextOCR -= TIMER_OVL_TICS;
    gptRegP->GTCCR[0] = nextOCR  ;
    cyclesLastIRQ = nextCycle;
#ifdef debugIRQ
	if( dbgIx < dbgIxMax ) {
		dbgTimer[dbgIx].postTimerCnt = gptRegP->GTCNT;
		dbgTimer[dbgIx].postTimerCmp = gptRegP->GTCCR[0];
		dbgTimer[dbgIx].nextCycle = nextCycle;
		dbgIx++;
	}
#endif
    CLR_TP1; // Oszimessung Dauer der ISR-Routine
}
 ////////////////////////////////////////////////////////////////////////////////////////////
 #ifdef debugOvf
void ISR_Ovf() {
    // GPT Timer Overflow - only for testing purposes
	static bool state = false;
	// Acknoledge irq-flag
	icuRegP->IELSR_b[timer_cfg.cycle_end_irq].IR = 0;
	if ( state ) SET_TP4;		// trigger for oscilloscope testing
	else CLR_TP4;
	state = !state;
	//digitalWrite(1, !digitalRead(1) );
	
}
#endif
////////////////////////////////////////////////////////////////////////////////////////////
void ISR_ServoRA4() {
	// Acknowledge interrupt
	icuRegP->IELSR_b[IRQnServo].IR = 0;
	ISR_Servo();
}
////////////////////////////////////////////////////////////////////////////////////////////

void seizeTimerAS() {
  int8_t tindex;  // used Timer;
  static bool timerInitialized = false;
  if ( !timerInitialized ) {
    // Initialize GPT Timer
	timerInitialized = true;
    uint8_t timer_type = GPT_TIMER;

    tindex = FspTimer::get_available_timer(timer_type);
    FspTimer::set_timer_is_used(GPT_TIMER, tindex);
    // Timer aktivieren
    if ( tindex > 1 ) R_MSTP->MSTPCRD_b.MSTPD6 = 0; // GPT2..GPT7
    else              R_MSTP->MSTPCRD_b.MSTPD5 = 0; // GPT0..GPT1 ( 32 bit timer )
    // compute pointer to active timer registers
    gptRegP = (R_GPT0_Type *)((uint8_t *)R_GPT0_BASE + (0x100 * tindex));
    timer_cfg.mode                                  = TIMER_MODE_PERIODIC;
    timer_cfg.source_div                            = TIMER_SOURCE_DIV_16;
    timer_cfg.period_counts                         = TIMER_OVL_TICS;
    timer_cfg.duty_cycle_counts                     = 0;
    timer_cfg.p_callback                            = NULL;
    timer_cfg.p_context                             = NULL;
    //timer_cfg.p_extend     already set by GPTimer
    timer_cfg.cycle_end_ipl                         = (BSP_IRQ_DISABLED);
    timer_cfg.cycle_end_irq                        = FSP_INVALID_VECTOR;;
    timer_cfg.channel = tindex;
    FspTimer::set_timer_is_used(GPT_TIMER, tindex);
    MoToGPT.ctrl.p_reg = gptRegP;       // Pointer to HW-registers
    MoToGPT.ctrl.channel_mask = 1 << tindex;  // Mask for starting/stopping the timer

    // IRQ-related structure
    timerIrqCfg.base_cfg = &timer_cfg;
    timerIrqCfg.gpt_ext_cfg = &MoToGPT.ext_cfg;
    timerIrqCfg.agt_ext_cfg = NULL;

    // set HW registers ( only if different from reset values )
    gptRegP->GTSSR_b.CSTRT  = 1;                    // enable starting the counter
    gptRegP->GTPSR_b.CSTOP  = 1;                    // enable stopping the counter
    gptRegP->GTCSR_b.CCLR   = 1;                    // enable clearing the counter
    gptRegP->GTCR_b.MD      = TIMER_MODE_PERIODIC;  // rest if max is reached
    gptRegP->GTCR_b.TPCS    = TIMER_SOURCE_DIV_16;  // Clock div16 ( 3tics per microsecond with 48MHz clock )
    gptRegP->GTUDDTYC_b.UD  = 1;                    // count up
    gptRegP->GTBER          = 0x3;                  // no Buffer operation
    gptRegP->GTCCR[0] = 0;
    gptRegP->GTCCR[1] = 0;
    gptRegP->GTPR = TIMER_OVL_TICS;                 // Max count
    gptRegP->GTPBR = TIMER_OVL_TICS;                // Max count buffer register ( not used )

    // start counter
    gptRegP->GTCR_b.CST = 1;
    gptRegP->GTSSR_b.CSTRT = 1;
    gptRegP->GTSTR = MoToGPT.ctrl.channel_mask;
#ifdef debugOvf
    IRQManager::getInstance().addTimerOverflow(timerIrqCfg, &ISR_Ovf);
    //IRQnOvf = timer_cfg.cycle_end_irq;   // NVIC IRQ-number overflow ISR
    NVIC_SetPriority(timer_cfg.cycle_end_irq,15);
    NVIC_EnableIRQ(timer_cfg.cycle_end_irq);

#endif
	MODE_TP1;
	MODE_TP2;
	MODE_TP3;
	MODE_TP4;
    }
}




#endif
