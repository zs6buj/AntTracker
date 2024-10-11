#ifndef MOTORA4M1_H
#define MOTORA4M1_H
// RA4M1 specific defines for Cpp files

#include "FspTimer.h"
#include <bsp_api.h>
//#define debugIRQ	// create variables for Stepper IRQ debugging
//#define debugSvIRQ	// create variables for Servo IRQ debugging
//#define debugOvf	// Create overflow IRQ for OSC-triggering
//#define debugPrint

#ifdef debugIRQ
// Variables for IRQ debugging zhat can be used in the sketch
extern uint8_t dbgIx;	// index of irq values
typedef struct {
	uint32_t timestamp;
	uint16_t cyclesLastIRQ;
	uint16_t nextCycle;
	uint16_t preTimerCnt;
	uint16_t preTimerCmp;
	uint16_t postTimerCnt;
	uint16_t postTimerCmp;
}irqValues_h ;
extern irqValues_h dbgTimer[];
constexpr uint8_t dbgIxMax = 10;	
#endif
#ifdef debugSvIRQ
	// debugging des Servo-IRQ
	extern uint8_t dbSvIx;
	constexpr uint8_t dbSvIxMax = 49;
typedef struct {
	uint32_t timestamp;
	uint16_t cmpBVal;
}irqSvVal_t;
	extern irqSvVal_t irqSvVal[];
#endif

//#warning RA4M1 specific cpp includes
constexpr byte NVIC_ServoPrio = IRQ_PRIO-1;
constexpr byte NVIC_StepperPrio = IRQ_PRIO;

extern R_GPT0_Type *gptRegP;                     // Pointer to active timer registers
extern R_ICU_Type *icuRegP;      // Pointer to Interrupt registers

//constexpr uint8_t evGPT_OFSET = 8;
extern timer_cfg_t timer_cfg;
extern TimerIrqCfg_t timerIrqCfg;
extern GPTimer MoToGPT; // create all structures for GPT.

extern IRQn_Type IRQnStepper ;        // NVIC-IRQ number for stepper IRQ ( cmpA )
extern IRQn_Type IRQnServo ;       // NVIC-IRQ number for servo IRQ    (cmpB )

void ISR_Servo();
void ISR_Stepper();
void seizeTimerAS();

extern uint8_t noStepISR_Cnt;   // Counter for nested StepISr-disable

void seizeTimerAS();

static inline __attribute__((__always_inline__)) void _noStepIRQ() {
    // disable stepper IRQ ( GPT cmpA match IRQ )
    NVIC_DisableIRQ(IRQnStepper);
    noStepISR_Cnt++;
    #if defined COMPILING_MOTOSTEPPER_CPP
        //Serial.println(noStepISR_Cnt);
        SET_TP3;
    #endif
}
static inline __attribute__((__always_inline__)) void  _stepIRQ(bool force = false) {
	// enable stepper IRQ ( GPT cmpA match IRQ )
    if ( force ) noStepISR_Cnt = 1;              //enable IRQ immediately
    if ( noStepISR_Cnt > 0 ) noStepISR_Cnt -= 1; // don't decrease if already 0 ( if enabling IRQ is called too often )
    if ( noStepISR_Cnt == 0 ) {
        #if defined COMPILING_MOTOSTEPPER_CPP
            CLR_TP3;
        #endif
        NVIC_EnableIRQ(IRQnStepper);

    }
    //Serial.println(noStepISR_Cnt);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSERVO_CPP
// Values for Servo: -------------------------------------------------------
constexpr uint8_t INC_PER_MICROSECOND = 12;		// one speed increment is 0.125 µs
constexpr uint8_t  COMPAT_FACT = 1; // no compatibility mode for this board                      
constexpr uint8_t INC_PER_TIC = INC_PER_MICROSECOND / TICS_PER_MICROSECOND;
#define time2tic(pulse)  ( (pulse) *  INC_PER_MICROSECOND )
#define tic2time(tics)  ( (tics) / INC_PER_MICROSECOND )
#define AS_Speed2Inc(speed)  (speed*INC_PER_MICROSECOND/8)  // Speedtic = 1/12 µs ( Timertic* Inc == 0.125 µs
//-----------------------------------------------------------------

void ISR_ServoRA4();
static inline __attribute__((__always_inline__)) void enableServoIsrAS() {
  if ( IRQnServo == FSP_INVALID_VECTOR ) {
    IRQManager::getInstance().addTimerCompareCaptureB(timerIrqCfg, &ISR_ServoRA4);
    IRQnServo = MoToGPT.ext_cfg.capture_b_irq;   // NVIC IRQ-number overflow ISR
    NVIC_SetPriority(IRQnServo,NVIC_ServoPrio);
    NVIC_EnableIRQ(IRQnServo);

  }
}

static inline __attribute__((__always_inline__)) void setServoCmpAS(uint16_t cmpValue) {
	// Set compare-Register for next servo IRQ
#ifdef debugSvIRQ
	if ( dbSvIx < dbSvIxMax ) {
		irqSvVal[dbSvIx].timestamp = micros();
		irqSvVal[dbSvIx].cmpBVal = cmpValue;
		dbSvIx++;
	}
#endif
	gptRegP->GTCCR[1] = cmpValue > TIMER_OVL_TICS ? TIMER_OVL_TICS : cmpValue;
}	
#endif // COMPILING_MOTOSERVO_CPP

void ISR_Stepper();
/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSOFTLED32_CPP
static inline __attribute__((__always_inline__)) void enableSoftLedIsrAS() {
  if ( IRQnStepper == FSP_INVALID_VECTOR ) {
    // IRQ not yet initialzed
    IRQManager::getInstance().addTimerCompareCaptureA(timerIrqCfg, &ISR_Stepper);
    IRQnStepper = MoToGPT.ext_cfg.capture_a_irq;   // NVIC IRQ-number overflow ISR
    NVIC_SetPriority(IRQnStepper,NVIC_StepperPrio);
    NVIC_EnableIRQ(IRQnStepper);

  }
}

#endif // COMPILING_MOTOSOFTLED_CPP

//////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSTEPPER_CPP

static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
	if ( IRQnStepper == FSP_INVALID_VECTOR ) {
    // IRQ not yet initialzed
    IRQManager::getInstance().addTimerCompareCaptureA(timerIrqCfg, &ISR_Stepper);
    IRQnStepper = MoToGPT.ext_cfg.capture_a_irq;   // NVIC IRQ-number overflow ISR
    printf("IRQnStepper=%d",(uint8_t)IRQnStepper);
    NVIC_SetPriority(IRQnStepper,NVIC_StepperPrio);
    NVIC_EnableIRQ(IRQnStepper);

  }
}

static uint8_t spiInitialized = false;
// Pointer für SPI-Register ( Minima uses SPI1, WiFi uses SPI0 )
// Ports für SPI pins
// MISO is not used ( SPI transfer only mode )
// SS is set by HW, so we don't need a finished IRQ
#define PERI_SPI 0b00110
#ifdef ARDUINO_UNOR4_MINIMA
#define SPI_ENABIT MSTPB18
#define R_SPI_R4 ((R_SPI0_Type *)R_SPI1_BASE)
#define SPI_PORT 1
#define SPI_PORT_MIMO 1
#define SS_PIN 12
#define MOSI_PIN 9
#define CLOCK_PIN 11
// MISO is not used ( SPI transfer only mode )
#elif defined ARDUINO_UNOR4_WIFI
#define R_SPI_R4 ((R_SPI0_Type *)R_SPI0_BASE)
#define SPI_ENABIT MSTPB19
#define SPI_PORT 1
#define SPI_PORT_MIMO 4
#define SS_PIN 3
#define MOSI_PIN 11
#define CLOCK_PIN 2
#else
#error unsupported board
#endif

static inline __attribute__((__always_inline__)) void initSpiAS() {
    if ( spiInitialized ) return;
    // initialize SPI hardware.
    // MSB first, default Clk Level is 0, shift on leading edge
	// SPI aktivieren
	R_MSTP->MSTPCRB_b.SPI_ENABIT = 0;  // SPI

	// Pins zuordnen
	// Der Zugriff auf die Pin-Register muss erst freigegeben werden
	//  #define R_PMISC        ((R_PMISC_Type *) R_PMISC_BASE)
	R_PMISC->PWPR_b.B0WI = 0;
	R_PMISC->PWPR_b.PFSWE = 1;

	R_PFS->PORT[SPI_PORT_MIMO].PIN[MOSI_PIN].PmnPFS_b.PMR = 1;  // Pin109 is MOSIB
	R_PFS->PORT[SPI_PORT_MIMO].PIN[MOSI_PIN].PmnPFS_b.PSEL = PERI_SPI;

	R_PFS->PORT[SPI_PORT].PIN[CLOCK_PIN].PmnPFS_b.PMR = 1;  // Pin111 is RSPCKB
	R_PFS->PORT[SPI_PORT].PIN[CLOCK_PIN].PmnPFS_b.PSEL = PERI_SPI;

	R_PFS->PORT[SPI_PORT].PIN[SS_PIN].PmnPFS_b.PMR = 1;  // Pin112 is SSLB0
	R_PFS->PORT[SPI_PORT].PIN[SS_PIN].PmnPFS_b.PSEL = PERI_SPI;

	R_SPI_R4->SPCR_b.SPE = 0;  // SPI disable ( set to 1 after full initialize

	R_SPI_R4->SPCR_b.SPMS = 0;    // SPI operation
	R_SPI_R4->SPCR_b.TXMD = 1;    // only transmit operations
	R_SPI_R4->SPCR_b.MODFEN = 0;  // no mode error detection / single master mode
	R_SPI_R4->SPCR_b.MSTR = 1;    // Master mode
	R_SPI_R4->SPCR_b.SPEIE = 0;   // no error interrupts
	R_SPI_R4->SPCR_b.SPTIE = 0;   // no transmit empty IRQ
	R_SPI_R4->SPCR_b.SPRIE = 0;   // no recv IRQ

	R_SPI_R4->SSLP = 0;  // Alle SSL active LOW

	R_SPI_R4->SPPCR_b.MOIFV = 0;  // MOSI idle is LOW
	R_SPI_R4->SPPCR_b.MOIFE = 1;  // MOSI idle entspr. MOIFV

	R_SPI_R4->SPBR = 5;  // bit rate setting

	R_SPI_R4->SPDCR_b.SPLW = 0;  // Half word access data buffer
							   /*
	R_SPI_R4->SPCKD_b.SCKDL = 4;    // delay SS to clock start
	R_SPI_R4->SSLND_b.SLNDL = 4;    // delay clock end to SS
	R_SPI_R4->SPND_b.SPNDL = 1;     // delay between transmissions 
	*/
	R_SPI_R4->SPCR2 = 0;         // default, no parity
	// Modes of operation
	R_SPI_R4->SPCMD_b[0].SPB = 0xF;  // Data length = 16  bit
	R_SPI_R4->SPCMD_b[0].CPHA = 0;   // sampling on rising edge
	R_SPI_R4->SPCMD_b[0].BRDV = 1;   // prescaler /2
	R_SPI_R4->SPCMD_b[0].SSLA = 0;   // SSL0 activ


	R_SPI_R4->SPCR_b.SPE = 1;  // SPI enable

    spiInitialized = true;  
}

static inline __attribute__((__always_inline__)) void startSpiWriteAS( uint8_t spiData[] ) {
	R_SPI_R4->SPDR_HA = (spiData[1]<<8) + spiData[0];
}    
    

#endif // COMPILING_MOTOSTEPPER_CPP


#endif