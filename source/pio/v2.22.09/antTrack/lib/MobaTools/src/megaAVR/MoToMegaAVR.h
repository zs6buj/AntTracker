#ifndef MOTOMEGAAVR_H
#define MOTOMEGAAVR_H
// AVR specific defines for Cpp files

//#warning megaAVR specific cpp includes

void seizeTimerAS();
// reenabling interrupts within an ISR
__attribute(( naked, noinline )) void isrIrqOn ();

extern uint8_t noStepISR_Cnt;   // Counter for nested StepISr-disable

// _noStepIRQ und _stepIRQ werden in servo.cpp und stepper.cpp genutzt
static inline __attribute__((__always_inline__)) void _noStepIRQ() {
        TCA0_SINGLE_INTCTRL &= ~TCA_SINGLE_CMP1_bm ; 
    noStepISR_Cnt++;
    #if defined COMPILING_MOTOSTEPPER_CPP
    SET_TP3;
    #endif
    interrupts(); // allow other interrupts
}

static inline __attribute__((__always_inline__)) void  _stepIRQ(bool force = false) {
    if ( force ) noStepISR_Cnt = 1; //enable IRQ immediately
    if ( noStepISR_Cnt > 0 ) noStepISR_Cnt -= 1; // don't decrease if already 0 ( if enabling IRQ is called too often )
    if ( noStepISR_Cnt == 0 ) {
        #if defined COMPILING_MOTOSTEPPER_CPP
            CLR_TP3;
        #endif
        TCA0_SINGLE_INTCTRL |= TCA_SINGLE_CMP1_bm ; 
    }
}

static inline __attribute__((__always_inline__)) void nestedInterrupts() {
	//to reenable interrupts within an ISR
	isrIrqOn ();
}
/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSERVO_CPP
// Values for Servo: -------------------------------------------------------
constexpr uint8_t INC_PER_MICROSECOND = 8;		// one speed increment is 0.125 µs
constexpr uint8_t  COMPAT_FACT = 1; // no compatibility mode for this board                     
// defaults for macros that are not defined in architecture dependend includes
constexpr uint8_t INC_PER_TIC = INC_PER_MICROSECOND / TICS_PER_MICROSECOND;
#define time2tic(pulse)  ( (pulse) *  INC_PER_MICROSECOND )
#define tic2time(tics)  ( (tics) / INC_PER_MICROSECOND )
#define AS_Speed2Inc(speed) (speed)


static inline __attribute__((__always_inline__)) void enableServoIsrAS() {
    // enable compare-A interrupt
    TCA0_SINGLE_INTCTRL |= TCA_SINGLE_CMP0_bm ; 
}

#define setServoCmpAS(x)		// ignore this call

#endif // COMPILING_MOTOSERVO_CPP

/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSOFTLED_CPP

#endif // COMPILING_MOTOSOFTLED_CPP

//////////////////////////////////////////////////////////////////////////////////////////////////
// Wird auch in MoToMegaAVR.cpp gebraucht ( SPI-Interrupt )
extern volatile PORT_t  *portSS;
extern uint8_t bitSS;
#define SET_SS portSS->OUTSET = bitSS 
#define CLR_SS portSS->OUTCLR = bitSS 
#if defined COMPILING_MOTOSTEPPER_CPP
    static uint8_t spiInitialized = false;
    // Macros für fast setting of SS Port	
    volatile PORT_t  *portSS;
    uint8_t bitSS;

    static inline __attribute__((__always_inline__)) void initSpiAS() {
        if ( spiInitialized ) return;
        // initialize SPI hardware.
        // MSB first, default Clk Level is 0, shift on leading edge
        const uint8_t oldSREG = SREG;
        cli();
		// MOSI,MISO ans SCK are the standard pins corresponding to the selected board
        pinMode( MOSI, OUTPUT );
        pinMode( SCK, OUTPUT );
		// SS is driven by MobaTools and can be any pin ( defined in MobaTools.h )
		// default is pin 8 for Nano Every and UNO Rev2 WiFi
        portSS = digitalPinToPortStruct(MoToSS);
        bitSS = digitalPinToBitMask(MoToSS);
        pinMode( MoToSS, OUTPUT );
		// Map SPI0-pins
		PORTMUX_TWISPIROUTEA &=  ~PORTMUX_SPI0_gm; // Clear SPI-Bits
		PORTMUX_TWISPIROUTEA |=  SPI_MUX; // set MUX according to Board
		// SPI-Mode 0 mit Sendebuffer ( 2.byte kann sofort geschrieben werden )
		// SPI0_CTRLB = SPI_MODE_0_gc | SPI_BUFEN_bm | SPI_BUFWR_bm | SPI_SSD_bm;
		SPI0_CTRLB = SPI_MODE_0_gc | SPI_BUFEN_bm | SPI_BUFWR_bm | SPI_SSD_bm;
		SPI0_INTCTRL = SPI_TXCIE_bm;     // Using Transfer complete ISR
		SPI0_CTRLA =  SPI_PRESC_DIV4_gc |      // prescaler 4 / 4Mhz SPI clk
					( 1 << SPI_MASTER_bp ) | // Master Mode
					( 0 << SPI_DORD_bp ) |   // MSB first
					( 1 << SPI_ENABLE_bp );     // SPI enable
        //digitalWrite( SS, HIGH );
        SET_SS;
        SREG = oldSREG;  // undo cli() 
        spiInitialized = true;  
    }

    static inline __attribute__((__always_inline__)) void startSpiWriteAS( uint8_t spiData[] ) {
        //digitalWrite( SS, LOW );
        CLR_SS;
        SPI0_DATA = spiData[1];
        SPI0_DATA = spiData[0];
    }    
    
    
 
static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
        TCA0_SINGLE_INTCTRL |= TCA_SINGLE_CMP1_bm ;  
}

#endif // COMPILING_MOTOSTEPPER_CPP


#endif