#ifndef MOTOAVR_H
#define MOTOAVR_H
// AVR specific defines for Cpp files

//#warning AVR specific cpp includes
void seizeTimerAS();
extern uint8_t noStepISR_Cnt;   // Counter for nested StepISr-disable

// _noStepIRQ und _stepIRQ werden in servo.cpp und stepper.cpp genutzt
static inline __attribute__((__always_inline__)) void _noStepIRQ() {
        TIMSKx &= ~_BV(OCIExB) ; 
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
        TIMSKx |= _BV(OCIExB) ; 
    }
}

static inline __attribute__((__always_inline__)) void nestedInterrupts() {
	//to reenable interrupts within an ISR
	interrupts(); 
}
/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSERVO_CPP
// Values for Servo: -------------------------------------------------------
constexpr uint8_t INC_PER_MICROSECOND = 8;		// one speed increment is 0.125 µs
constexpr uint8_t  COMPAT_FACT = INC_PER_MICROSECOND /2; // old Increment value was same as Timer Tics ( 2 Tics/µs                           
// defaults for macros that are not defined in architecture dependend includes
constexpr uint8_t INC_PER_TIC = INC_PER_MICROSECOND / TICS_PER_MICROSECOND;
#define time2tic(pulse)  ( (pulse) *  INC_PER_MICROSECOND )
#define tic2time(tics)  ( (tics) / INC_PER_MICROSECOND )
#define AS_Speed2Inc(speed) (speed)


static inline __attribute__((__always_inline__)) void enableServoIsrAS() {
    // enable compare-A interrupt
    TIMSKx |=  _BV(OCIExA) ; 
}

#define setServoCmpAS(x)		// ignore this call

#endif // COMPILING_MOTOSERVO_CPP

/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSOFTLED_CPP

#endif // COMPILING_MOTOSOFTLED_CPP

//////////////////////////////////////////////////////////////////////////////////////////////////
// Wird auch in MoToAVR.cpp gebraucht ( SPI-Interrupt )
extern volatile uint8_t *portSS;
extern uint8_t bitSS;;
#define SET_SS *portSS |= bitSS 
#define CLR_SS *portSS &= ~bitSS 
#if defined COMPILING_MOTOSTEPPER_CPP
    static uint8_t spiInitialized = false;
    // Macros für fast setting of SS Port
    #if !defined SPCR && defined USI_SS
        // we don't have a HW SPI, and SS has been defined in MobaTools.h
        #undef SS
        #define SS USI_SS
    #endif
    /* //Testweise den SS-Pin ausgeben
    #define STRING2(x) #x
    #define STRING(x) "\n\r>>>>>>SS-Pin: "  STRING2(x)
    #ifdef PIN_SPI_SS
        #pragma message (STRING(PIN_SPI_SS))
    #elif defined SS
        #pragma message (STRING(SS))
    #else
        #pragma message "SS-Pin not defined"
    #endif
    */
    volatile uint8_t *portSS;
    uint8_t bitSS;;

#ifdef SPCR
    // we hae ar real SPI hardware
    //#warning "SPI Hardware is used"
    uint8_t spiByteCount = 0;
    static inline __attribute__((__always_inline__)) void initSpiAS() {
        if ( spiInitialized ) return;
        // initialize SPI hardware.
        // MSB first, default Clk Level is 0, shift on leading edge
        const uint8_t oldSREG = SREG;
        cli();
        pinMode( MOSI, OUTPUT );
        pinMode( SCK, OUTPUT );
        portSS = portOutputRegister(digitalPinToPort(SS));
        bitSS = digitalPinToBitMask(SS);
        pinMode( SS, OUTPUT );
        SPCR = (1<<SPIE)    // Interrupt enable
             | (1<<SPE )    // SPI enable
             | (0<<DORD)    // MSB first
             | (1<<MSTR)    // Master Mode
             | (0<<CPOL)    // Clock is low when idle
             | (0<<CPHA)    // Data is sampled on leading edge
             | (0<<SPR1) | (1<<SPR0);    // fosc/16
        //digitalWrite( SS, HIGH );
        SET_SS;
        SREG = oldSREG;  // undo cli() 
        spiInitialized = true;  
    }

    static inline __attribute__((__always_inline__)) void startSpiWriteAS( uint8_t spiData[] ) {
        //digitalWrite( SS, LOW );
        CLR_SS;
        spiByteCount = 0;
        SPDR = spiData[1];
    }    
    
    
#elif defined USICR
    // only an USI HW is available
    
    //#warning "USI in 3wire-Mode ist used"
    static inline __attribute__((__always_inline__)) void initSpiAS() {
        if ( spiInitialized ) return;
        // set OutputPins MISO ( =DO )
        USI_SCK_PORT |= _BV(USCK_DD_PIN);   //set the USCK pin as output
        USI_DDR_PORT |= _BV(DO_DD_PIN);     //set the DO pin as output
        USI_DDR_PORT &= ~_BV(DI_DD_PIN);    //set the DI pin as input
        // set Controlregister USICR 
        USICR = 0;  //reset
        // set to 3-wire ( =SPI ) mode0,  Clock by USITC-bit, positive edge
        USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK);
        portSS = portOutputRegister(digitalPinToPort(SS));
        bitSS = digitalPinToBitMask(SS);
        pinMode( SS, OUTPUT );
        SET_SS; //digitalWrite( SS, HIGH );
        spiInitialized = true;  
    }

    
    static inline __attribute__((__always_inline__)) void startSpiWriteAS( uint8_t spiData[] ) {
        SET_TP4;
        CLR_SS;
        USIDR = spiData[1];
        #ifdef FASTSPI  // SPI mit syclk/2
        uint8_t usicrTemp = USICR | _BV(USITC);
        USICR = usicrTemp;  // Anweisung benötigt einen systic      
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USIDR = spiData[0];
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;        
        USICR = usicrTemp;   
        #else  // SPI mit syclk/4
        USICR |= _BV(USITC);      // Anweisung benötigt zwei systic          
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USIDR = spiData[0];
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);        
        USICR |= _BV(USITC);   
        #endif
        SET_SS;
        CLR_TP4;
    }    
    
#endif  // Ende unterschiedliche AVR Prozessoren für initSPI
 
static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
        TIMSKx |= _BV(OCIExB) ; 
}

#endif // COMPILING_MOTOSTEPPER_CPP


#endif