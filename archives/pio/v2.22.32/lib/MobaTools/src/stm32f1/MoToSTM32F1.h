#ifndef MOTOSTM32F1_H
#define MOTOSTM32F1_H
// STM32F1 specific defines for Cpp files

//#warning STM32F1 specific cpp includes
extern uint8_t noStepISR_Cnt;   // Counter for nested StepISr-disable

void seizeTimerAS();
static inline __attribute__((__always_inline__)) void _noStepIRQ() {
    //timer_disable_irq(MT_TIMER, TIMER_STEPCH_IRQ);
    *bb_perip(&(MT_TIMER->regs).adv->DIER, TIMER_STEPCH_IRQ) = 0;
    noStepISR_Cnt++;
    //noInterrupts();
    //nvic_globalirq_disable();
    #if defined COMPILING_MOTOSTEPPER_CPP
        //Serial.println(noStepISR_Cnt);
        SET_TP3;
    #endif
}
static inline __attribute__((__always_inline__)) void  _stepIRQ(bool force = false) {
    //timer_enable_irq(MT_TIMER, TIMER_STEPCH_IRQ) cannot be used, because this also clears pending irq's
    if ( force ) noStepISR_Cnt = 1;              //enable IRQ immediately
    if ( noStepISR_Cnt > 0 ) noStepISR_Cnt -= 1; // don't decrease if already 0 ( if enabling IRQ is called too often )
    if ( noStepISR_Cnt == 0 ) {
        #if defined COMPILING_MOTOSTEPPER_CPP
            CLR_TP3;
        #endif
        *bb_perip(&(MT_TIMER->regs).adv->DIER, TIMER_STEPCH_IRQ) = 1;
        //nvic_globalirq_enable();
        //interrupts();
    }
    //Serial.println(noStepISR_Cnt);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSERVO_CPP
// Values for Servo: -------------------------------------------------------
constexpr uint8_t INC_PER_MICROSECOND = 8;		// one speed increment is 0.125 µs
constexpr uint8_t  COMPAT_FACT = 1; 			// no compatibility mode for stm32F1                       
constexpr uint8_t INC_PER_TIC = INC_PER_MICROSECOND / TICS_PER_MICROSECOND;
#define time2tic(pulse)  ( (pulse) *  INC_PER_MICROSECOND )
#define tic2time(tics)  ( (tics) / INC_PER_MICROSECOND )
#define AS_Speed2Inc(speed) (speed)
//-----------------------------------------------------------------

void ISR_Servo( void );


static inline __attribute__((__always_inline__)) void enableServoIsrAS() {
    timer_attach_interrupt(MT_TIMER, TIMER_SERVOCH_IRQ, ISR_Servo );
    timer_cc_enable(MT_TIMER, SERVO_CHN);
}

static inline __attribute__((__always_inline__)) void setServoCmpAS(uint16_t cmpValue) {
	// Set compare-Register for next servo IRQ
	timer_set_compare(MT_TIMER,  SERVO_CHN, cmpValue);
}	

#endif // COMPILING_MOTOSERVO_CPP

/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSOFTLED32_CPP
static inline __attribute__((__always_inline__)) void enableSoftLedIsrAS() {
    timer_cc_enable(MT_TIMER, STEP_CHN);
}

#endif // COMPILING_MOTOSOFTLED_CPP

//////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSTEPPER_CPP

static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
    timer_cc_enable(MT_TIMER, STEP_CHN);
}

static uint8_t spiInitialized = false;
static inline __attribute__((__always_inline__)) void initSpiAS() {
    if ( spiInitialized ) return;
    // initialize SPI hardware.
    // MSB first, default Clk Level is 0, shift on leading edge
    #ifdef USE_SPI2// use SPI 2 interface
    spi_init(SPI2);
    spi_config_gpios(SPI2, 1,  // initialize as master
                     PIN_MAP[BOARD_SPI2_NSS_PIN].gpio_device, PIN_MAP[BOARD_SPI2_NSS_PIN].gpio_bit,        
                     PIN_MAP[BOARD_SPI2_SCK_PIN].gpio_device, PIN_MAP[BOARD_SPI2_SCK_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI2_MISO_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI2_MOSI_PIN].gpio_bit);

    uint32 flags = (SPI_FRAME_MSB | SPI_CR1_DFF_16_BIT | SPI_SW_SLAVE | SPI_SOFT_SS);
    spi_master_enable(SPI2, (spi_baud_rate)SPI_BAUD_PCLK_DIV_64, (spi_mode)SPI_MODE_0, flags);
    spi_irq_enable(SPI2, SPI_RXNE_INTERRUPT);
    pinMode( BOARD_SPI2_NSS_PIN, OUTPUT);
    digitalWrite( BOARD_SPI2_NSS_PIN, LOW );

    #else// use SPI 1 interface
    spi_init(SPI1);
    spi_config_gpios(SPI1, 1,  // initialize as master
                     PIN_MAP[BOARD_SPI1_NSS_PIN].gpio_device, PIN_MAP[BOARD_SPI1_NSS_PIN].gpio_bit,        
                     PIN_MAP[BOARD_SPI1_SCK_PIN].gpio_device, PIN_MAP[BOARD_SPI1_SCK_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI1_MISO_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI1_MOSI_PIN].gpio_bit);

    uint32 flags = (SPI_FRAME_MSB | SPI_CR1_DFF_16_BIT | SPI_SW_SLAVE | SPI_SOFT_SS);
    spi_master_enable(SPI1, (spi_baud_rate)SPI_BAUD_PCLK_DIV_64, (spi_mode)SPI_MODE_0, flags);
    spi_irq_enable(SPI1, SPI_RXNE_INTERRUPT);
    pinMode( BOARD_SPI1_NSS_PIN, OUTPUT);
    digitalWrite( BOARD_SPI1_NSS_PIN, LOW );
    #endif
    spiInitialized = true;  
}

    static inline __attribute__((__always_inline__)) void startSpiWriteAS( uint8_t spiData[] ) {
        #ifdef USE_SPI2
        digitalWrite(BOARD_SPI2_NSS_PIN,LOW);
        spi_tx_reg(SPI2, (spiData[1]<<8) + spiData[0] );
        #else
        digitalWrite(BOARD_SPI1_NSS_PIN,LOW);
        spi_tx_reg(SPI1, (spiData[1]<<8) + spiData[0] );
        #endif
    }    
    

#endif // COMPILING_MOTOSTEPPER_CPP


#endif