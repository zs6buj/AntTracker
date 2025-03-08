#ifndef MOTORP2040_H
#define MOTORP2040_H
// RP2040 specific defines for Cpp files

//#pragma message RP2040 specific cpp includes
bool seizeTimerAS();

void servoISR( );

// defined in MoToRP2040.cpp:
extern uint8_t noStepISR_Cnt;   // Counter for nested StepISR-disable
extern uint8_t stepperIRQNum;  // dynamically assigned when timer is initialized
extern timer_hw_t *motoRPtimer; // Only for test-prints in .ino
extern uint8_t spiInitialized;
extern spi_inst_t *stepperSPI;
void initSpiAS();

inline __attribute__((__always_inline__)) void _noStepIRQ() {
  //disable stepper IRQ and count how often it has been disabled
  irq_set_enabled (stepperIRQNum, false);
  noStepISR_Cnt++;
#if defined COMPILING_MOTOSTEPPER_CPP
  //Serial.println(noStepISR_Cnt);
  //SET_TP3;
#endif
}
inline __attribute__((__always_inline__)) void  _stepIRQ(bool force = true) {
  //enable stepper IRQ id disable counter is 0
  if ( force ) noStepISR_Cnt = 1;              //enable IRQ immediately
  if ( noStepISR_Cnt > 0 ) noStepISR_Cnt -= 1; // don't decrease if already 0 ( if enabling IRQ is called too often )
  if ( noStepISR_Cnt == 0 ) {
#if defined COMPILING_MOTOSTEPPER_CPP
    //CLR_TP3;
#endif
    irq_set_enabled (stepperIRQNum, true);
  }
  //Serial.println(noStepISR_Cnt);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSERVO_CPP
constexpr uint8_t INC_PER_MICROSECOND = 8;		// one speed increment is 0.125 µs
constexpr uint8_t  COMPAT_FACT = 1; 			// no compatibility mode for RP2040                     
constexpr uint8_t INC_PER_TIC = INC_PER_MICROSECOND / 1; //PWM counter is initialized to 1µs per tic
#define time2tic(pulse)  ( (pulse) *  INC_PER_MICROSECOND )
#define tic2time(tics)  ( (tics) / INC_PER_MICROSECOND )
#define AS_Speed2Inc(speed) (speed)

//returns slice and channel number for the given pin. Returns -1 if not possible to get one
// sets slice and PWM accordingly
static inline __attribute__((__always_inline__)) int8_t servoPwmSetup( servoData_t *servoDataP ) {
	gpio_set_function(servoDataP->pin, GPIO_FUNC_PWM);

	// Find out which PWM slice is connected to GPIO
	uint slice_num = pwm_gpio_to_slice_num(servoDataP->pin);
	uint chanNum = pwm_gpio_to_channel(servoDataP->pin);
	//DB_PRINT("slice_num=%d, chanNum=%d, gpio=%d\n", slice_num, chanNum, servoDataP->pin );
	servoDataP->pwmNbr = (slice_num << 1) + chanNum;

	pwm_set_clkdiv(slice_num, F_CPU / 1000000L );
	pwm_set_irq_enabled (slice_num, true);
	// Set period
	pwm_set_wrap(slice_num, 20000);
	// Set initial duty
	pwm_set_chan_level(slice_num, chanNum, 0); // don't create impulses yet
	// Set the PWM running
	pwm_set_enabled(slice_num, true);

	// Note we could also use pwm_set_gpio_level(gpio, x) which looks up the
	// correct slice and channel for a given GPIO.
	//DB_PRINT("ServoNbr%d, Slice=%d, Chan=%d, Gpio=%d\n", servoDataP->servoIx, servoDataP->pwmNbr >> 1, servoDataP->pwmNbr & 1, servoDataP->pin);
	irq_set_exclusive_handler (PWM_IRQ_WRAP, servoISR);
	irq_set_enabled (PWM_IRQ_WRAP, true);
	return servoDataP->pwmNbr;
}

static inline __attribute__((__always_inline__)) void startServoPulse( servoData_t *servoDataP, uint32_t pulseWidth ) {
}

static inline __attribute__((__always_inline__)) void servoWrite( servoData_t *servoDataP, uint32_t pulseWidth ) {
  // the same funktion exists for ESP8266 ( with another internal call )

}

static inline __attribute__((__always_inline__)) void servoPulseOff( servoData_t *servoDataP ) {
  //DB_PRINT("Stop Puls, ledcNr=%d", servoDataP->pwmNbr );
}

static inline __attribute__((__always_inline__)) void servoDetach( servoData_t *servoDataP ) {
}
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef COMPILING_MOTOSOFTLED32_CPP
//returns the channelnumber ( 0...15 ) of the leds channel to be used, or -1 if no channel is availabele
static inline __attribute__((__always_inline__)) void enableSoftLedIsrAS() {
    // ToDo: timer_cc_enable(MT_TIMER, STEP_CHN);
}


#endif  // Ende COMPILING_MOTOSOFTLED32_CPP
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSTEPPER_CPP
static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
  // dummy
}


static inline __attribute__((__always_inline__)) void startSpiWriteAS( uint8_t spiData[] ) {
  // ToDo - spiWriteShortNL(spiHs, (spiData[1]<<8) + spiData[0] );
  // clear/discard rcv data ( from previous call )
    while (spi_is_readable(stepperSPI))
        (void)spi_get_hw(stepperSPI)->dr;
    while (spi_get_hw(stepperSPI)->sr & SPI_SSPSR_BSY_BITS)
        tight_loop_contents();
    while (spi_is_readable(stepperSPI))
        (void)spi_get_hw(stepperSPI)->dr;

    // Don't leave overrun flag set
    spi_get_hw(stepperSPI)->icr = SPI_SSPICR_RORIC_BITS;

    uint16_t datasent = (spiData[1]<<8) + spiData[0];
        while (!spi_is_writable(stepperSPI)) tight_loop_contents(); // sent register should be free ...
        spi_get_hw(stepperSPI)->dr = (uint32_t)datasent;
    
  
}
/* from SDK
// Write len bytes directly from src to the SPI, and discard any data received back
int __not_in_flash_func(spi_write16_blocking)(spi_inst_t *spi, const uint16_t *src, size_t len) {
    invalid_params_if(HARDWARE_SPI, 0 > (int)len);
    // Deliberately overflow FIFO, then clean up afterward, to minimise amount
    // of APB polling required per halfword
    for (size_t i = 0; i < len; ++i) {
        while (!spi_is_writable(spi))
            tight_loop_contents();
        spi_get_hw(spi)->dr = (uint32_t)src[i];
    }

    while (spi_is_readable(spi))
        (void)spi_get_hw(spi)->dr;
    while (spi_get_hw(spi)->sr & SPI_SSPSR_BSY_BITS)
        tight_loop_contents();
    while (spi_is_readable(spi))
        (void)spi_get_hw(spi)->dr;

    // Don't leave overrun flag set
    spi_get_hw(spi)->icr = SPI_SSPICR_RORIC_BITS;

    return (int)len;
}
*/

#endif

#endif