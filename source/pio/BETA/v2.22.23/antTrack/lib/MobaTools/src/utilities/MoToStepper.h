#ifndef MOTOSTEPPER_H
#define MOTOSTEPPER_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2020 All right reserved.

  Definitions and declarations for the stepper part of MobaTools
*/

// defines for the stepper motor
#define NOSTEP      0   // invalid-flag
// The numbers for FULLSTEP and HALFSTEP must not be changed!
#define HALFSTEP    1	// in HALFSTEP steps are counted in increments of 1 ( same like STEPDIR )
#define FULLSTEP    2	// in FULLSTEP the steps are  counted in increments of 2
#define A4988       3   // using motordriver A4988
#define STEPDIR		3	// all motordrivers with a step und dir input ( same as A4988 )


// Output modes ( outarg in attach method )
#define NO_OUTPUT   0

#if defined PORTD && defined PORTB
// V2.6: not allowed anymore ( wasn't described in Doku since V0.8
//#define PIN8_11     1
//#define PIN4_7      2
#endif

#ifndef ESP8266
#define SPI_1           3
#define SPI_2           4
#define SPI_3           5
#define SPI_4           6
#define SINGLE_PINS     7
#endif
#define A4988_PINS      8


// #define CYCLETICS       (CYCLETIME*TICS_PER_MICROSECOND)
constexpr uint16_t CYCLETICS   =  (CYCLETIME*TICS_PER_MICROSECOND);
#define MIN_START_CYCLES 4000/CYCLETIME  // 5ms min until first step if stepper is in stop
#define MIN_STEPTIME    (CYCLETIME * MIN_STEP_CYCLE) 
#ifdef IS_32BIT
#define MAXRAMPLEN      160000 
#else
#define MAXRAMPLEN      16000       // Do not change!
#endif

#ifdef __STM32Fx__
    void ISR_Stepper(void);
#endif


/////////////////////////////////////////////////////////////////////////////////
// global stepper data ( used in ISR )
enum class rampStat:byte { INACTIVE, STOPPED, SPEED0, STOPPING, STARTING, CRUISING, LASTSTEP, RAMPACCEL, RAMPDECEL, SPEEDDECEL  };
/*
// states from CRUISING and above mean that the motor is moving
// STOPPING: Motor does not move - waiting for disabling the motor.
// SPEED0:	 motor is stopped because speed is set to 0, target is not yet reached ( esp. for ESP8266 )
// STARTING: motor does not yet move, waiting time after enable
*/
typedef struct stepperData_t {
  struct stepperData_t *nextStepperDataP;    // chain pointer
  volatile uint32_t stepCnt;        // nmbr of steps to take
  uint32_t stepCnt2;                // nmbr of steps to take after automatic reverse
  volatile int8_t patternIx;    // Pattern-Index of actual Step (0-7)
  int8_t   patternIxInc;        // halfstep: +/-1, fullstep: +/-2, STEPDIR +1/-1  the sign defines direction
  #ifdef ESP8266
	// on ESP8266 all time values are in µs
	uint32_t tCycSteps;           // µseconds per step ( target value of motorspeed  )
	volatile uint32_t aCycSteps;  // µseconds per step ( actual motorspeed  )
	uint32_t cyctXramplen;        // precompiled  tCycSteps*(rampLen+RAMPOFFSET)
    uint16_t cycDelay;            // delay time: enable -> stepping
    boolean  dirChange;          // Flag: Dir has to be changed ( at falling edge )
  #else
	// on the other platforms the time values count in cycles.
    // On 32-bit processors cyclelength is 1 µsec, and there is no remainder
	uintxx_t tCycSteps;           // nbr of IRQ cycles per step ( target value of motorspeed  )
	volatile uintxx_t aCycSteps;           // nbr of IRQ cycles per step ( actual motorspeed  )
    #ifndef IS_32BIT
    // Remainder needed only on 8-Bit processors
	uint16_t tCycRemain;          // Remainder of division when computing tCycSteps
	uint16_t aCycRemain;          // accumulate tCycRemain when cruising
    #endif
	uintxx_t cyctXramplen;        // precompiled  tCycSteps*(rampLen+RAMPOFFSET)
    volatile uintxx_t cycCnt;     // counting cycles until a step is due
	uintxx_t cycDelay;            // delay time enable -> stepping
  #endif
  uintxx_t  stepRampLen;        // Length of ramp in steps
  uintxx_t  stepsInRamp;        // stepcounter within ramp ( counting from stop ( = 0 ): incrementing in startramp, decrementing in stopramp
  uintxx_t  deltaSteps;         // number of computed steps per real step in SPEEDDECEL
                                // max value is stepRampLen
  rampStat rampState;        	// State of stepper: stopped, cruising, acceleration/deceleration ...
  volatile long stepsFromZero;  // distance from last reference point ( always as steps in HALFSTEP mode )
                                // in FULLSTEP mode this is twice the real step number
  // bit-coded byte:
  uint8_t output  :5 ;             // PORTB(pin8-11), PORTD (pin4-7), SPI0,SPI1,SPI2,SPI3, SINGLE_PINS, A4988_PINS
  uint8_t delayActiv :1;        // enable delaytime is running
  uint8_t enable:1;             // true: enablePin=HIGH is active, false: enablePin=LOW is active
  uint8_t enableOn:1;			// true: Enable is active, can be switched off/on by user if it is
								// generally enabled ( see enablePin )

  uint8_t enablePin;            // define an enablePin, which is active while the stepper is moving 
								// 255: enable is not active, 254 no pin defined, bur enable is active for FULLSTEP and HALFSTEP (4pin steppers)
	#define NO_STEPPER_ENABLE 255 // enableOn is fixed to 'false'
	#define NO_ENABLEPIN 254
  uint8_t speedZero;			// Flag for speed is set to zero
    #define NORMALSPEED		0	// speed is not set to zero
	#define DECELSPEEDZERO	1	// lowering speed to zero ( ramping down )
	#define ZEROSPEEDACTIVE	2	// create no further steps
	#define MINSPEEDZERO   20	// real minimum speed before actually stopping ( creating no more steppulses )
	
  #ifdef FAST_PORTWRT
  portBits_t portPins[4];       // Outputpins as Portaddress and Bitmask for faster writing ( only AVR processors)
  #else
  uint8_t pins[4];                 // Outputpins as Arduino numbers
  #endif
  uint8_t lastPattern;             // only changed pins are updated ( is faster )
} stepperData_t ;

typedef union { // used output channels as bit and uint8_t
      struct {
        uint8_t pin8_11 :1;
        uint8_t pin4_7  :1;
        uint8_t spi_1    :1;
        uint8_t spi_2    :1;
        uint8_t spi_3    :1;
        uint8_t spi_4    :1;
      };
      uint8_t outputs;
    
} outUsed_t;

//////////////////////////////////////////////////////////////////////////////
class MoToStepper
{
  private:
    static outUsed_t outputsUsed;
    static byte     _stepperCount;  // number of objects ( objectcounter )
    stepperData_t _stepperData;      // Variables that are used in IRQ
    uint8_t _stepperIx;              // Objectnumber ( 0 ... MAX_STEPPER )
    long stepsRev;                   // steps per full rotation
    uintxx_t _stepSpeed10;      	// speed in steps/10sec as last set by user
    uintxx_t _lastRampLen ;         // last manually set ramplen
    uintxx_t _lastRampSpeed;        // speed when ramp was set manually
    long stepsToMove;               // from last point
    uint8_t stepMode;               // STEPDIR, FULLSTEP or HALFSTEP
    
    long getSFZ();                  // get step-distance from last reference point
    long lastSFZ;                   // last read value ( for corrections in doSteps ) 
    void _doSteps(long count, bool absPos ); // rotate count steps. abs=true means it was called from write methods

    bool _chkRunning();             // check if stepper is running
    void initialize(long,uint8_t);
    uint16_t  _setRampValues();
    uint8_t attach(uint8_t outArg, uint8_t*  ); // internal attach function ( called by one of the public attach
  public:
    // don't allow copying and moving of Stepper objects
    MoToStepper &operator= (const MoToStepper & )    =delete;
    MoToStepper &operator= (MoToStepper && )       =delete;
    MoToStepper (const MoToStepper & )             =delete;
    MoToStepper (MoToStepper && )            =delete;

    //Constuctor:
    MoToStepper(long steps);            // steps per 360 degree in HALFSTEP mode or STEPDIR Mode on ESP
    MoToStepper(long steps, uint8_t mode ); // with ESP8266 only STEPDIR is allowed
	#ifndef ESP8266 				// there are no different modes with ESP8266
                                        // mode means STEPDIR ( Step/Dir), HALFSTEP or FULLSTEP
        //Methods                                
        uint8_t attach( uint8_t,uint8_t,uint8_t,uint8_t); //single pins definition for output
        uint8_t attach(uint8_t outArg);    // stepMode defaults to halfstep
		void attachEnable( uint16_t delay ); // enable for unipolar steppers with 4 pins
    #endif
    uint8_t attach( uint8_t stepP, uint8_t dirP); // Port for step and direction in STEPDIR mode
                                    // returns 0 on failure
    void attachEnable( uint8_t enableP, uint16_t delay, bool active ); // define an enable pin and the delay (ms) between enable and starting/stopping the motor. 
                                                                          // 'active' defines if the output is HIGH or LOW to activate the motirdriver.
	bool autoEnable(bool state);	// enable/disable switching off the stepper ( only if attachEnable was called )
									// returns active state ( always false, if attachEnable  was not called )
	bool autoEnable ();				// returns active state
    void detach();                  // detach from output, motor will not move anymore
    void write(long angle);         // specify the angle in degrees, mybe pos or neg. angle is
                                    // measured from last 'setZero' point
    void write(long angle, uint8_t factor);        // factor specifies resolution of parameter angle
                                    // e.g. 10 means, 'angle' is angle in .1 degrees
	void writeSteps( long stepPos );// Go to position stepPos steps from zeropoint
    void setZero();                 // actual position is set as 0 angle (zeropoint)
    void setZero( long zeroPoint);  // new zeropoint ist zeroPoint steps apart from actual position
    void setZero( long zeroPoint, long stepsPerRotation);  // beside zero point change steps per Rotation too
    int setSpeed(int rpm10 );       // Set movement speed, rpm*10
    uintxx_t setSpeedSteps( uintxx_t speed10 ); // set speed withput changing ramp, returns ramp length
    uintxx_t setSpeedSteps( uintxx_t speed10, intxx_t rampLen ); // set speed and ramp, returns ramp length
    uintxx_t setRampLen( uintxx_t rampLen ); // set new ramplen in steps without changing speed
    int32_t getSpeedSteps();		// returns actual speed in steps/10sec ( even in ramp )
    void doSteps(long count);       // rotate count steps. May be positive or negative
                                    // angle is updated internally, so the next call to 'write'
                                    // will move to the correct angle
    void rotate(int8_t direction ); // rotate endless until 'stop',
    void stop();                    // stops moving immediately
    long stepsToDo();               // remaining steps until target position
    uint8_t moving();               // returns the remaining way to the position last set with write() in
                                    // in percentage. '0' means, that the target positio is reached
                                    // 255 means the motor is rotating endlessly
    long read();                    // actual angle from zeropoint 
    long read(byte factor);         // actual angle from zeropoint ( in fractions)
    long readSteps();               // actual distance to zeropoint in steps
    uint8_t attached();
    void prDynData();             // print actual Stepperdata
    
    //some AccelStepper compatible method names ( may be sligtly different in functionality
    void moveTo ( long stepPos )    { writeSteps( stepPos ); }
    void move(long count)           { doSteps( count ); }
    uintxx_t setMaxSpeed( uintxx_t speed ){ return setSpeedSteps( speed*10 ); }
    long distanceToGo()             { return stepsToDo(); }
    long currentPosition()          { return readSteps(); } 
};

#endif