#ifndef MOTOSERVO_H
#define MOTOSERVO_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2020 All right reserved.

  Definitions and declarations for the servo part of MobaTools
*/

/* Some definitions for the servo class: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	increments (INC) is the internal timebase for all time computing ( it's a virtual value ). All time values ( e.g. pulselength ) are internally stored based on this value.
	tics (TIC) ist the physical resolutin of the servo timer. This must always be an integer multiple of INC.
*/

// defines for servos
#define Servo2	MoToServo		// Kompatibilität zu Version 01 und 02
#define AUTOOFF 1               // 2nd Parameter for servo.attach to switch off pulses in standstill
#define OVLMARGIN           280     // Overlap margin ( Overlap is MINPULSEWIDTH - OVLMARGIN )
#define OVL_TICS       ( ( MINPULSEWIDTH - OVLMARGIN ) * TICS_PER_MICROSECOND )
#define MARGINTICS      ( OVLMARGIN * TICS_PER_MICROSECOND )
#define MAX_SERVOS  16  

#define MINPULSETICS    (MINPULSEWIDTH * TICS_PER_MICROSECOND)
#define MAXPULSETICS    (MAXPULSEWIDTH * TICS_PER_MICROSECOND)

#define OFF_COUNT       50  // if autoOff is set, a pulse is switched off, if it length does not change for
                            // OFF_COUNT cycles ( = OFF_COUNT * 20ms )
#define FIRST_PULSE     100 // first pulse starts 200 tics after timer overflow, so we do not compete
                            // with overflow IRQ

/* Regarding servo Speed:
One 'Speed tic' in setSpeed should be about 0.125 µs. That's not the real timer tic. So in reality the 
pulse length may not change with every cycle (20ms). But this way speedresolution is independent from real timer tics and boards.
Pulse length are always stored in inc. Only when creating the real pulse ( in IRQ ) this value is devided by INC_PER_TIC to get the real 'timer tics' ( this is different for ESP )
 All position values in tics are multiplied by this factor.  Only when computing the next interrupt time 
 the values are divided by this value again to get the real 'timer tics'
 The following values are defined in the board-specific .h files:
 
 INC_PER_MICROSECOND  on most boards this is 8 (according to 0.125µs) but it must be selected so that
 INC_PER_TIC		  ( increments per timer tic ) is an integer value.
 AS_Speed2Inc(speed)  mostly returns simply speed, but if INC_PER_MICROSECOND is not 8, it is different.
 time2tic(pulse)	  returns increments (pulsewidth in µs)
 tic2time(tics)		  returns pulswidth in  µs
 
 These defines are different for ESP8266 and ESP32, because these boards don't use the standard timer IRQ of all other boards ( see board specific .h files ).

*/
////////////////////////////////////////////////////////////////////////////////////
//void ISR_Servo( void *arg );

// global servo data ( used in ISR )
struct servoData_t {
  struct servoData_t* prevServoDataP;
  uint8_t servoIx :6 ;  // Servo number. On ESP32 this is also the nuber of  the PWM timer
  uint8_t on   :1 ;     // True: create pulse
  uint8_t noAutoff :1;  // don't switch pulses off automatically
                        // on ESP32 'soll' 'ist' and 'inc' are in duty values (  0... DUTY100 )
  int soll;             // Position, die der Servo anfahren soll ( in Tics ). -1: not initialized
  volatile int ist;     // Position, die der Servo derzeit einnimt ( in Tics )
  int inc;              // Schrittweite je Zyklus um Ist an Soll anzugleichen( in Tics )
  uint8_t offcnt;       // counter to switch off pulses if length doesn't change
  #ifdef FAST_PORTWRT
  volatile uint8_t* portAdr;     // port adress related to pin number
  volatile uint8_t  bitMask;     // bitmask related to pin number
  #endif
  uint8_t pin     ;     // pin
  int8_t pwmNbr;        // pwm channel on ESP32 , -1 means not attached on all platforms
						// on RP2040 its slice_nmbr+channel ( channel 0/1 in LSB )
} ;

////////////////////////////////////////////////////////////////////////////////////////
class MoToServo
{
  private:
    int16_t _lastPos;     // startingpoint of movement
    //uint8_t pin;
    //uint8_t _angle;       // in degrees
    uint16_t _minPw;       // minimum pulse, uS units  
    uint16_t _maxPw;       // maximum pulse, uS units
    servoData_t _servoData;  // Servo data to be used in ISR

	public:
    // don't allow copying and moving of Servo objects
    MoToServo &operator= (const MoToServo & )   =delete;
    MoToServo &operator= (MoToServo && )        =delete;
    MoToServo (const MoToServo & )              =delete;
    MoToServo (MoToServo && )                   =delete;
    
    MoToServo();
    uint8_t attach(int pin); // attach to a pin, sets pinMode, returns 0 on failure, won't
                             // position the servo until a subsequent write() happens
    uint8_t attach( int pin, bool autoOff );        // automatic switch off pulses with constant length
    uint8_t attach(int pin, uint16_t pos0, uint16_t pos180 ); // also sets position values (in us) for angele 0 and 180
    uint8_t attach(int pin, uint16_t pos0, uint16_t pos180, bool autoOff );
    void detach();
    void write(uint16_t);         // specify the angle in degrees, 0 to 180. Values obove 180 are interpreted
                             // as microseconds, limited to MaximumPulse and MinimumPulse
    void setSpeed(int);      // Set movement speed, the higher the faster
                             // Zero means no speed control (default)
    void setSpeed(int,bool); // Set compatibility-Flag (true= compatibility with version V08 and earlier)
    #define HIGHRES 0
    #define SPEEDV08 1
	void setSpeedTime(uint16_t  );  // set Speed as time between 0...180° in milliseconds
    
    uint8_t moving();        // returns the remaining Way to the angle last set with write() in
                             // in percentage. '0' means, that the angle is reached
    uint8_t read();          // current position in degrees (0...180)
    uint16_t  readMicroseconds();// current pulsewidth in microseconds
    uint8_t attached();
    void setMinimumPulse(uint16_t);  // pulse length for 0 degrees in microseconds, 700uS default
    void setMaximumPulse(uint16_t);  // pulse length for 180 degrees in microseconds, 2300uS default
};

#endif