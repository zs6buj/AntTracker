#include <MobaTools.h>
/* Demo: non blocking timers without delay()
 *  In principle, the 'MoToTimer' works like a kitchen timer.
 *  You wind it up to a certain time and then it runs back to 0. 
 *  Unlike the kitchen alarm clock, however, it does not ring.
 *  You have to check cyclically to see if the time has expired. But that fits
 *  perfectly with the principle of the 'loop', i.e. a cyclical query in an endless loop.
 *  Function calls:
 *  MoToTimer.setTime( long Laufzeit );  sets the time in ms
 *  bool = MoToTimer.running();       == true as long as the time is still running, 
 *  bool = MoToTimer.expired();         This call returns true only once if the time has expired
 *                                      succeding calls will return 'false' again.
 *  
 *  In this demo, 2 leds are switched on with one button each for different times.
 *  At each expiry of the time, a message is output once on the serial monitor.
 *  The buttons must be connected between the pin and Gnd
*/

const byte led1P =  5;
const byte led2P =  6;

const byte button1Pin = A0;
const byte button2Pin = A1;


MoToTimer timer1;
MoToTimer timer2;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // only needed for Arduino Micro/Leonardo
  pinMode(led1P, OUTPUT);
  pinMode(led2P, OUTPUT);
  pinMode(button1Pin, INPUT_PULLUP );
  pinMode(button2Pin, INPUT_PULLUP );
}

void loop() {
  // -------- switching the first led ------------------
  if ( !digitalRead(button1Pin) && !timer1.running() ) {
    Serial.println("start first timer");
    // Switch led on and start timer
    timer1.setTime( 2000 );
    digitalWrite( led1P, HIGH );
  }
  if ( timer1.expired() ) {
    Serial.println("first timer expired");
    digitalWrite( led1P, LOW );
  }
  // -------- switching the second led ------------------
  if ( !digitalRead(button2Pin) && !timer2.running() ) {
    // Switch led on and start timer
    Serial.println("start second timer");
    timer2.setTime( 1500 );
    digitalWrite( led2P, HIGH );
  }
  if ( timer2.expired() ) {
    Serial.println("second timer expired");
    digitalWrite( led2P, LOW );
  }
  
}
