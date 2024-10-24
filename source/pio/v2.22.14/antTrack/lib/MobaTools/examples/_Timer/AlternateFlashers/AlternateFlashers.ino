#include <MobaTools.h>

/* Example: Alternately flashing red lights for a level crossing
    This example is a bit more complex and shows the realisation of an alternating flasher 
    that is switched on and off by a switch. As usual with alternating flashers at a 
    level crossing ( at least in germany), the two lamps start at the same time and then flash alternately.
    This example also uses the MoToTimer to realise time delays. 
*/

// defining the pins
const int flasher1Pin =  5;  // The two leds
const int flasher2Pin =  6;  // of the flasher
const int switchP =  7;  // The switch to turn it on and off

// additional constants
const int wbZykl = 1100;   // Cycle time of the alternating flasher
const int wbSoft = 400;    // Fade-in/fade-out time of the leds

// State of the flasher
byte aflState = 0;   
#define   AFL_OFF      0 	// Both leds off
#define   AFL_START    1   	// Starting: both leds on
#define   AFL_FLASHES  2   	// flashing alternately
byte ledState;              // HIGH : led1 is on, LOW led2 ist on

MoToSoftLed flasher1;
MoToSoftLed flasher2;

MoToTimer flashTimer;

void setup() {
    pinMode(switchP, INPUT_PULLUP); 
    flasher1.attach(flasher1Pin);  // pins are set to OUTPUT in class
    flasher2.attach(flasher2Pin); 
    flasher1.riseTime( wbSoft );    //  ms
    flasher2.riseTime( wbSoft );    //  ms
}

void loop() {
    // Alternate flasher
    switch (aflState) {
      case AFL_OFF:
        // both leds off, waiting for switching on
        if ( digitalRead(switchP) == HIGH && flashTimer.running() == false ) {
            // switch both leds on, set time for both leds on
            flasher1.on();
            flasher2.on();
            flashTimer.setTime( wbSoft );
            aflState = AFL_START;
        }
        break;
      case AFL_START:
        // starting: sitch first led off if time expires
        if ( flashTimer.running() == false ) {
            // time expired, switch to flashing
            ledState = HIGH;
            flasher2.off();
            flashTimer.setTime(wbZykl/2); // set flash time
            aflState = AFL_FLASHES;
        }
        break;
      case AFL_FLASHES:
        if ( flashTimer.running() == false ) {
            flashTimer.setTime(wbZykl/2); // set flash time
            if ( ledState == LOW ) {
                flasher1.on();
                flasher2.off();
                ledState = HIGH;
            } else {
                ledState = LOW;
                flasher2.on();
                flasher1.off();
            }
        }
        if ( digitalRead(switchP) == LOW ) {
            // switch off
            flasher1.off();
            flasher2.off();
            aflState = AFL_OFF;
            flashTimer.setTime(wbZykl);   // minmum off time to debounce the switch
        }
        break;
            
    } // End switch 
}
