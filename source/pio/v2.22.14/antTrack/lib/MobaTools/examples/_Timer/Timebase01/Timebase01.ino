// Blink 4 leds in random intervals with the Timebase class.
// Blinking starts with first press of the correspondig button 
// and can be stopped and restarted by pressing this button.
// The blink frequency is determined at the first start.

#include <MobaTools.h>

const byte ledPins[] = {2,3,4,5};
const byte buttonPins[] = { A0,A1,A2,A3 };
const byte ledCnt = sizeof( ledPins );      // ledPins must be type of byte
MoToTimebase ledFlasher[ledCnt];
MoToButtons Buttons( buttonPins, ledCnt, 20, 500 );

void setup() {
    // Set ledPins to OUTPUT. The mode of the button pins is set
    // by the MoToButtons constructor.
    for ( auto pin : ledPins  ) {
        pinMode( pin, OUTPUT );
        digitalWrite( pin, HIGH );  // initial led test 
        delay(500);
        digitalWrite( pin, LOW );
    }
}

void loop() {
    Buttons.processButtons();
    
    for ( byte i=0; i<ledCnt; i++ ) {
        if ( ledFlasher[i].tick() ) {
            digitalWrite( ledPins[i], !digitalRead( ledPins[i] ) );
        }
        if ( Buttons.pressed(i) ) {
            if ( ledFlasher[i].inactive() ) {
                // first Start defines interval
                randomSeed( micros() );
                ledFlasher[i].setBasetime( random( 100, 2000 ) ); 
                digitalWrite( ledPins[i], HIGH );
            } else {
                if ( ledFlasher[i].running() ) {
                    ledFlasher[i].stop();
                    digitalWrite( ledPins[i], LOW );
                } else {
                    ledFlasher[i].start();
                    digitalWrite( ledPins[i], HIGH );
                }
            }
        }
    }
}
