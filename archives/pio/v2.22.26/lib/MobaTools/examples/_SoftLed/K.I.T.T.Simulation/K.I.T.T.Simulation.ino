// K.I.T.T. Simulation ( do you remember 'Knight Rider' ? )
// Simulation of the lights at the front of K.I.T.T.
// There are 8 leds in a line
//
// an example of the MobaTools Library

#define MAX8BUTTONS
#include <MobaTools.h>

//  Arrays with bitcoded sequence of LED patterns. At the end it starts over again.
//  first byte is timedelay between steps to next pattern, 0xff marks the end
// feel free to add more ... ;-)
const byte simu1[] = { 100, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0xff };
const byte simu2[] = { 200, 0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81, 0xff };
const byte simu3[] = { 150, 0x02, 0x04, 0x08, 0x11, 0x22, 0x44, 0x88, 0x10, 0x20, 0x40, 0x80, 0x00,
                       0x80, 0x40, 0x20, 0x10, 0x88, 0x44, 0x22, 0x11, 0x08, 0x04, 0x02, 0x01, 0x00, 0xff };
const byte *simPtr[] = { simu1, simu2, simu3 };

const byte simCount = sizeof(simPtr) / sizeof(simPtr[0]);   // amount of selectable simulations
byte simuIx;                                                // Index of actual simulation

#ifdef ESP32
const byte taster = 12;                                     // a press switches to next simulation
const byte ledPins[] = {15, 2, 4, 16, 17, 5, 18, 19};       // pins for ESP32 dev module, must be 8 leds
#else
const byte taster = 2;                                     // a press switches to next simulation
const byte ledPins[] = {3, 4, 5, 6, 7, 8, 9, 10};            // pins for AVR ( UNO,Nano,Mini,Leonardo.., must be 8 leds
#endif
const byte pinCount = sizeof(ledPins);

button_t getHW( void ) {                                    // User-Callback-Funktion for reading buttonstate
  return (button_t) !digitalRead(taster);
}

MoToButtons Taster( getHW, 20, 500 );
MoToSoftLed myLeds[pinCount];
MoToTimer myTimer;

void setup() {
  pinMode(taster, INPUT_PULLUP);                            // button must switch to ground

  for (byte led = 0; led < pinCount; led++) {               // assign the pins to the softleds
    myLeds[led].attach(ledPins[led]);
  }
}

void loop() {
  Taster.processButtons();
  if (Taster.pressed(0)) {
    if ( (++simuIx) >=  simCount ) simuIx = 0;              // button pressed: switch to next type of simulation
  }
  lauflicht(simPtr[simuIx]);                                // call simulation with pointer to active sim-Array
}

void lauflicht( const byte *actSim ) {
  static byte *lastSim;                                     // to check if Simulation has changed
  static byte lastPattern;                                  // leds to switch off ( last led pattern )
  static byte patternIx;                                    // index of active pattern

  if ( lastSim != actSim ) {                                // check if type of simulation changed
    lastPattern = 1;                                        // is new Simulation --> reset indexes
    patternIx = 1;
    lastSim = (byte *)actSim;
  }
  
  if (!myTimer.running()) {                                 // if its time for next pattern ...
    for ( byte ledIx = 0; ledIx < pinCount; ledIx++ ) {     // switch off all active leds
      if ( (1 << ledIx) & lastPattern ) {
        myLeds[ledIx].riseTime( actSim[0] * 2);             // use a long risetime ( actually its decaytime here )  ...
        myLeds[ledIx].off();                                // for switching off
      }
    }

    if ( actSim[++patternIx] == 0xff ) patternIx = 1;       // switch to next pattern, start over at the end ...
    for ( byte ledIx = 0; ledIx < pinCount; ledIx++ ) {     // and turn on leds according to new pattern
      if ( (1 << ledIx) & actSim[patternIx] ) {
        myLeds[ledIx].riseTime( 5);                         // use a short risetime ...
        myLeds[ledIx].on();                                 // for switching on
      }     
    }
    lastPattern = actSim[patternIx];                        // remember pattern for switching off of leds
    myTimer.setTime(actSim[0]);                             // set delay until next pattern
  }
}
