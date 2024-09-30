/*  Position a stepper according to buttonpresses
 *  The stepper is connected by means of a step/dir driver
 *  The buttons are connected between pins and GND ( using internal pullups )
 */
 
#include <MobaTools.h>

const int stepRev = 3200;    // steps per revolution ( 1/16 microsteps )
// adjust stepper pins to your needs
const byte dirPin = 5;
const byte stepPin = 6;
const byte enablePin = 7;

// create stepper object
MoToStepper myStepper( stepRev, STEPDIR );

// create button objects
const byte buttonPins[] = { A0,A1,A2,A3,A4 };               // adjust pins to your needs
const long stepperPositions[] = { 0, 90, 180, 270, 360 };   // in degrees, must be same number of elements as buttonPins
const byte buttonCnt = sizeof(buttonPins);                  // number of buttons/positions

MoToButtons myButtons( buttonPins, buttonCnt, 20, 500 );

void setup() {
  myStepper.attach(stepPin, dirPin);
  myStepper.setSpeed( 600 );                   // = 60 rpm
  myStepper.setRampLen( 100 );                        // 100 steps to achive set speed
  myStepper.attachEnable( enablePin, 100, LOW );    // if you want to switch off power when stepper reached position
  // a homing procedure may be needed here
}

void loop() {
  myButtons.processButtons();                // Check buttonstates
  
  for( byte pos= 0; pos<buttonCnt; pos ++ ) {
    if ( myButtons.pressed(pos) ) {
      // Button was pressed, move stepper to the according position
      myStepper.write( stepperPositions[pos] );
    }
  }
}
