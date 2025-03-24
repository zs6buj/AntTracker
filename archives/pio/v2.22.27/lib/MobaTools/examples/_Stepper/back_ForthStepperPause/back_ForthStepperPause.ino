/*  Example for MobaTools
    Moving a stepper back and forth
*/
#include <MobaTools.h>

// Adjust pins, steps and time as needed
const byte stepPin = 9;
const byte dirPin  = 8;
const int stepsPerRev = 800;   // Steps per Revolution ( example with 1/4 microsteps )
const long  targetPos = 1600;         // stepper moves between 0 and targetpos
long nextPos;


MoToStepper myStepper ( stepsPerRev, STEPDIR );
MoToTimer stepperPause;                    // Pause between stepper moves
bool stepperRunning;

void setup() {
  myStepper.attach( stepPin, dirPin );
  myStepper.setSpeed( 600 );  // 60 Rev/Min ( if stepsPerRev is set correctly )
  myStepper.setRampLen( 200 );
  stepperRunning = true;
}

void loop() {
  if ( stepperRunning ) {
    // Wait till stepper has reached target, then set pause time
    if ( !myStepper.moving() ) {
      // stepper has reached target, start pause
      stepperPause.setTime( 1000 );
      stepperRunning = false;
    }
  } else {
    // stepper doesn't move, wait till stepperPause time expires
    if ( stepperPause.expired() ) {
      // stepperPause time expired. Start stepper in opposite direction
      if ( nextPos == 0 ) {
        nextPos = targetPos;
      } else {
        nextPos = 0;
      }
      myStepper.moveTo( nextPos );
      stepperRunning = true;
    }
  }

  // The sketch is not blocked while the stepper is moving nor while it is stopped.
  // Other nonblocking  tasks can be added here
}
