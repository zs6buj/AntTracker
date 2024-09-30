/* ====== minimumStepper =======================================
 *  Bare minimum to get a stepper with step/dir driver turning
 */
#include <MobaTools.h>
// Stepper connections - Please adapt to your own needs.
const byte stepPin = D6;
const byte dirPin = D5;

const int stepsPerRev = 200;    // Steps per revolution - may need to be adjusted

MoToStepper stepper1( stepsPerRev, STEPDIR );  // create a stepper instance

void setup() {
  stepper1.attach( stepPin, dirPin );
  stepper1.setSpeed( 300 );              // 30 rev/min (if stepsPerRev is set correctly)
  stepper1.setRampLen( stepsPerRev / 2); // Ramp length is 1/2 revolution
  stepper1.rotate(1);                    // start turning, 1=vorward, -1=backwards                    
}

void loop() {
}
