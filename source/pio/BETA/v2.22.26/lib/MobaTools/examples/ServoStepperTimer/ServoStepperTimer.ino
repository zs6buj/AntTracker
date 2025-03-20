#include <MobaTools.h>
/* This is an example using all classes of MobaTools
    - 2 servos and 2 steppers and 2 buttons are used
    - 1st servos sweeps alle the time between 0° and 180° witth different speeds
    - 2nd servo sweeps every time when a button is pressed. if the button is
      pressed while it sweeps its direction is changed immediately.
    - 1. stepper moves every 10 seconds for one revolution. It is disabled when not moving
    - 2. stepper moves to position 2*(steps/rev) when a button is pressed short. If the button
      is pressed long it moves to position -2*(steps/rev). If the button is pressed
      while its moving, it imediately moves to position 0.
    -
*/
// Pin definitions - adjsut to your own needs
// The buttons must be connected between pin and Gnd
const byte servoButton = A5;
const byte stepperButton = A4;

const byte servo1Pin = 3;
const byte servo2Pin = 4;

const byte step1DirPin = 8;
const byte step1StepPin = 9;
const byte step1EnaPin = 12;

const byte step2DirPin = 5;
const byte step2StepPin = 6;
const byte step2EnaPin = 7;

const byte led1Pin = 13;



// Positions and times to sweep servo
const byte servoTarget1 = 0;
const byte servoTarget2 = 180;
const int sweepTime1 = 1000;    // 1000ms to sweep from 0 to 180
const int sweepTime2 = 500;     // 500ms to sweep from 180 to 0
byte lastTarget = servoTarget1;

// -------------  creating the MobaTools objects: ----------------------------
// the two servos
MoToServo sweepServo;
MoToServo buttonServo;

// Steppers
const int aStepRev = 800;                   // steps per revolution for autoStepper
const int mStepRev = 800;                   // steps per revolution for buttonStepper
MoToStepper autoStepper( aStepRev, STEPDIR );    // Stepper that moves automaticalle based on time
MoToStepper buttonStepper(mStepRev, STEPDIR );   // Stepper that moves according to buttonpresses

// Timer
MoToTimebase stepperTimer;  // for timed movement of autoStepper

// Buttons
const byte buttonPins[] = { servoButton, stepperButton };
const byte buttonCount = sizeof(buttonPins);  // buttonPins must be of type byte for this to work
MoToButtons myButtons( buttonPins, buttonCount, 20, 500 );
enum buttontype : byte { SRVBUT, STPBUT };

//////////////////////////////////////////////////////////////////////////////

void sweepServoFSM();       // Sweeping servo
void buttonServoFSM();      // Servo controlled by button SRVBUT
void autoStepperFSM();      // sweeping stepper
void buttonStepperFSM();    // Stepper controlled by button STPBUT

void setup() {
  Serial.begin(115200);
  while (!Serial);          // needed for boards with native USB
  Serial.println("Starting Demo");

  // initialize the servos
  sweepServo.attach( servo1Pin, 1000, 2000 );
  sweepServo.write(lastTarget = servoTarget1);
  buttonServo.attach( servo2Pin, 1000, 2000 );
  buttonServo.write(servoTarget1);

  // Steppers are initialised in their respective tabs
}

void loop() {
  myButtons.processButtons();
  sweepSServoFSM();
  buttonServoFSM(); 
  autoStepperFSM();
  buttonStepperFSM();
}
