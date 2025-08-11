#include <MobaTools.h>
/* Demo to move a servo slowly
   In this example a servo sweeps slowly between two positions as long as
   a button is pressed. If the button is released, it stops immediately and
   starts again from that position if the button is pressed again.
   The sketch does not block and can be extended to do other tasks.
*/

// The button must be connected between pin and Gnd
const int buttonPin1 = A2;  //button1 Pin 2
const int servoPin   = 3 ;  // The servo is connected to this pin

// Position to sweep
const byte target1 = 30;
const byte target2 = 120;

bool buttonPressed;
byte targetPos = target1;    // actual targetposition for the servo

MoToServo myServo;

void setup() {
  pinMode(buttonPin1, INPUT_PULLUP);

  myServo.attach(servoPin);
  myServo.setSpeed( 5 );    // set speed of servo
}

void loop() {
  buttonPressed =  !digitalRead(buttonPin1); // 'ButtonPressed is 'true' if pin is LOW

  if (buttonPressed && not myServo.moving() ) {
    if ( myServo.read() == target1 ) targetPos = target2;
    if ( myServo.read() == target2 ) targetPos = target1;
    myServo.write(targetPos); //will move slowly
  }

  if ( !buttonPressed ) myServo.write( myServo.read() );    // stop immediately
}
