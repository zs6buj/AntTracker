////////// Example 3 for MoToStepper - attaching a bipolar stepper with step/dir and enable ////////////

/*  Example for the control of a bipolar stepper with 4 buttons and a potentiometer ( for the speed ).
    In this example, besides the stepper class (MoToStepper), the classes MoToButtons and MoToTimebase 
    are also used.
    The stepper current is switched off, if the stepper is not moving.
    The buttons must switch to Gnd, no resistors are needed.
    Button1: Rotates right as long as the button is pressed and held
    Button2: Rotates left as long as the button is pressed and held
    Button3: One turn to the right
    Button4: One turn to the left
*/

/*  Beispiel für die Ansteuerung eines bipolaren Steppers über 4 Taster und 
    ein Poti ( für die Geschwindigkeit )
    In diesem Beispiel werden neben der Stepperklasse (MoToStepper), auch die MoToButtons
    und der MoToTimebase genutzt.
    Der Spulenstrom wird abgeschaltet, wenn der Stepper nicht läuft.
    Die Taster müssen gegen Gnd schalten, es sind keine Widerstände notwendig 
    Button1:  Dreht rechts, solange Taster gedrückt
    Button2:  Dreht links, solange Taster gedrückt
    Button3:  1 Umdrehung rechts
    Button4:  1 Umdrehung links
*/

#define MAX8BUTTONS // spart Speicher, da nur 4 Taster benötigt werden (saves RAM)
#include <MobaTools.h>
// Pindefinitions - change to your needs
const byte dirPin       = 5;
const byte stepPin      = 6;
const byte enaPin       = 7;
const byte button1Pin   = A1;
const byte button2Pin   = A2;
const byte button3Pin   = A3;
const byte button4Pin   = A4;

const byte potPin       = A0;   // must be an analog input


const int STEPS_REVOLUTION = 800;
//Stepper einrichten ( 800 Schritte / Umdrehung - 1/4 Microstep )
MoToStepper myStepper( STEPS_REVOLUTION, STEPDIR );  // 800 Steps/ Umdrehung

// Taster einrichten 
enum { Button1=0, Button2, Button3, Button4 } ; // Den Tasternamen die Indizes 0...3 zuordnen
const byte buttonPins[] = { button1Pin, button2Pin, button3Pin, button4Pin };    // muss als byte definiert sein, damit ein enfaches sizeof funktioniert
MoToButtons button( buttonPins, sizeof(buttonPins), 20, 500 );

MoToTimebase speedIntervall;    // Zeitinterval zum Auslesen des Speedpotentiometers
                                // the speed pot ist read only every 'speedintervall' ms

int vspeed = 0;                 //Steppergeschwindigkeit in U/min*10

void setup()
{
  myStepper.attach( stepPin, dirPin );
  myStepper.attachEnable( enaPin, 10, LOW );        // Enable Pin aktivieren ( LOW=aktiv )
  myStepper.setSpeed( 200 );
  myStepper.setRampLen( 100 );                       // Rampenlänge 100 Steps bei 20U/min
  speedIntervall.setBasetime( 100 );                  // 100ms Tickerzeit
}

void loop() {
  button.processButtons();          // Taster einlesen und bearbeiten

  // Speed alle 100ms neu einlesen und setzen
  if ( speedIntervall.tick() ) {
    // wird alle 100ms aufgerufen ( Tickerzeit = 100ms im setup() )
    vspeed = map((analogRead(potPin)), 0, 1023, 20, 1800);  //Poti mappen auf 2 ... 180 Umdr/Min
    //min speed =2 and max speed =180 rpm
    myStepper.setSpeed( vspeed );
  }

  // Drehen rechtsrum
  if (button.pressed(Button1) ) {
    //Taster1 gedrückt
    myStepper.rotate( 1 );          // Stepper dreht vorwärts
  }
  if ( button.released(Button1) ) {
    //Taster1 losgelassen
    myStepper.rotate(0);             // Stepper stoppt
  }

  //Drehen linksrum
  if (button.pressed(Button2) ) {
    //Taster2 gedrückt
    myStepper.rotate( -1 );         // Stepper dreht rückwärts
  }
  if ( button.released(Button2) ) {
    //Taster2 losgelassen 
    myStepper.rotate(0);            // Stepper stoppt
  }

  // 1 Umdrehung rechts {
  if (button.pressed(Button3) ) {
    //Taster3 wurde gedrückt
    myStepper.doSteps(STEPS_REVOLUTION); 
  }

  // 1 Umdrehung links 
  if (button.pressed(Button4) ) {
    //Taster4 wurde gedrückt
    myStepper.doSteps(-STEPS_REVOLUTION);
  }
}
