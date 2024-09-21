/* Beipiel für die MobaTools / Example for MobaTools
 *  Einen Schrittpmotor in regelmäßigen Abständen bewegen
 *  Move a stepper motor in regular intervals
 */
#include <MobaTools.h>

// Adjust pins, steps and time as needed
const byte stepPin = 9;
const byte dirPin  = 8;
const int stepsProUmdr = 800;   // Steps per Revolution ( exammple with 1/4 microsteps )
int stepsProTick = 400;         // Steps to be executed every intervall
const int intervallZeit = 1000; // Steps are executed every second
const byte intervallMax = 8;     // change direction every intervallMax intervals

byte intervallZaehler;

MoToStepper myStepper ( stepsProUmdr, STEPDIR );
MoToTimebase intervall;

void setup() {
    myStepper.attach( stepPin, dirPin );
    myStepper.setSpeed( 1200 );  // 120 Umdrehungen/Min
    myStepper.setRampLen( 20 );
    intervall.setBasetime( intervallZeit );
}

void loop() {
    if ( intervall.tick() ) {
        myStepper.doSteps( stepsProTick );
        if ( ++intervallZaehler >= intervallMax ) {
            intervallZaehler = 0;
            stepsProTick = -stepsProTick;
        }
     //delay(1500);
    }
    
    // hier können weitere Aufgaben erledigt werden / additional tasks to be done
    // Der Sketch wird weder während der Wartezeit noch bei der Stepausführung blockiert.
    // The sketch is not blocked during the waiting time or during step execution.
}
