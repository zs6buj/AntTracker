/* An Example with 3 steppers ( 2 unipolar, 1 bipolar ).
 *  A reference run is started in setup.  2 limit switches are required for this.
 *  This example does not run on ESP8266
 */
 
#include <MobaTools.h>
MoToTimer Pause;
MoToStepper Step_X(4096);                    // X-Achse, unipolarer Schrittmotor 28BYJ-48
MoToStepper Step_Y(4096);                    // Y-Achse, unipolarer Schrittmotor 28BYJ-48
MoToStepper Step_Z(200, STEPDIR);            // Z-Achse, bipolarer Schrittmotor mit Treiber wie A4988, DRV8825 oder vergleichbare
const byte pinRef[] = { A0, A1 };            // pinRef_X, pinRef_Y
const byte nbrOfButtons = sizeof(pinRef);    // Anzahl der angeschlossenen Taster
enum {X_AXIS=0, Y_AXIS=1};

void setup() {
  for (byte i = 0; i < nbrOfButtons; i++)  {
    pinMode(pinRef[i], INPUT_PULLUP);        // gedrückt = LOW
  }
  Step_X.attach( 12, 11, 10, 9 );            // IN1, IN2, IN3, IN4
  Step_Y.attach( 5, 4, 3, 2 );               // IN1, IN2, IN3, IN4

  Step_Z.attach( 7, 8 );                     // STEPpin, DIRpin
  Step_Z.setSpeed( 800 );                    // = 80 U/Min (motorspezifisch)
  Step_Z.setRampLen( 50 );                   // Beschleunigung (motorspezifisch)
  Step_Z.write(360);                         // Winkel 360 Grad drehen
  Pause.setTime( 2500 );                     // Dreh- und Pausenzeit
}

void loop() {
  enum {SEEK_ZERO, MOVE_FORWARD, MOVE_BACWARD};
  static byte step_X = SEEK_ZERO;          // Schrittkettenstatus X-Achse
  static byte step_Y = SEEK_ZERO;          // Schrittkettenstatus Y-Achse
  static int16_t angle = 360;

  //-- X-Achse
  switch (step_X) {
    case SEEK_ZERO:
      if (seekZeropoint(Step_X, X_AXIS)) {
        step_X = MOVE_FORWARD;
      }
      break;
    case MOVE_FORWARD:
      if ( !Step_X.moving() ) {              // warten bis die Bewegung abgeschlossen ist
        Step_X.setSpeed( 200 );              // = 20 U/Min (motorspezifisch)
        Step_X.setRampLen( 100 );            // Beschleunigung (motorspezifisch)
        Step_X.writeSteps(4096);
        step_X = MOVE_BACWARD;
      }
      break;
    case MOVE_BACWARD:
      if ( !Step_X.moving() ) {              // warten bis die Bewegung abgeschlossen ist
        Step_X.writeSteps(-4096);
        step_X = MOVE_FORWARD;
      }
      break;
  }
  //-- Y-Achse
  switch (step_Y) {
    case SEEK_ZERO:
      if (seekZeropoint(Step_Y, Y_AXIS)) {
        step_Y = MOVE_FORWARD;
      }
      break;
    case MOVE_FORWARD:
      if ( !Step_Y.moving() ) {              // warten bis die Bewegung abgeschlossen ist
        Step_Y.setSpeed( 150 );              // = 20 U/Min (motorspezifisch)
        Step_Y.setRampLen( 100 );            // Beschleunigung (motorspezifisch)
        Step_Y.writeSteps(4096);             // Bewegung starten
        step_Y = MOVE_BACWARD;
      }
      break;
    case MOVE_BACWARD:
      if ( !Step_Y.moving() ) {              // warten bis die Bewegung abgeschlossen ist
        Step_Y.writeSteps(-4096);            // Bewegung starten
        step_Y = MOVE_FORWARD;
      }
      break;
  }
  //-- Z-Achse
  if ( !Step_Z.moving() ) {                  // warten bis die Bewegung abgeschlossen ist
    if ( !Pause.running() ) {                // warten bis Ablauf der Zeit
      Pause.setTime( 5000 );                 // Zeit für Bewegung und Pause
      angle *= -1;                          // andere Richtung
      Step_Z.write(angle);                  // Bewegung starten
    }
  }

}

bool seekZeropoint(MoToStepper &Step, byte axis) {     // keine Kopie, sondern eine Referenz des Schrittmotorobjektes
  enum {REFLEAVE, REFREACH, REACHZERO};
  static byte step[] = {REFLEAVE,REFLEAVE};        // Schrittkettenstatus

  switch (step[axis]) {
    case REFLEAVE:
      Step.setSpeed( 50 );                   // = 5 U/Min (motorspezifisch) Schleichfahrt
      Step.setRampLen( 5 );                  // Beschleunigung (motorspezifisch)
      Step.rotate(1);                        // vom Referentpunkt runterbewegen
      if (digitalRead(pinRef[axis])) {      // Referenzsensor nicht (mehr) betätigt
        Step.doSteps(100);                   // mit etwas Entfernung anhalten
        step[axis]++;
      }
      break;
    case REFREACH:
      if ( !Step.moving() ) {                // warten bis die Bewegung abgeschlossen ist
        Step.rotate(-1);                     // Richtung Referentpunkt bewegen
        step[axis]++;
      }
      break;
    case REACHZERO:
      if (!digitalRead(pinRef[axis])) {     // Ref_X betätigt
        Step.setZero(100);                   // setze Nullpunkt 100 Schritte von Referenzpunkt entfernt
        Step.writeSteps(0);                  // zum Nullpunkt bewegen
        step[axis]++;
      }
      break;
    default:
      step[axis] = REFLEAVE;
      return true;
  }
  return false;
}
