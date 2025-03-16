/* MobaTools - Model railroad example for servos
    Switching two turnouts with servos and a relais for frog polarisation.
    The relais is switched in the middle of the path whe switching the turnout.
    It uses a FSM for each turnout.
    The example can easily expanded for more turnouts

*/

#include <MobaTools.h>

// Pin definitions, change to your needs
constexpr byte servoPin[] = { 2, 3 };
constexpr byte nbrTurnouts = sizeof(servoPin); // Number of turnouts
constexpr byte relPin[nbrTurnouts]    = {4, 5};
constexpr byte switchPin[nbrTurnouts] = {A0, A1}; // The switches must be connected between pin and Gnd
// Open switch is the straight position of the turnout

// you can set the servo positions for STRAIGHT and DEVIATE individually for each turnout:
constexpr byte straightPos[nbrTurnouts]  = { 30, 120 };  // Servo position in the straight position of the turnout.
constexpr byte deviatePos[nbrTurnouts] = { 120, 30 };  // Servo position in the deviating position of the turnout
constexpr uint16_t servoSpeed = 2000;                   // 2 sec to move from 0 to 180 deg

//FSM states:
enum fsmStates : byte { STRAIGHT, MOVE2DEV, DEVIATE, MOVE2STRAIGHT };
fsmStates turnoutState[nbrTurnouts];

MoToServo   myServo[nbrTurnouts];                              // create servo objects

void setup() {
  Serial.begin(115200);
  // initialize all objects
  Serial.print("Initializing all turnouts ...");
  for ( byte toIx = 0; toIx < nbrTurnouts; toIx++ ) {
    pinMode( relPin[toIx], OUTPUT );
    pinMode( switchPin[toIx], INPUT_PULLUP);
    myServo[toIx].attach( servoPin[toIx], true );   // with auto switch off the pulses
    myServo[toIx].setSpeedTime( servoSpeed );

    // Set initial servo position and relais according to the switches
    // If switches are not changed since last shut down, this should be the actual servo position
    if ( digitalRead( switchPin[toIx] ) ) {
      // switch is open -> 'strait' position
      myServo[toIx].write( straightPos[toIx] );
      digitalWrite( relPin[toIx], LOW );
      turnoutState[toIx] = STRAIGHT;
    } else {
      // switch is closed -> 'deviate' position
      myServo[toIx].write( deviatePos[toIx] );
      digitalWrite( relPin[toIx], HIGH );
      turnoutState[toIx] = DEVIATE;
    }
  }
  Serial.println("finished");
}


void loop() {

  for ( byte toIx = 0; toIx < nbrTurnouts; toIx++ ) {
    // turnout-FSM
    switch ( turnoutState[toIx] ) {
      case STRAIGHT: /////// Turnout is in the STRAIGHT position. wait until switch is changed
        if ( !digitalRead(switchPin[toIx] ) ) {
          // switch now in deviate position -> change servo position
          myServo[toIx].write( deviatePos[toIx] );
          turnoutState[toIx] = MOVE2DEV;
          Serial.print(toIx); Serial.print(" - moving to DEV position...");
        }
        break;
      case MOVE2DEV:  /////// Turnout is moving to deviate position
        // set relais when servo reaches middle position
        if ( myServo[toIx].moving() < 50  ) {
          // Servo has reached middle position -> switch relais
          digitalWrite( relPin[toIx], HIGH );
        }
        if ( !myServo[toIx].moving()  ) {
          // Servo has reached end position -> next FSM state
          turnoutState[toIx] = DEVIATE;
          Serial.println("finished");
        }
        break;
      case DEVIATE:  /////// Turnout is in DEVIATE position
        if ( digitalRead(switchPin[toIx] ) ) {
          // switch now is in straight position -> change servo position
          myServo[toIx].write( straightPos[toIx] );
          turnoutState[toIx] = MOVE2STRAIGHT;
          Serial.print(toIx); Serial.print(" - moving to STRAIGHT position...");
        }
        break;
      case MOVE2STRAIGHT:  /////// Turnout is moving to straight position
        // set relais when servo reaches middle position
        if ( myServo[toIx].moving() < 50  ) {
          // Servo has reached middle position -> switch relais
          digitalWrite( relPin[toIx], LOW );
        }
        if ( !myServo[toIx].moving()  ) {
          // Servo has reached end position -> next FSM state
          turnoutState[toIx] = STRAIGHT;
          Serial.println("finished");
        }
        break;
    }
  }
}
