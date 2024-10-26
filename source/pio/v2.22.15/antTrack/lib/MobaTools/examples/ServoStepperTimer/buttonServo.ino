//  buttonServo sweeps every time when a button is pressed. If the button is
//  pressed while it sweeps its direction is changed immediately.

enum : byte {  ATPOS1, TOPOS2, ATPOS2, TOPOS1 };
void buttonServoFSM() {
  static byte srvState = ATPOS1;
  const byte POS1 = 0;
  const byte POS2 = 180;
  switch ( srvState ) {
    case ATPOS1: // wait for buttonpress
      if ( myButtons.pressed(SRVBUT) ) {
        Serial.println( "Button pressed -> POS2");
        buttonServo.setSpeedTime(3000);
        buttonServo.write(POS2);
        srvState = TOPOS2;
      }
      break;
    case TOPOS2:   // moving to pos 2, if button pressed, go back
      if ( myButtons.pressed(SRVBUT) ) {
        Serial.println( "Button presse backt to POS1");
        buttonServo.setSpeedTime(2000);
        buttonServo.write(POS1);
        srvState = TOPOS1;
      }
      if ( !buttonServo.moving() ) {
        // we reached the position 2
        srvState = ATPOS2;
      }
      break;
    case ATPOS2: // wait for buttonpress
      if ( myButtons.pressed(SRVBUT) ) {
        Serial.println( "Button pressed -> POS1");
        buttonServo.setSpeedTime(2000);
        buttonServo.write(POS1);
        srvState = TOPOS1;
      }
      break;
    case TOPOS1:   // moving to pos 1, if button pressed, go back
      if ( myButtons.pressed(SRVBUT) ) {
        Serial.println( "Button pressed backt to POS2");
        buttonServo.setSpeedTime(3000);
        buttonServo.write(POS2);
        srvState = TOPOS2;
      }
      if ( !buttonServo.moving() ) {
        // we reached the position 2
        srvState = ATPOS1;
      }
      break;
    default: // Should never be reached
      ;
  }

}
