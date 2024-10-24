
// Servo sweeps between 0 and 180° back and forth
void sweepSServoFSM() {
  // sweeping the first servo, sweeping back with half speed
  if ( !sweepServo.moving() ) {
    // final position reached, move to other position
    if ( lastTarget == servoTarget1 ) {
      sweepServo.setSpeedTime( sweepTime2 );
      sweepServo.write( lastTarget = servoTarget2 );
    } else {
      sweepServo.setSpeedTime( sweepTime1 );
      sweepServo.write( lastTarget = servoTarget1 );
    }
  }
}
