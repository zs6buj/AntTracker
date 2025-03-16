void buttonStepperFSM() {
  /* move stepper according to button presses:
  The stepper moves forward to the position two revolutions from zero when the button is pressed short.
  If the button is pressed long it moves to the position two revolutions back from zero.
  If the button is pressed while it is moving, it moves to zero position.
  */
  static bool initialCall = true;
  static long targetPos = mStepRev * 2;
  if ( initialCall ) {
    // first call, initialize everything
    initialCall = false;
    buttonStepper.attach( step2StepPin, step2DirPin );
    buttonStepper .setSpeedSteps( 10000, mStepRev / 4);
    buttonStepper.attachEnable( step2EnaPin, 200, LOW );
    Serial.println( "Stepper initialized");

  } else {
    // waiting for the button to be pressed
    if ( myButtons.shortPress(STPBUT) ) {
      targetPos = buttonStepper.moving() ? 0 : mStepRev * 2;
      Serial.print( "Moving to "); Serial.println( targetPos );
      buttonStepper.moveTo( targetPos);
    }
    if ( myButtons.longPress(STPBUT) ) {
      targetPos = buttonStepper.moving() ? 0 : mStepRev * -2;
      Serial.print( "Moving to "); Serial.println( targetPos );
      buttonStepper.moveTo( targetPos);
    }
    
  }
}
