// Stepper moving back and forth in fixed time slots
void autoStepperFSM() {
  static bool initialCall = true;
  static long intervallSteps = aStepRev * 2;
  static byte intervallCount = 0;
  const byte intMax = 4; // direction is changed

  if ( initialCall ) {
    // first call of function, initiate everything
    initialCall = false;
    stepperTimer.setBasetime(2000); // timer tic every 2 sec
    autoStepper.attach( step1StepPin, step1DirPin );
    autoStepper .setSpeedSteps( 20000, aStepRev / 4);
    autoStepper.attachEnable( step1EnaPin, 200, LOW );
  } else {
    if ( stepperTimer.tick() ) {
      autoStepper.doSteps ( intervallSteps );
      if ( ++intervallCount >= intMax ) {
        // change direction
        intervallSteps = -intervallSteps;
        intervallCount = 0;
      }
    }

  }

}
