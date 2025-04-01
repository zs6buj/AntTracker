// Syncronous move of MoToStepper steppers
#include <MobaTools.h>

constexpr uint8_t step1Pin  = 6;
constexpr uint8_t dir1Pin   = 5;
constexpr uint8_t ena1Pin   = 7;
constexpr uint8_t step2Pin  = 9;
constexpr uint8_t dir2Pin   = 8;
constexpr uint8_t ena2Pin   = 12;

MoToStepper stepper1( 800, STEPDIR );
MoToStepper stepper2( 800, STEPDIR );

MoToSyncStepper mySteppers;

constexpr uint8_t posCnt = 7;
long syncPos[posCnt][2] = { // List of target positions
    { 800, 1600 },
    { 1600, 3200 },
    { 800, -6000 },
    { 777, -3333 },
    {0,0},
    {400,700 },
    {0,0},
};

char txtBuf[60];

void setup() {
    Serial.begin(115200);
    // Initialize steppers
    stepper1.attach( step1Pin,dir1Pin );
    stepper1.attachEnable(ena1Pin,50,LOW);
    stepper2.attach(step2Pin,dir2Pin);
    stepper2.attachEnable(ena2Pin,50,LOW);
    // Add steppers to MoToSyncStepper
    mySteppers.addStepper( stepper1 );
    mySteppers.addStepper( stepper2 );
    mySteppers.setMaxSpeedSteps(5000);
}

void loop() {
    Serial.println("Press any key to start...");
    while(  !Serial.available() );
    // type any character in serial monitor to start
    while (Serial.available() ) Serial.read(); // clear buffer
    // Move steppers to targets
    for ( byte i=0; i<posCnt;i++ ) {
        sprintf( txtBuf," Targets: %ld, %ld", syncPos[i][0], syncPos[i][1] ); Serial.println(txtBuf);
        mySteppers.moveTo( &syncPos[i][0] );
        while ( mySteppers.moving() ) delay(100);;
        Serial.println("Targets reached");
        delay(1000);
    }
    delay(2000);
}
