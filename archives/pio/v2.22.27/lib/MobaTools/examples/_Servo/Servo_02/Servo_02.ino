#include <MobaTools.h>
/* Demo to move a servo slowly
 * The current version of MoBatools allows up to 16 servos only on arbitrary pins. 
 * A simultaneous use with the standard servo library of the Arduino is not possible, 
 * because the same HW components are used. 
 * The calls are compatible with the standard Servo Library. 
 * Additional calls:
 * Servo2.setSpeed( value ); Set the speed. The larger the number, the greater the speed. 
 *                           At 0 (default value) the servo behaves as with the standard library.
 * byte = Servo2.moving();   specifies the remaining travel distance in % of the total travel distance. 
 *                           At 0 the servo has reached the target position and stops.
 *                           
 * missing calls:
 * writeMicroseconds();      microseconds can also be passed in the normal write call. The distinction 
 *                           between angle (0..180) and microseconds ( 700...2300 ) results from the 
 *                           numerical value.
 *                           
 * Demo 02 : with query of the servo movement. In this sketch, the servo can only be reversed when 
 * it has reached its target position and is stationary.
 * In the demo Servo_01 the servo can be reversed at any time.
*/

// The pushbuttons must be connected in such a way that the input goes LOW (=0) 
// when the pushbutton is pressed. 
const int buttonPin1 = 2;    //Taster1 Pin 2
const int buttonPin2 = 3;    //Taster2  Pin 3
const int servoPin =  9;  // Connection for the servo

int buttonState1, buttonState2;
MoToServo myServo;

void setup() {
    pinMode(buttonPin1, INPUT_PULLUP); // no external pullup resistor 
    pinMode(buttonPin2, INPUT_PULLUP); // is required at the pushbutton
    
    myServo.attach(servoPin); //servo at pin 9
    myServo.setSpeed( 5 );    // set servo speed
}

void loop() {
    buttonState1 = digitalRead(buttonPin1);
    buttonState2 = digitalRead(buttonPin2);

    if (buttonState1 == LOW && myServo.moving() == 0 ) {
        myServo.write(40); // will turn slowly
        delay(100);     // This is not necessary, but shows that the servo even moves during delay()
    }

    if (buttonState2 == LOW && myServo.moving() == 0) {
        myServo.write(120); // will turn slowly
        delay(100);
    }

    delay(20);

}
