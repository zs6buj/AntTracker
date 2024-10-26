#include <MobaTools.h>
/* Demo zum langsamen Bewegen eines Servos
 * Die aktuelle Version der MoBatools erlaubt bis zu 16 Servos nur an beliebigen Pins
 * Eine gleichzeitige Verwendung mit der Standard Servo Library des Arduino
 * ist nicht möglich, da die gleichen HW-Komponenten verwendet werden.
 * Die Aufrufe sind kompatibel zur Standard-Servo Library. 
 * zusätzliche Aufrufe:
 * Servo2.setSpeed( wert ); Vorgabe der Geschwindigkeit. Je größer die Zahl, umso
 *                          größer ist die Geschwindigkeit. Bei 0 (defaultwert)
 *                          verhält das Servo sich wie bei der Standard Bibliothek
 * byte = Servo2.moving();  gibt den noch verbleibenden Fahrweg in % vom gesamten
 *                          Verfahrweg an. Bei 0 hat das Servo die Zielposition
 *                           erreicht und steht.
 * fehlende Aufrufe:
 * writeMicroseconds() auch beim normalen write Aufruf können Microsekunden übergeben 
 *                     werden. Die Unterscheidung Winkel (0..180) und Mikrosekunden 
 *                    ( 700...2300 )ergibt sich aus dem Zahlenwert.
 * 
*/
// Die Taster müssen so angeschlossen sein, dass der Eingang bei gedrücktem
// Taster auf LOW (=0) geht. 
const int tasterPin1 = 2;    //Taster1 Pin 2
const int tasterPin2 = 3;    //Taster2  Pin 3
const int servoPin =  9;  // Anschluß für den Servo 
// bei Werten, die sich im Programm nie verändern, sollte immer 'const' vorangestellt
// werden


bool tasterStatus1, tasterStatus2;
MoToServo meinServo;

void setup() {
    pinMode(tasterPin1, INPUT_PULLUP); // so ist kein externer pullup Widerstand am 
    pinMode(tasterPin2, INPUT_PULLUP); // Taster erforderlich
    
    meinServo.attach(servoPin); //Servo an Pin 9
    meinServo.setSpeed( 5 );    // Verfahrgeschwindigkeit einstellen
}

void loop() {
    tasterStatus1 = digitalRead(tasterPin1);
    tasterStatus2 = digitalRead(tasterPin2);

    if (tasterStatus1 == LOW) {
        meinServo.write(40); //wird langsam  drehen
        delay(100);
    }

    if (tasterStatus2 == LOW) {
        meinServo.write(120); //wird langsam drehen
        delay(100);
    }

    delay(20);

}
