#include <MobaTools.h>
/* Demo: Zeitverzögerungen ohne delay-befehl
 * Der 'MoToTimer' arbeitet im Prinzip wie ein Kurzzeitwecker in der
 * Küche: Man zieht ihn auf eine bestimmte Zeit auf, und dann läuft 
 * er bis 0 zurück. Im Gegensatz zum Küchenwecker klingelt er aber nicht.
 * Man muss zyklisch nachschauen, ob er abgelaufen ist. Das passt aber
 * perfekt zum prinzip des 'loop', also einer Endlosschleife, in der man
 * zyklisch abfragt.
 * Aufrufe:
 * MoToTimer.setTime( long Laufzeit );    setzt die Zeit in ms
 * bool = MoToTimer.running();       == true solange die Zeit noch läuft, 
 *                                  == false wenn abgelaufen
 *                                  
 * Im Gegensatz zum Verfahren mit delay() lassen sich damit mehrere
 * unabhängige und asynchrone Taktzeiten realisieren
 * In dieser Demo blinken 2 Led mit unterschiedlichen Taktraten
*/

const int led1P =  5; 
const int led2P =  6; 

MoToTimer Blinkzeit1;
MoToTimer Blinkzeit2;

void setup() {
    pinMode(led1P, OUTPUT); 
    pinMode(led2P, OUTPUT);
}

void loop() {
    // -------- Blinken der 1. Led ------------------
    // diese Led blinkt mit unsymetrischem Taktverhältnis
	if ( Blinkzeit1.running()== false ) {
        // Blinkzeit abgelaufen, Ausgang toggeln und
        // Zeit neu aufziehen
        if ( digitalRead( led1P ) == HIGH ) {
            digitalWrite( led1P, LOW );
            Blinkzeit1.setTime( 600 );
       } else {
            digitalWrite( led1P, HIGH );
            Blinkzeit1.setTime( 300 );
        }
	}
   
    // -------- Blinken der 2. Led ------------------
    // Diese Led blinkt symetrisch
    if ( Blinkzeit2.running() == false ) {
        // Blinkzeit abgelaufen, Ausgang toggeln und
        // Zeit neu aufziehen
        if ( digitalRead( led2P ) == HIGH ) {
            digitalWrite( led2P, LOW);
        } else {
            digitalWrite( led2P, HIGH);
        }
        Blinkzeit2.setTime( 633 );
    }
}
