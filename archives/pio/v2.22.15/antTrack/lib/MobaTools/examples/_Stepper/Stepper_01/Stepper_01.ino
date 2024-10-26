/******************************************************************  
   Demo zum Anschluß eines unipolaren Stepmotors 28BYJ-48 ( über SPI )
  und eines Standard bipolaren Steppers über 4 Pins und eine H-Brücke.
  Dieses Beispiel läuft nicht auf ESP8266!
  
  Der Schrittmotor kann wahlweise direkt über 4 frei wählbare Pins, oder über die SPI-
  Schnittstelle und Schieberegister angeschlossen werden. 
  SPI belegt beim UNO oder nano die Pins 10(SS),11(MOSI) und 13(SCK)
  Pin 12 (MISO) wird zwar nicht genutzt, aber von der HW belegt
*/
/*******************************************************************
  Demo for connecting a unipolar stepper motor 28BYJ-48 ( via SPI ) and a standard bipolar stepper
  via 4 pins and a H-bridge.  This example does not run on ESP8266!
  
  The stepper motor can be connected either directly via 4 freely selectable pins, or via the SPI 
  interface and shift register (see circuit diagram in documentation). 
  SPI occupies the pins 10(SS), 11(MOSI) and 13(SCK) of the UNO or nano.
  Pin 12 (MISO) is not used, but is occupied by the hardware
  In this example Step1 is connected directly while Step2 is connected via SPI.

Translated with www.DeepL.com/Translator (free version)
 */
#include <MobaTools.h>
const int FULLROT1 = 400;
MoToStepper Step1(FULLROT1);           // HALFSTEP ist default
MoToStepper Step2(2048,FULLSTEP);

//const byte stPn[] = { 16,17,18,19 }; // für unipolaren Stepper (5-Draht)
const byte stPn[] = { 16,18,17,19 }; // bei bipolarem Stepper (4-Draht) über ein H-Brücke müssen
                                     // gegebenenfalls die inneren Pins vertauscht werden
                                     // ( oder entsprechend verdrahten )
                                     // Spule 1: 1. und 3. Pin
                                     // Spule 2: 2. und 4. Pin
void setup() {
    Step1.attach( stPn[0],stPn[1],stPn[2],stPn[3] );    // Anschluß an digitalen Ausgängen
    Step2.attach( SPI_1 );      // an die SPI-Schnittstelle muss ein Schieberegister angschlossen werden
    Step1.setSpeed( 240 );       // = 24 U/Min
    Step1.setRampLen( FULLROT1/4 );     // = 1/4 Umdrehung
    Step2.setSpeed( 120 );      // = 12 U/Min
    Step2.rotate( -1 );         // Motor 2 dreht dauerhaft rückwärts
    Step1.setZero();            // Referenzpunkt für Motor 1 setzen
    Step1.write(360);           // 1 Umdrehung vorwärts
    while( Step1.moving() );    // warten bis die Bewegung abgeschlossen ist
    delay(1000);                // 1 Sec stillstehen
    Step1.write(0);             // 1 Umdrehung zurück zum Referenzpunkt
    while( Step1.moving() );    // warten bis Stillstand
    delay(1000);                // Motor steht 1 sec.
    Step1.doSteps(400);        // 4096 Schritte vorwärts ( = auch 1 Umdrehung )
    while( Step1.moving() );
    delay(1000);
    Step1.write(0);             // 1 Umdrehung zurück zum Referenzpunkt ( auch bei doSteps wird die
    while( Step1.moving() );    // Position verfolgt
    delay(1000);                // Motor steht 1 sec.
    Step1.detach();             // Motor deaktivieren
    delay(2000);
    Step1.attach(stPn[3],stPn[2],stPn[1],stPn[0]);      // mit umgekehrter Reihenfolge der Pins aktivieren
    Step1.setSpeedSteps( 8000, 400); // Rampe und Steprate neu ( ist nach attach wieder default  )
    Step1.doSteps(4000);        // 'vorwärts' ist jetzt andersherum!
    while( Step1.moving() );
    delay(2000);
    Step1.writeSteps(0);        // 1. Stepper läuft zurück zum Referenzpunkt
    Step2.setRampLen( 1024 );   // Rampe für 2. Stepper 1/2 Umdrehung
    Step2.doSteps(0);           // Momentanposition ist Haltepunkt: Bremsen mit Rampe, dann zurück.
    while( (Step1.moving() + Step2.moving() ) >0 );
    Step1.detach(); 
    Step2.detach(); 

}

void loop() {
}
