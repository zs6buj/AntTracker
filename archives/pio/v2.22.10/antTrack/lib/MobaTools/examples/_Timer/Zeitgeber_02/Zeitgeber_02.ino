#include <MobaTools.h>
/* Demo: Zeitverzögerungen ohne delay-befehl
   Der 'MoToTimer' arbeitet im Prinzip wie ein Kurzzeitwecker in der
   Küche: Man zieht ihn auf eine bestimmte Zeit auf, und dann läuft
   er bis 0 zurück. Im Gegensatz zum Küchenwecker klingelt er aber nicht.
   Man muss zyklisch nachschauen, ob er abgelaufen ist. Das passt aber
   perfekt zum prinzip des 'loop', also einer Endlosschleife, in der man
   zyklisch abfragt.
   Aufrufe:
   MoToTimer.setTime( long Laufzeit );    setzt die Zeit in ms
   bool = MoToTimer.running();       == true solange die Zeit noch läuft,
                                    == false wenn abgelaufen
   bool = MoToTimer.expired();    Der Aufruf ist einmalig true, nachdem der Timer abgelaufen ist.
                                  Weitere Aufrufe ergeben dann wieder false

   Im Gegensatz zum Verfahren mit delay() lassen sich damit mehrere
   unabhängige und asynchrone Taktzeiten realisieren
   In dieser Demo werden 2 Led mit jeweils einem Taster für unterschiedlichen Zeiten
   eingeschaltet. Bei jedem Ablauf der Zeit wird einmalig eine Meldung auf dem seriellen Monitor ausgegeben
*/

const byte led1P =  5;
const byte led2P =  6;

const byte taster1P = A0;
const byte taster2P = A1;


MoToTimer Blinkzeit1;
MoToTimer Blinkzeit2;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // nur für Arduino Micro/Leonardo notwendig
  pinMode(led1P, OUTPUT);
  pinMode(led2P, OUTPUT);
  pinMode(taster1P, INPUT_PULLUP );
  pinMode(taster2P, INPUT_PULLUP );
}

void loop() {
  // -------- Schalten der 1. Led ------------------
  if ( !digitalRead(taster1P) && !Blinkzeit1.running() ) {
    Serial.println("Zeit1 starten");
    // Zeit aufziehen und Led schalten
    Blinkzeit1.setTime( 2000 );
    digitalWrite( led1P, HIGH );
  }
  if ( Blinkzeit1.expired() ) {
  //if (! Blinkzeit1.running() ) {
    Serial.println("Zeit 1 abgelaufen");
    digitalWrite( led1P, LOW );
  }
  // -------- Schalten der 2. Led ------------------
  if ( !digitalRead(taster2P) && !Blinkzeit2.running() ) {
    // Zeit aufziehen und Led schalten
    Serial.println("Zeit2 starten");
    Blinkzeit2.setTime( 1500 );
    digitalWrite( led2P, HIGH );
  }
  if ( Blinkzeit2.expired() ) {
    Serial.println("Zeit 2 abgelaufen");
    digitalWrite( led2P, LOW );
  }
  
}
