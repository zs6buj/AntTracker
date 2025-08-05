/*  Demo zum Anschluß eines unipolaren Stepmotors 28BYJ-48
    mit Verwendung einer Beschleunigungsrampe
    Slider-Minimalprogramm: Endposition - Startposition - Bewegung von Start- zu Endposition
    Dieses Beispiel läuft nicht auf ESP8266
    Danke an 'agmue' vom arduino.cc Forum für dieses Beispiel
*/

#include <MobaTools.h>
MoToStepper Step1(4096);           // HALFSTEP ist default
const byte linksPin = 8, rechtsPin = 9, weiterPin = 10;

void setup() {
  Serial.begin(9600);
  Serial.println("Anfang");
  pinMode(linksPin, INPUT_PULLUP);   // aktiv LOW
  pinMode(rechtsPin, INPUT_PULLUP);  // aktiv LOW
  pinMode(weiterPin, INPUT_PULLUP);  // aktiv LOW
  Step1.attach( 4, 5, 6, 7 ); // Anschluß an digitalen Ausgängen; Treiber IN1,IN2,IN3,IN4
  Step1.setSpeed( 60 );       // = 6 U/Min
}

void loop() {
  enum {ENDPOSITION, STARTPOSITION, BEWEGUNG};
  static byte status;               // Schrittkettenstatus
  static bool alt = digitalRead(weiterPin), akt = digitalRead(weiterPin);
  alt = akt;
  akt = digitalRead(weiterPin);
  if (alt != akt) {
    delay(30);                // einfaches Entprellen
  }

  switch (status) {
    case ENDPOSITION:
      if (!digitalRead(linksPin)) {
        Step1.doSteps(10);
      } else if (!digitalRead(rechtsPin)) {
        Step1.doSteps(-10);
      } else if (alt && !akt) {
        Step1.setZero();              // Endpunkt für Motor 1 setzen
        Serial.println("STARTPOSITION");
        status++;
      } else {
        Step1.stop();
      }
      break;
    case STARTPOSITION:
      if (!digitalRead(linksPin)) {
        Step1.doSteps(10);
      } else if (!digitalRead(rechtsPin)) {
        Step1.doSteps(-10);
      } else if (alt && !akt) {
        Step1.setSpeed( 240 );          // = 24 U/Min
        Step1.setRampLen(500);          // Beschleunigung
        Step1.write(0);                 // Bewegung zu Endpunkt
        Serial.println("BEWEGUNG");
        status++;
      } else {
        Step1.stop();
      }
      break;
    case BEWEGUNG:
      if ( !Step1.moving() ) {          // warten bis die Bewegung abgeschlossen ist
        Step1.setSpeed( 60 );           // = 6 U/Min
        Step1.setRampLen(0);            // Beschleunigung
        Serial.println("ENDPOSITION");
        status = 0;
      }
      break;
    default:
      status = 0;
  }
}
