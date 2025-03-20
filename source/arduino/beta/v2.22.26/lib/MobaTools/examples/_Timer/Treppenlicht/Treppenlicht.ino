/*
 * Licht im Treppenhaus steuern:
 * - nach kurzem Druck Licht kurz einschalten zum Begehen
 * - nach langem Druck Licht lange einschalten zum Treppeputzen
 *
 * MoToButtons wird zum Entprellen eines mechanischen Tasters verwendet und zur Definition eines langen Drucks.
 * MoToTimer wird für die Brenndauer des Treppenlichts verwendet.
 */
// Zeiten in Millisekunden
const long      TREPPENLICHTKURZ = 3000;     // gegebenenfalls verlängern
const long      TREPPENLICHTLANG = 15000;    // gegebenenfalls verlängern
const byte      TASTENENTPRELLZEIT = 30;
const uint16_t  TASTENDRUCKLANG = 500;

const byte led_pin = 2;                     // Lichtrelais an Pin 2
const byte pinArray [] = { 12, 13 };        // Lichttaster an Pin 12/14 ( Ein und Aus )
enum :byte { LICHTAN, LICHTAUS };
const byte pinCount = sizeof(pinArray);

#define MAX8BUTTONS
#include <MobaTools.h>
MoToButtons Taster( pinArray, pinCount, TASTENENTPRELLZEIT, TASTENDRUCKLANG );
MoToTimer Treppenlicht;

void setup() {
  pinMode(led_pin, OUTPUT);
}

void loop() {
  //------------------------
  Taster.processButtons();
  //------------------------
  if (Taster.pressed(LICHTAN)) {
    // bei jedem Tastendruck Licht einschalten und kurze Zeit setzen
    digitalWrite(led_pin, HIGH);
    Treppenlicht.setTime(TREPPENLICHTKURZ);
  }
  if (Taster.longPress(LICHTAN)) {
    // Wird lange auf den Taster gedrückt, wird die Zeit verlängert.
    Treppenlicht.setTime(TREPPENLICHTLANG);
  }
  
  if (Taster.pressed(LICHTAUS)) {
    // Mit diesem Taster wird das Licht vorzeitig ausgeschaltet.
    Treppenlicht.setTime(10);   // Zeit auf 10ms verkürzen, also praktisch unmittelbares Ausschalten )
  }
  
  if ( Treppenlicht.expired() ) {
    // Wenn die Zeit abgelaufen ist, Licht ausschalten
    digitalWrite(led_pin, LOW);
  }
}
