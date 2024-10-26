// Test-Sketch zum an- und ausschalten einer LED mit einem Taster beim Drücken und Loslasen des Tasters
// mit Entprellung der Taster und Aufteilung in Eingabe - Verarbeitung - Ausgabe
// mit Flankenerkennung im Eingabeblock

// Test sketch for switching an LED on and off with a pushbutton when pressing and releasing the pushbutton
// with debouncing of the buttons and division of the sketch into input - processing - output

#include <MoToButtons.h>
// Variablen deklarieren und mit definierten Grundwerten vorbelegen
const byte tasterPinNr [] = { 14, 15, 16, 17 };     // = A0...A3 auf UNO/Nano
const byte anzahlTaster = sizeof(tasterPinNr);                  // Anzahl der angeschlossenen Taster
// Variablen für die LEDs
const byte LEDgruenPinNr[] = { 2, 3, 4, 5 };  // Array für Pin-Nummern der grünen LEDs definieren
const byte LEDrotPinNr[]   = { 6, 7, 8, 9 };   // Array für Pin-Nummern der roten LEDs definieren

button_t getHW( void ) {
  // Einlesen der Tasterstates
button_t tasterTemp = 0;
  for (byte i = 0; i < anzahlTaster; i++) {
    bitWrite( tasterTemp,i,!digitalRead(tasterPinNr[i]) );     // Fragt den Taster ab und merkt sich den Status
  }
  return tasterTemp;
}

MoToButtons Taster1( getHW, 20, 500 );  // 20ms Entprellzeit, 500ms für den Unterschied kurz/lang gedrückt.

void setup()
{
  Serial.begin(115200);
  while(!Serial);                           // nur für Leonardo und Micro notwendig (Mega32u4)
  // alle Taster und LEDs definieren und Initialisieren
  for (int i = 0; i < anzahlTaster; i++)  { // Bei Abfrage auf < kann man das -1 sparen
    // Taster definieren
    pinMode(tasterPinNr[i], INPUT_PULLUP);  // Taster-Pins als Input definieren (gedrückt = LOW)

    // LEDs definieren
    pinMode(LEDgruenPinNr[i], OUTPUT);      // grüne LED-Pins als Output definieren
    pinMode(LEDrotPinNr[i], OUTPUT);        // rote LED-Pins als Output definieren
  }
  Serial.println("Start loop");
}

void loop() {
  //--------------------------------------------------------
  // Block "Eingabe": Taster entprellt einlesen und Startzeit merken
  Taster1.processButtons();
  // Ende Block "Eingabe"
  //--------------------------------------------------------
  
  // Block "Verarbeitung / Ausgabe": TasterStellung auswerten und Aktion durchführen
  for (byte i = 0; i < anzahlTaster; i++) {
    // Mit dieser Bedingung wird bei langem Tastendruck (also hier 500ms nach dem Drücken des Tasters) geschaltet
    // -> beim langen Drücken rote Led toggeln
    if ( Taster1.longPress(i) ) {
      digitalWrite(LEDrotPinNr[i], !digitalRead(LEDrotPinNr[i]));      // rote LED einschalten
      Serial.print(" Langer Tastendruck: "); Serial.println(i);
    }

    // Mit dieser Bedingung wird bei kurzem Tastendruck geschaltet
    // (also beim Loslassen des Tasters bei weniger als 500ms Drückzeit )
    // -> beim Loslassen (nach kurzem Drücken ) gruene Led toggeln
    if ( Taster1.shortPress(i) ) {
      digitalWrite(LEDgruenPinNr[i], !digitalRead(LEDgruenPinNr[i]));      // rote LED ausschalten
      Serial.print(" Kurzer Tastendruck: "); Serial.println(i);
    }
  }
  // Ende Block "Verarbeitung / Ausgabe"

}   // Ende loop
