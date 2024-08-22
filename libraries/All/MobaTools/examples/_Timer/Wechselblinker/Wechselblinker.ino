#include <MobaTools.h>

/* Demo: Wechselblinker
 * Dieses Demo ist etwas komplexer und zeigt die Realisierung eines
 * Wechselblinker, der per Schalter ein- und ausgeschaltet wird.
 * Wie bei Wechselblinkern an einem BÜ üblich, starten die beiden Lampen
 * gleichzeitig, um dann im Wechseltakt zu blinken
 * Dieses Demo verwendet auch die 'Eieruhr' um Zeitverzögerungen zu realisieren
 * Die 'Eieruhr' ist ein Zeitzähler, der aufgezogen werden kann und im loop kann
 * abgefragt werden, ob er abgelaufen ist.
*/

// Festlegen der Ports
const int Blinker1P =  5;  // Die beiden Led's des
const int Blinker2P =  6;  // Wechselblinkers.
const int SchalterP =  7;  // Schalter, der den Wechselblinker ein und ausschaltet

// Weitere Konstante
const int wbZykl = 1100;   // Zykluszeit des Wechselblinkers
const int wbSoft = 400;    // Auf/Abblendzeit der Lampen

// Zustand des Wechselblinker
byte wblZustand = 0;   // In dieser Variable wird hinterlegt, in welchem Zustand
						// sich der Wechselblinker gerade befindet
#define   WBL_AUS     0 	// beide Lampen sind aus
#define   WBL_START   1   	// Startphase: beide Lampen sind an
#define   WBL_BLINKT  2   	// Die Lampen blinken normal im Wechsel
byte ledState;              // HIGH : Blinker1 ist an, LOW Blinker2 ist an

MoToSoftLed Blinker1;
MoToSoftLed Blinker2;

MoToTimer BlinkUhr;

void setup() {
    pinMode(SchalterP, INPUT_PULLUP); 
    Blinker1.attach(Blinker1P);  // die Ausgänge werden automatisch auf OUTPUT gesetzt
    Blinker2.attach(Blinker2P); 
    Blinker1.riseTime( wbSoft );    // Aufblendzeit in ms
    Blinker2.riseTime( wbSoft );    // Aufblendzeit in ms
}

void loop() {
    // Wechselblinker
    switch (wblZustand) {
      case WBL_AUS:
        // Beide Lampen sind aus, warten auf einschalten
        if ( digitalRead(SchalterP) == HIGH && BlinkUhr.running() == false ) {
            // Beide Leds einschalten, Timer für gemeinsames Startleuchten
            Blinker1.on();
            Blinker2.on();
            BlinkUhr.setTime( wbSoft );
            wblZustand = WBL_START;
        }
        break;
      case WBL_START:
        // Startphase: Nach Zeitablauf erste Led wieder aus
        if ( BlinkUhr.running() == false ) {
            // Die Startzeit ist abgelaufen, Übergang zur normalen Blinkphase
            ledState = HIGH;
            Blinker2.off();
            BlinkUhr.setTime(wbZykl/2); // Zeitverzögerung setzen
            wblZustand = WBL_BLINKT;
        }
        break;
      case WBL_BLINKT:
        if ( BlinkUhr.running() == false ) {
            BlinkUhr.setTime(wbZykl/2); // Zeitverzögerung setzen
            if ( ledState == LOW ) {
                Blinker1.on();
                Blinker2.off();
                ledState = HIGH;
            } else {
                ledState = LOW;
                Blinker2.on();
                Blinker1.off();
            }
        }
        if ( digitalRead(SchalterP) == LOW ) {
            // Wechselblinker abschalten
            Blinker1.off();
            Blinker2.off();
            wblZustand = WBL_AUS;
            BlinkUhr.setTime(wbZykl);   // minimale 'Aus' zeit des Blinkers
                                        //( schützt vor Schalterprellen )
        }
        break;
            
    } // Ende switch Wechselblinker
}
