#include <MobaTools.h>
/* Demo: weiches Auf/Abblenden einer LED
 *  Mit der Klasse SoftLed lässt sich eine Led 'weich' ein- und ausschalten
 * Die Led muss nicht an einem PWM-fähigen Pin angeschlossen sein, es funktioniert
 * mit jedem Digital-Pin.
 * SoftLed.attach( byte Pinnr );// Anschlußpin für die Led festlegen
 * SoftLed.riseTime( int zeit );// Zeit für auf/abblenden in ms
 * SoftLed.on();                // 'weich' einschalten
 * SoftLed.off();               // 'weich' ausschalten
*/

const byte ledPin =  13;  // beliebiger Digital Pin


MoToSoftLed meinLed;

void setup() {
    meinLed.attach(ledPin); 
    meinLed.riseTime( 1000 );    // Aufblendzeit in ms
}

void loop() {
	meinLed.on();
	delay(1000);
	meinLed.off();
	delay(1000);
}
