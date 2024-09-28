/*
  SoftBlink
  A variation of the Arduino-Examples Blink Sketch
  
  Fading in LED on for one second, then fading out for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA
  it is attached to digital pin 13. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  modified 12 Sep 2023
  by F.-P. Mueller

  This example code is in the public domain.


*/
#include <MobaTools.h>
MoToSoftLed myLed;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN
  myLed.attach(LED_BUILTIN);
  myLed.riseTime( 800 );    // Risetime in ms
}

// the loop function runs over and over again forever
void loop() {
  myLed.on();       // turn the LED on
  delay(1000);      // wait one second while the LED slowly fades in
  myLed.off();      // turn the LED off
  delay(1000);      // wait one second while the LED slowly dims
}
