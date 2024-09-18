// Show Events of buttons in serial monitor
#define MAX8BUTTONS
#include <MobaTools.h>
// define pin numbers
const byte buttonPin [] = { A0, A1, A2, A3 };
const byte buttonCnt = sizeof(buttonPin);
char txtBuf[50];

//this instance is without a callback ( pins are read directly )
// double click time is 400ms
MoToButtons Buttons( buttonPin, buttonCnt, 30, 500, 400 );

void printPressedEvent( uint8_t buttonNbr ) {
  if ( Buttons.pressed(buttonNbr) ) {
    sprintf( txtBuf, "button %d pressed", buttonNbr );
    Serial.println(txtBuf);
  }
}

void printReleasedEvent( uint8_t buttonNbr ) {
  if ( Buttons.released(buttonNbr) ) {
    sprintf( txtBuf, "button %d released", buttonNbr );
    Serial.println(txtBuf);
  }
}

void printlongpressEvent( uint8_t buttonNbr ) {
if ( Buttons.longPress(buttonNbr) ) {
    sprintf( txtBuf, "button %d pressed long", buttonNbr );
    Serial.println(txtBuf);
  }
}

void printshortpressEvent( uint8_t buttonNbr ) {
  if ( Buttons.shortPress(buttonNbr) ) {
    sprintf( txtBuf, "button %d pressed short", buttonNbr );
    Serial.println(txtBuf);
  }
}

void printClickedEvent( uint8_t buttonNbr ) {
  switch ( Buttons.clicked(buttonNbr) ) {
    case NOCLICK:
      ; // do nothing
      break;
    case DOUBLECLICK:
      sprintf( txtBuf, "button %d double clicked", buttonNbr );
      Serial.println(txtBuf);
      break;
    case SINGLECLICK:
      sprintf( txtBuf, "button %d single clicked", buttonNbr );
      Serial.println(txtBuf);
      break;
  }
}

void setup()
{
  Serial.begin(115200);
  while(!Serial);       // only for Leonardo/Micro ( mega32u4 based boards 
  
  for (int i = 0; i < buttonCnt; i++)  {    
    // buttons must switch to Gnc
    pinMode(buttonPin[i], INPUT_PULLUP); 
  }
  Serial.println("Starting loop");
  sprintf( txtBuf, "max. managable buttons: %d", sizeof(button_t)*8 );
  Serial.println( txtBuf );
  //Buttons.forceChanged();
}

void loop() {
  //--------------------------------------------------------
  // read and process buttons
  Buttons.processButtons();
  // 
  //--------------------------------------------------------
  // print state of buttons if at least one changed
  if ( Buttons.changed() ) {
    sprintf( txtBuf, "------------ State: %d %d %d %d - ", Buttons.state(0), Buttons.state(1), Buttons.state(2), Buttons.state(3) );
    Serial.print( txtBuf ); Serial.println( Buttons.allStates(),BIN );
  }
  // print to serial monitor if an event happens ( pressing or releasing )
  // Button 0 checked for all events
  printPressedEvent(0 );
  printReleasedEvent( 0 );
  printshortpressEvent( 0 );
  printlongpressEvent( 0 );
  printClickedEvent( 0 );

  // Button 1 checked for releasing
  printReleasedEvent( 1 );
  
  // Button 2 checked for short/long press
  printshortpressEvent( 2 );
  printlongpressEvent( 2 );

  // Button 3 checked for double Click
  printClickedEvent( 3 );

}
