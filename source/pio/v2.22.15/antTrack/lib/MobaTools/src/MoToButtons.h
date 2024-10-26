/* MoToButtons - a Arduino library to manage up to 8, 16 or 32 buttons/Switches 
  with debouncing, edge detection and long/short press detection.
  Author: fpm, fpm-gh@mnet-mail.de
  Copyright (c) 2020 All right reserved.
  This Library is part of MoBaTools

 Default is managing up to 16 buttons/switches.
 The default can be changed to save RAM (up to 8 buttons) or to manage up to 32 buttons (with additional RAM consumption). 
 This can be achieved by inserting '#define MAX32BUTTONS' or '#define MAX8BUTTONS'  before the #include <MoToButtons.h>.
  
 Reading the hardware state of the buttons is done by a usercallback function. 
 This enables designs where the buttons/switches are arranged in a matrix and/or read via a port extender.
 The return value of this function has to be a 8-Bit, 16-Bit or 32-Bit value according to the maximum manageable 
 number of buttons. Every button/switch is represented by one bit, where '1' means the button is pressed.

 'button_t' is automtically set to the correct type and can be used to define the type of the callback function.
 

  Constructor parameters:
    button_t (*getHWbuttons)()  Adress of the userfuction that reads the state of the buttons
    debTime                     Debouncetime in ms
    pressTime                   (in ms ) If the button is pressed longer, it is a 'long press'
                                max presstime = debTime*255
    doubleClick                 max time between two presses to be recognized as dounle click
                                ( optional, default: 2*pressTime )
    MoToButtons( button_t (*getHWbuttons)(), uint8_t debTime, uint16_t pressTime );
    example in sketch: 
        MoToButtons Buttons( readFunction, 20, 500 );
    The first parameter can alternatively be replaced by a pointer to an array with pin numbers and the number of active pins. In this case the state of the pins 
    is read by the class itself. 'LOW' at the pins means 'pressed':
    MoToButtons( const uint8_t pinNumbers[], const uint8_t pinCnt, uint8_t debTime, uint16_t pressTime );
    
    In both cases there may be an additional parameter ( uint16_t clickTime  ) to define the time for clicks/double clicks ( default ist 300ms )
    
  Methods to be called in loop:
    void    processButtons();                   // must be called in the loop frequently
                                                // if it is called less frequently than debTime, pressTime will be inaccurate
    Reading the debounced state of the Buttons/Switches:                                          
      byte state( uint8_t buttonNbr );       // get static state of button (debounced) 
                                                // 0 if button not pressed
                                                // presstime in debounce tics ( up to 255 ) if pressed
      button_t allStates();                     // bit field of all buttons (debounced)
      button_t changed();                       // all bits are set where state has changed since last call
  
    Reading events:
      bool shortPress( uint8_t buttonNbr );  // true if button was pressed short ( set when button is released, reset after call )  
      bool longPress( uint8_t buttonNbr );   // true if button was pressed long ( set when button is released, reset after call )  
      bool pressed( uint8_t buttonNbr );     // true if button is pressed ( reset after call )
      byte released( uint8_t buttonNbr );    // returns pressed time im debounce tics if button is released, 0 otherwise ( reset after call )
      uint8_t clicked( uint8_t buttonNbr );  // = NOCLICK, SINGLECLICK or DOUBLECLICK ( reset to NOCLICK after call )
  
    void forceChanged(){                        // force all changed with call of next 'pressed', 'released' ore 'changed'
    void resetChanged(){                        // clear alle events of 'pressed', 'released' or 'changed'
                                                 // ( longPress() and shortPress() are unaffected )
   
    Event bits are set at the corresponding edge and they are cleared 
    when read or at the next inverted edge ( pressed-> released or vice versa )

    buttonNbr is a value from 0 to max buttons-1.
 */
 /*
   This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef MOTOBUTTONS_H
#define MOTOBUTTONS_H

#include <Arduino.h>

#ifdef BUTTON_CNT 
    #if BUTTON_CNT>32
        #error "too much buttons"
    #elif BUTTON_CNT>16
        #define MAX32BUTTONS
        #warning "Please use MAX32BUTTONS instead of BUTTON_CNT"
    #elif BUTTON_CNT>8
        #define MAX16BUTTONS
        #warning "Please use MAX16BUTTONS instead of BUTTON_CNT or leave it out completely"
    #else
        #define MAX8BUTTONS
        #warning "Please use MAX8BUTTONS instead of BUTTON_CNT"
    #endif
#endif

#ifdef MAX32BUTTONS
typedef uint32_t button_t;
#elif defined MAX8BUTTONS
typedef uint8_t button_t;
#else
typedef uint16_t button_t;
#endif

#define NOCLICK     0
#define SINGLECLICK 1
#define DOUBLECLICK 2

class MoToButtons {
  public:
   MoToButtons( const uint8_t pinNumbers[], const uint8_t buttonCnt, uint8_t debTime, uint16_t pressTime, uint16_t doubleClick = (400 ) ) {
      _pinCnt = buttonCnt;
      _pinArray = pinNumbers;
      _getHWbuttons = NULL;
      _debTime = debTime;
      _pressTime = pressTime / debTime;   // in debTime tics
      _dClickTime = doubleClick / debTime;
      // Set Pins to INPUT_PULLUP
      // now done in processButtons on first call. It doesn't work here on STM32F4
      //for ( byte i= 0; i < _pinCnt; i++ ) pinMode( pinNumbers[i], INPUT_PULLUP );
      _initLocals( );
    }
    
    MoToButtons( button_t (*getHWbuttons)(), uint8_t debTime, uint16_t pressTime, uint16_t doubleClick = (400 ) ) {
      _getHWbuttons = getHWbuttons;
      _debTime = debTime;
      _pressTime = pressTime / debTime;   // in debTime tics
      _dClickTime = doubleClick / debTime;
      _initLocals( );
    }
    
    
    
    void processButtons() {
      // must be called in loop frequently
      // set input port mode at first call when read directly ( cannot be done in constructor on STM32F4 
      if (_lastReadTime == 0 && !_getHWbuttons ) for ( byte i= 0; i < _pinCnt; i++ ) pinMode( _pinArray[i], INPUT_PULLUP );
      
      if ( millis() - _lastReadTime > (uint32_t) _debTime || _lastReadTime == 0 ) {
        // Read button state with first call or when debounce time has elapsed
        _lastReadTime = millis();
        if ( _getHWbuttons )
            _actState = _getHWbuttons();    // read button states by user function
        else
            _actState = _getPinStates();    // read button states by ínternal function
        // edge detection - new detected edges are added to the corresponding bit field
        // edge bits are cleard when read or at the next inverted edge ( pressed-> released or vice versa )
        // leading edge
        _leadingEdge &= _actState;  // clear bits if button is no longer pressed
        button_t pressEvent  = ~_lastState & _actState;     // new press event 
        button_t releaseEvent  = _lastState & ~_actState;     // new release event 
        _leadingEdge = pressEvent | _leadingEdge;
        // trailing edge
        _trailingEdge &= ~_actState;  // clear bits if button is pressed again
        _trailingEdge = (_lastState & ~_actState) | _trailingEdge ;
        /* only for debugging if ( _lastState != _actState ) {
            char txtBuf[80];
            sprintf(txtBuf, "State= %04X, Lead=%04X, Trail=%04x, press=%04X, rel=%04X", _actState, _leadingEdge, _trailingEdge, pressEvent, releaseEvent );
            Serial.println(txtBuf);
        } */
        _lastState = _actState;
        // process pressing time ( long/short/clicked )
        for ( byte i = 0; i < _buttonCnt; i++ ) {
          if ( _buttonTime[i] < 255 ) _buttonTime[i]++;
          
          if ( bitRead( pressEvent, i ) ) {
            // button is pressed, check doubleClick and reset time counter
            if ( bitRead( _clicked, i ) ) {
              // there is a new press, but the previous click event has not yet been read
              // clear all click events.
              bitClear( _noDoubleClick, i );
              bitClear( _clicked, i );
            } else if ( bitRead( _noDoubleClick , i ) && _buttonTime[i] < _dClickTime ) {
              // Time since last clicked  is short -> it's a double click
              bitSet( _clicked, i );
              bitClear( _noDoubleClick, i );
            }
            bitClear( _longPress, i );
            bitClear( _shortPress, i );
            _buttonTime[i] = 0;
          } else if ( bitRead( releaseEvent, i ) ) {
            // button was released, check if it was presssed long, short or clicked,  
            if (_buttonTime[i] <= _pressTime ) {
                // this release event is a short press
                bitSet( _shortPress, i );
            } else {
                // it was a long press, clear bit to not recognize as short press
                bitClear ( _shortPress, i );
            }

            if ( ! bitRead( _clicked, i ) && bitRead( _noDoubleClick, i ) ) {
              // it was the release of the second click with state already read
              // only clear _noDoubleClick
              bitClear( _noDoubleClick, i );
              #ifdef CLICK_NO_SHORT
                // it was the release of a doubleClick so delete shortpress too
                bitClear( _shortPress, i );
              #endif
            } else if ( _buttonTime[i] < _clickTime ) {
                // it was a first click
                bitSet( _noDoubleClick, i );
            }
          }
          // check if it is a long press ( button is active, and longpresstime elapsed
          if ( bitRead( _actState,i) && _buttonTime[i]>_pressTime &&!bitRead( _shortPress,i )  ) {
            bitSet( _shortPress,i ); // to inhibit further setting of longpress bit while button is still pressed
            bitSet( _longPress,i );
          }
          // check if there was a single click. This has to be done without an button event.
          // It's only time dependent after a click while the button is not pressed again.
          //if ( !bitRead( _actState, i ) && bitRead(_noDoubleClick, i) && !bitRead( _clicked, i ) && _buttonTime[i] > _dClickTime ) {
          if ( bitRead( ~_actState & _noDoubleClick & ~_clicked, i) && _buttonTime[i] > _dClickTime ) {
            // there was a click, and double click time has passed without another click
            // -> it is a single click
            bitSet( _clicked, i );
          }
        }
      }
    }

    byte state( uint8_t buttonNbr ) {            // get static state of button (debounced)
      if ( buttonNbr >= _buttonCnt ) return 0;
      if ( bitRead( _actState, buttonNbr ) ) return _buttonTime[buttonNbr];
      else return 0;
    }

    button_t allStates() {                          // bit field of all buttons (debounced)
       return ( _actState );
    }
    
    button_t changed(){                             // all bits are set where state is different from last call
      button_t temp = _actState ^ _lastChanged;
      _lastChanged = _actState;
      return temp;
    }
    
    void forceChanged(){                             // force all changed with call of next 'pressed', 'released' ore 'changed'
      _lastChanged = ~_actState;
      _leadingEdge = _actState;
      _trailingEdge = ~_actState;
      return;
    }
    
    void resetChanged(){                             // clear alle events of 'pressed', 'released' or 'changed'
      _lastChanged = _actState;
      _leadingEdge = 0;
      _trailingEdge = 0;
      return;
    }
    
    bool shortPress( uint8_t buttonNbr ) {       // if button was pressed short
     if ( buttonNbr >= _buttonCnt ) return 0;
      // get short pressed state of button (debounced)
      bool temp = false;
      if ( !bitRead(_actState,buttonNbr ) ) {
        // shortpress can only be active is button is no longer pressed
        // (_shorPress Bit is also set after lonpress time is elapsed an button is still pressed)
        temp = bitRead( _shortPress, buttonNbr );
        bitClear( _shortPress, buttonNbr );
      }
      return temp;
    }
    bool longPress( uint8_t buttonNbr ) {        // if button was pressed long
      if ( buttonNbr >= _buttonCnt ) return 0;
      // get long pressed state of button (debounced)
      bool temp = bitRead( _longPress, buttonNbr );
      bitClear( _longPress, buttonNbr );
      return temp;
    }

    bool pressed( uint8_t buttonNbr ) {          // leading edge of button press
      if ( buttonNbr >= _buttonCnt ) return 0;
      // get momentarily pressed state of button (debounced)
      bool temp = bitRead( _leadingEdge, buttonNbr );
      bitClear( _leadingEdge, buttonNbr );
      return temp;
    }
    byte released( uint8_t buttonNbr ) {         // trailing edge of button press
      if ( buttonNbr >= _buttonCnt ) return 0;
      // get momentarily released state of button (debounced)
      bool temp = bitRead( _trailingEdge, buttonNbr );
      bitClear( _trailingEdge, buttonNbr );
      if ( temp ) return  _buttonTime[buttonNbr];
      else return 0;
    }

    uint8_t clicked( uint8_t buttonNbr ) {         // single/double click of button press
      if ( buttonNbr >= _buttonCnt ) return 0;
      // get click state of button (debounced)
      uint8_t clickType = NOCLICK;
      #ifdef CLICK_NO_SHORT
        // if __noDoubleClick is set it is at least a single click, so delete shortpress
        if ( bitRead( _noDoubleClick, buttonNbr ) )bitClear( _shortPress, buttonNbr );
      #endif
      if (bitRead( _clicked, buttonNbr ) ) {
        if ( bitRead( _noDoubleClick, buttonNbr) ) clickType = SINGLECLICK;
        else clickType = DOUBLECLICK;
        // if button of second press ist still pressed, set _no_DoubleClick
        // it will be reset when the button is released ( to mark that this
        // is not a new click )
        if ( bitRead( _actState, buttonNbr ) ) bitSet( _noDoubleClick, buttonNbr );
        else bitClear( _noDoubleClick, buttonNbr );
        //bitWrite( _noDoubleClick, buttonNbr, bitRead( _actState, buttonNbr )  );
        bitClear( _clicked, buttonNbr );
      }
      return clickType;
    }

    private:
    button_t _getPinStates() {
      // read pins to get the HW state of the buttons
      button_t buttonTemp = 0;
      for (byte i = 0; i < _pinCnt; i++) {
        bitWrite( buttonTemp,i,!digitalRead(*(_pinArray+i)) ); 
      }
      return buttonTemp;
    }
    
    void _initLocals() {
      _lastReadTime = 0;     // Last time HW state was read
      _clickTime = _dClickTime/2;
      // Bit fields to hold various button states
      _lastState = 0;
      _lastChanged = 0;
      _actState = 0;
      _longPress = 0;
      _shortPress = 0;
      _leadingEdge = 0;
      _trailingEdge = 0;
      for ( byte i = 0; i < _buttonCnt; i++ ) {
        _buttonTime[ i ] = 0; // Time in debounce tics
      }
    }
    uint8_t _pinCnt;
    const uint8_t *_pinArray;
    uint8_t _debTime;            // Debounce time im ms
    uint8_t _pressTime;          // pressTime measured in debounce tics
    uint8_t _clickTime;
    uint8_t _dClickTime;        // double click time measured in debouce tics
    uint32_t _lastReadTime;     // Last time HW state was read
    static const uint8_t   _buttonCnt = sizeof(button_t)*8;        // Number of buttons
    button_t  (*_getHWbuttons)();  // Ptr to user function to read raw state of buttons
    // Bit fields to hold various button states
    button_t  _lastState;
    button_t  _lastChanged;
    button_t  _actState;
    button_t  _longPress;       // Set after longpresstime has elapsed and button is still pressed.
    button_t  _shortPress;      // Bit is also set after longpress time has elapsed until button is released
    button_t  _leadingEdge;
    button_t  _trailingEdge;
    // The next two Bits work together: After the first click (release Event) _noDoubleClick
    // set. If there is a new pressing event within double click time, _clicked is set, and
    // _noDoubleClick is reset. This is recognized as doubleClick Event in method 'clicked'.
    // if double click time passes without a new pressing event, _noDoubleClick and _clicked are
    // both set. This is recognized as single click.
    button_t  _noDoubleClick;
    button_t  _clicked;
    uint8_t   _buttonTime[ _buttonCnt ]; // Time in debounce tics

  

};


#endif
