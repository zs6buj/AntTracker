#ifndef MOTODEBUG_H
#define MOTODEBUG_H
// die folgenden defines werden im aufrufenden cpp-File gesetzt.
// so können die debugs klassenspezifisch eingeschaltet werden
//#define debugTP
//#define debugPrint

// über diese undefs kann das Debugging global abgeschaltet werden
#undef debugTP
#undef debugPrint

#ifdef debugTP 
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        #define MODE_TP1 DDRF |= (1<<2) //pinA2
        #define SET_TP1 PORTF |= (1<<2)
        #define CLR_TP1 PORTF &= ~(1<<2)
        #define MODE_TP2 DDRF |= (1<<3) //pinA3
        #define SET_TP2 PORTF |= (1<<3)
        #define CLR_TP2 PORTF &= ~(1<<3)
        #define MODE_TP3 DDRF |= (1<<4) //pinA4 
        #define SET_TP3 PORTF |= (1<<4) 
        #define CLR_TP3 PORTF &= ~(1<<4) 
        #define MODE_TP4 DDRF |= (1<<5) //pinA5 
        #define SET_TP4 PORTF |= (1<<5) 
        #define CLR_TP4 PORTF &= ~(1<<5) 
    #elif defined(__AVR_ATmega32U4__)
        #define MODE_TP1 DDRF |= (1<<0) //A5
        #define SET_TP1 PORTF |= (1<<0)
        #define CLR_TP1 PORTF &= ~(1<<0)
        #define MODE_TP2 DDRF |= (1<<1) //A4
        #define SET_TP2 PORTF |= (1<<1)
        #define CLR_TP2 PORTF &= ~(1<<1)
        #define MODE_TP3 DDRC |= (1<<7) //13
        #define SET_TP3 PORTC |= (1<<7)
        #define CLR_TP3 PORTC &= ~(1<<7)
        #define MODE_TP4 DDRB |= (1<<7) //11
        #define SET_TP4 PORTB |= (1<<7)
        #define CLR_TP4 PORTB &= ~(1<<7)
    #elif defined(__AVR_ATmega328P__) 
        #warning "Debug-Ports active"
        #define MODE_TP1 DDRC |= (1<<0) //A0
        #define SET_TP1 PORTC |= (1<<0)
        #define CLR_TP1 PORTC &= ~(1<<0)
        #define MODE_TP2 DDRC |= (1<<1) // A1
        #define SET_TP2 PORTC |= (1<<1)
        #define CLR_TP2 PORTC &= ~(1<<1)
        #define MODE_TP3 DDRC |= (1<<2) //A2
        #define SET_TP3 PORTC |= (1<<2) 
        #define CLR_TP3 PORTC &= ~(1<<2) 
        #define MODE_TP4 DDRC |= (1<<3) //A3 
        #define SET_TP4 PORTC |= (1<<3) 
        #define CLR_TP4 PORTC &= ~(1<<3) 
//    #elif defined(__AVR_ATmega4809__) // Every and UNO Rev2 have different mappings regarding A0...A7
    #elif defined (ARDUINO_AVR_NANO_EVERY)
        #warning "Debug-Ports active"
        #define MODE_TP1 pinMode(A0, OUTPUT ) // A0 = PD3
        #define SET_TP1 PORTD_OUTSET = (1<<3)
        #define CLR_TP1 PORTD_OUTCLR = (1<<3)
        #define MODE_TP2 pinMode(A1,OUTPUT)	// A1 = PD2
        #define SET_TP2 PORTD_OUTSET = (1<<2)
        #define CLR_TP2 PORTD_OUTCLR = (1<<2)
        #define MODE_TP3
        #define SET_TP3
        #define CLR_TP3
        #define MODE_TP4
        #define SET_TP4
        #define CLR_TP4
    #elif defined (ARDUINO_AVR_UNO_WIFI_REV2)
        #warning "Debug-Ports active"
        #define MODE_TP1 pinMode(A0, OUTPUT ) // A0 = PD0
        #define SET_TP1 PORTD_OUTSET = (1<<0)
        #define CLR_TP1 PORTD_OUTCLR = (1<<0)
        #define MODE_TP2 pinMode(A1,OUTPUT)	// A1 = PD1
        #define SET_TP2 PORTD_OUTSET = (1<<1)
        #define CLR_TP2 PORTD_OUTCLR = (1<<1)
        #define MODE_TP3
        #define SET_TP3
        #define CLR_TP3
        #define MODE_TP4
        #define SET_TP4
        #define CLR_TP4
    #elif defined(ARDUINO_AVR_ATTINYX4) 
        #warning "Debug-Ports active"
        #define MODE_TP1 DDRA |= (1<<1) //1
        #define SET_TP1 PORTA |= (1<<1)
        #define CLR_TP1 PORTA &= ~(1<<1)
        #define MODE_TP2 DDRA |= (1<<2) // 2
        #define SET_TP2 PORTA |= (1<<2)
        #define CLR_TP2 PORTA &= ~(1<<2)
        #define MODE_TP3 DDRA |= (1<<3) //3
        #define SET_TP3 PORTA |= (1<<3) 
        #define CLR_TP3 PORTA &= ~(1<<3) 
        #define MODE_TP4 DDRB |= (1<<0) //11 
        #define SET_TP4 PORTB |= (1<<0) 
        #define CLR_TP4 PORTB &= ~(1<<0) 
    #elif defined(__AVR_ATmega128__) ||defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
    #elif defined (__SAM3X8E__)
        // Arduino Due
        #define MODE_TP1 pinMode( A1,OUTPUT )   // A1= PA24
        #define SET_TP1  REG_PIOA_SODR = (1<<24)
        #define CLR_TP1  REG_PIOA_CODR = (1<<24)
        #define MODE_TP2 pinMode( A2,OUTPUT )   // A2= PA23
        #define SET_TP2  REG_PIOA_SODR = (1<<23)
        #define CLR_TP2  REG_PIOA_CODR = (1<<23)
        #define MODE_TP3 pinMode( A3,OUTPUT )   // A3 = PA22
        #define SET_TP3  REG_PIOA_SODR = (1<<22)
        #define CLR_TP3  REG_PIOA_CODR = (1<<22)
        #define MODE_TP4 pinMode( A4,OUTPUT )   // A4 = PA6
        #define SET_TP4  REG_PIOA_SODR = (1<<6)
        #define CLR_TP4  REG_PIOA_CODR = (1<<6)
    #elif defined (__STM32F1__)
        //Test-HW füer Stepper:
        #define TP1 PB1
        #define TP2 PB0
        #define TP3 PB13 
        #define TP4 PB15
        #define MODE_TP1 pinMode( TP1,OUTPUT )   // TP1= PA1
        #define SET_TP1  digitalWrite( TP1, HIGH )
        #define CLR_TP1  digitalWrite( TP1, LOW )
        #define MODE_TP2 pinMode( TP2,OUTPUT )  
        #define SET_TP2  digitalWrite( TP2, HIGH )
        #define CLR_TP2  digitalWrite( TP2, LOW )
        #define MODE_TP3 pinMode( TP3,OUTPUT )   // 
        #define SET_TP3  digitalWrite( TP3, HIGH )
        #define CLR_TP3  digitalWrite( TP3, LOW )
        #define MODE_TP4 pinMode( TP4,OUTPUT )   // 
        #define SET_TP4  digitalWrite( TP4, HIGH )
        #define CLR_TP4  digitalWrite( TP4, LOW )
    #elif defined ESP8266
        #define MODE_TP1 pinMode( 15,OUTPUT )  // GPIO 15
        #define SET_TP1  GPOS = (1 << 15)
        #define CLR_TP1  GPOC = (1 << 15)
        #define MODE_TP2 pinMode( 13,OUTPUT )  
        #define SET_TP2  digitalWrite( 13, HIGH );
        #define CLR_TP2  digitalWrite( 13, LOW );
        #define MODE_TP3 
        #define SET_TP3 
        #define CLR_TP3
        #define MODE_TP4 
        #define SET_TP4 
        #define CLR_TP4 
         
    #elif defined ESP32
        #define TP1 15
        #define TP2 21
        #define TP3 14
        #define TP4 13
        #define MODE_TP1 pinMode( TP1,OUTPUT )  // GPIO 13
        #define SET_TP1  digitalWrite( TP1, HIGH )
        #define CLR_TP1  digitalWrite( TP1, LOW )
        #define MODE_TP2 pinMode(TP2,OUTPUT )  
        #define SET_TP2  digitalWrite( TP2, HIGH )
        #define CLR_TP2  digitalWrite( TP2, LOW )
        #define MODE_TP3  pinMode(TP3,OUTPUT )  
        #define SET_TP3   digitalWrite( TP3, HIGH )
        #define CLR_TP3  digitalWrite( TP3, LOW )
        #define MODE_TP4  pinMode(TP4,OUTPUT )  
        #define SET_TP4   digitalWrite( TP4, HIGH )
        #define CLR_TP4  digitalWrite( TP4, LOW ) 
    
	#elif defined ARDUINO_ARCH_RENESAS_UNO 
	    //#warning "Debug ra4M1 aktiv" 
        #define TP1 BSP_IO_PORT_00_PIN_14 // (14) A0
        #define TP2 BSP_IO_PORT_00_PIN_00 //* (15) A1
        #define TP3 BSP_IO_PORT_00_PIN_01 /* (16) A2  */
        #define TP4 BSP_IO_PORT_00_PIN_02 //* (17) A3 
        #define MODE_TP1 pinMode( A0,OUTPUT )  
		//#define SET_TP1 digitalWrite(A0,HIGH)
		//#define CLR_TP1 digitalWrite(A0,LOW);
        #define SET_TP1  R_IOPORT_PinWrite(NULL, TP1, BSP_IO_LEVEL_HIGH)
        #define CLR_TP1  R_IOPORT_PinWrite(NULL, TP1, BSP_IO_LEVEL_LOW)
        #define MODE_TP2 pinMode(A1,OUTPUT )  
        #define SET_TP2  R_IOPORT_PinWrite(NULL, TP2, BSP_IO_LEVEL_HIGH)
        #define CLR_TP2  R_IOPORT_PinWrite(NULL, TP2, BSP_IO_LEVEL_LOW)
        #define MODE_TP3  pinMode(A2,OUTPUT )  
        #define SET_TP3   R_IOPORT_PinWrite(NULL, TP3, BSP_IO_LEVEL_HIGH)
        #define CLR_TP3  R_IOPORT_PinWrite(NULL, TP3, BSP_IO_LEVEL_LOW)
        #define MODE_TP4  pinMode(A3,OUTPUT )  
        #define SET_TP4   R_IOPORT_PinWrite(NULL, TP4, BSP_IO_LEVEL_HIGH)
        #define CLR_TP4  R_IOPORT_PinWrite(NULL, TP4, BSP_IO_LEVEL_LOW) 
	
    #elif defined ARDUINO_ARCH_RP2040
		#warning "RP2040 TP aktiv"
        #define TP1 20
        #define TP2 21
        #define TP3 22
        #define TP4 28
        #define MODE_TP1 pinMode( TP1,OUTPUT ) 
        #define SET_TP1  gpio_set_mask( 1<<TP1 )
        #define CLR_TP1  gpio_clr_mask( 1<<TP1 )
        #define MODE_TP2 pinMode(TP2,OUTPUT )  
        #define SET_TP2  gpio_set_mask( 1<<TP2 )
        #define CLR_TP2  gpio_clr_mask( 1<<TP2 )
        #define MODE_TP3  pinMode(TP3,OUTPUT )  
        #define SET_TP3   gpio_set_mask( 1<<TP3 )
        #define CLR_TP3  gpio_clr_mask( 1<<TP3 )
        #define MODE_TP4  pinMode(TP4,OUTPUT ) 
        #define SET_TP4   gpio_set_mask( 1<<TP4 )
        #define CLR_TP4  gpio_clr_mask( 1<<TP4 )
 
	#else // processor not known
		//#warning "no testpins - processor unknown"
        #define MODE_TP1
        #define SET_TP1
        #define CLR_TP1
        #define MODE_TP2
        #define SET_TP2
        #define CLR_TP2
        #define MODE_TP3 
        #define SET_TP3 
        #define CLR_TP3 
        #define MODE_TP4 
        #define SET_TP4 
        #define CLR_TP4 
    #endif 

    
#else
	//#warning "no testpins activated"
    #define MODE_TP1 
    #define SET_TP1 
    #define CLR_TP1 
    #define MODE_TP2 
    #define SET_TP2 
    #define CLR_TP2 
    #define MODE_TP3 
    #define SET_TP3 
    #define CLR_TP3
    #define MODE_TP4 
    #define SET_TP4 
    #define CLR_TP4 
// special Testpoint for Servo-testing
    #define SET_SV3 
    #define CLR_SV3 
    #define SET_SV4 
    #define CLR_SV4 

#endif
        //#define MODE_TP3 pinMode( PB14,OUTPUT )   // TP3 = PB14
        //#define SET_SV3  gpio_write_bit( GPIOB,14, HIGH )
        //#define CLR_SV3  gpio_write_bit( GPIOB,14, LOW )
        //#define MODE_TP4 pinMode( PB15,OUTPUT )   // TP4 = PB15
        //#define TOG_TP4  gpio_toggle_bit( GPIOB,15)
        //#define TOG_TP2  gpio_toggle_bit( GPIOB,13)
        #define TOG_TP4
        #define TOG_TP2


#ifdef debugPrint
    #warning "Debug-printing is active"
	#ifdef ARDUINO_ARCH_AVR
        //#define DB_PRINT( x, ... ) {char dbgBuf[80]; sprintf_P( dbgBuf, PSTR( x ), ##__VA_ARGS__ ) ; Serial.println( dbgBuf ); }
        #define DB_PRINT( x, ... ) {char dbgBuf[80]; snprintf_P( dbgBuf, 80, PSTR( x ), ##__VA_ARGS__ ) ; Serial.println( dbgBuf ); }
        //#define DB_PRINT( x, ... ) {char dbgBuf[80]; sprintf( dbgBuf,  x , ##__VA_ARGS__ ) ; Serial.println( dbgBuf ); }
    #elif defined ARDUINO_ARCH_ESP32
        #define DB_PRINT( x, ... ) Serial.printf(  x , ##__VA_ARGS__ ) 
    #else
        #define DB_PRINT( x, ... ) {char dbgBuf[80]; sprintf( dbgBuf,  x , ##__VA_ARGS__ ) ; Serial.println( dbgBuf ); }
    #endif
    extern const char *rsC[] ;    
#else
    #define DB_PRINT( x, ... ) ;
#endif

#endif