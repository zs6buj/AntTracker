#include <Arduino.h>

#if (PROTOCOL == 0)  //  AUTO  
  #if (MEDIUM_IN == 1) || (MEDIUM_IN == 3) || (MEDIUM_IN == 4)// UART or BT or BLE 4.2
    uint16_t Lth=0;
    uint8_t Mav1 = 0;
    uint8_t Mav2 = 0;
    uint8_t SPort = 0;
    uint8_t FPort1 = 0;
    uint8_t FPort2 = 0;
    uint8_t LTM = 0;
    uint8_t MSP = 0;
    uint8_t NMEA_GPS = 0;
    uint8_t CRSF = 0;
    uint16_t  q = 0;
    uint16_t  r = 0;
    char ch[3];        // rolling input bytes

    //====================================================
    uint32_t SenseUart(uint8_t  pin) {

      uint32_t pw = 999999;  //  Pulse width in uS
      uint32_t min_pw = 999999;
      uint32_t su_baud = 0;
      const uint32_t su_timeout = 5000; // uS !  Default timeout 1000mS!
      
        #if defined DEBUG_ALL || defined DEBUG_BAUD
          log.printf("pin:%d  in_invert:%d\n", pin, in_invert);        
        #endif  
      
        if (in_invert) {
          while(digitalRead(pin) == 0){ };  // idle_low, wait for high bit (low pulse) to start
        } else {
          while(digitalRead(pin) == 1){ };  // idle_high, wait for low bit (high pulse) to start  
        }
        for (int i = 0; i < 10; i++) {
      
          if (in_invert) {               
            pw = pulseIn(pin,HIGH, su_timeout);     //  Returns the length of the pulse in uS
          } else {
            pw = pulseIn(pin,LOW, su_timeout);    
          }    
      
          if (pw !=0) {
            min_pw = (pw < min_pw) ? pw : min_pw;  // Choose the lowest
            //log.printf("i:%d  pw:%d  min_pw:%d\n", i, pw, min_pw);    
          } 
        } 
        #if defined DEBUG_ALL || defined DEBUG_BAUD
          log.printf("pw:%d  min_pw:%d\n", pw, min_pw);         
        #endif
      
        switch(min_pw) {   
          case 1:     
          su_baud = 921600;
            break;
          case 2:     
          su_baud = 460800;
            break;     
          case 4 ... 11:     
          su_baud = 115200;
            break;
          case 12 ... 19:  
          su_baud = 57600;
            break;
          case 20 ... 28:  
          su_baud = 38400;
            break; 
          case 29 ... 39:  
          su_baud = 28800;
            break;
          case 40 ... 59:  
          su_baud = 19200;
            break;
          case 60 ... 79:  
          su_baud = 14400;
            break;
          case 80 ... 149:  
          su_baud = 9600;
            break;
          case 150 ... 299:  
          su_baud = 4800;
            break;
          case 300 ... 599:  
          su_baud = 2400;
            break;
          case 600 ... 1199:  
          su_baud = 1200;  
            break;                        
          default:  
          su_baud = 0;    // no signal        
      }
        return su_baud;
      } 
    //=================================================
    uint32_t GetConsistent(uint8_t pin) {
      uint32_t t_baud[5];

      while (true) {  
        t_baud[0] = SenseUart(pin);
        delay(10);
        t_baud[1] = SenseUart(pin);
        delay(10);
        t_baud[2] = SenseUart(pin);
        delay(10);
        t_baud[3] = SenseUart(pin);
        delay(10);
        t_baud[4] = SenseUart(pin);
        #if defined DEBUG_ALL || defined DEBUG_BAUD
          log.print("  t_baud[0]="); log.print(t_baud[0]);
          log.print("  t_baud[1]="); log.print(t_baud[1]);
          log.print("  t_baud[2]="); log.print(t_baud[2]);
          log.print("  t_baud[3]="); log.println(t_baud[3]);
        #endif  
        if (t_baud[0] == t_baud[1]) {
          if (t_baud[1] == t_baud[2]) {
            if (t_baud[2] == t_baud[3]) { 
              if (t_baud[3] == t_baud[4]) {   
                #if defined DEBUG_ALL || defined DEBUG_BAUD    
                  log.print("Consistent baud found="); log.println(t_baud[3]); 
                #endif   
                return t_baud[3]; 
              }          
            }
          }
        }
      }
    }
    //===========================================
    uint16_t detectProtocol(uint32_t baud) 
    {
      log.println("Detecting protocol...");  
      delay(100);  
      #if (MEDIUM_IN == 1) // UART
        #if ( (defined ESP8266) || (defined ESP32) ) 
          inSerial.begin(baud, SERIAL_8N1, in_rxPin, in_txPin, in_invert);
        #elif (defined TEENSY3X) 
          inSerial.begin(baud); // Teensy 3.x    tx pin hard wired
          if (in_invert) 
          {          // For S.Port not F.Port
            UART0_C3 = 0x10;       // Invert Serial1 Tx levels
            UART0_S2 = 0x10;       // Invert Serial1 Rx levels;       
          }
          #if (defined frOneWire )  // default
            UART0_C1 = 0xA0;        // Switch Serial1 to single wire (half-duplex) mode  
          #endif    
        #else
            inSerial.begin(baud);
        #endif    
      #endif

      // now loop until char pattern found
      chr = nextByte();  // wait for the first char
      uint8_t found = 0;
      while (found == 0) 
      {   
        if (chr==0xFE) 
        {   
          int pl = nextByte();   // Payload length
          if (pl>=8 && pl<=37) 
          {
            Mav1++;                  // Found candidate Mavlink 1
            if (Mav1>=10) {
              found = 1;
              exit;
            }
          }
        }
        if (chr==0xFD) 
        {
          Mav2++;                     // Found candidate Mavlink 2
          if (Mav2>=10) 
          {
              found = 2;
              exit;
          }
        }
        if (chr==0x7E)                             // Could be S.Port or F.Port1
          if (ch[1]==0x7E) 
          {                       // must be FPort1
            FPort1++;
            if (SPort) SPort--;                    // remove the trailing 0x7E count                           
            if (FPort1>=10) 
            {
              found = 4;
              exit;  
            }
        } else 
        {                                   // else it's SPort
          SPort++;                                
          if (SPort>=10) 
          {
              found = 3;
              exit;      
          }
        } 

        /*  FPORT2
            ctl = (((prev_chr == 0x0D) || (prev_chr == 0x18)|| (prev_chr == 0x23)) && (chr == 0xFF)); 
            ota = (((prev_chr == 0x0D) || (prev_chr == 0x18)|| (prev_chr == 0x23)) && (chr == 0xF1)); 
            dlink = ((prev_chr == 0x08)  && (chr == 0x1B));   
        */
        if ( ( (ch[1] == 0x0D) || (ch[1] == 0x18)|| (ch[1] == 0x23) ) && (ch[2] == 0xF1)  )  
        {  // fp2 OTA     
          FPort2++; 
          //log.printf("ch[0]%2X ch[1]%2X  ch[2]%2X FPort2a:%d\n", ch[0], ch[1], ch[2], FPort2);        
          if (FPort2>=10) 
          {
              found = 5;
              exit;           
          }
        } else  
        if ( (ch[1] == 0x08) && ( (ch[2] >= 0) && (ch[2] < 0x1b) ) )   
        {     // fp2 downlink  
          FPort2++; 
          //log.printf("ch[0]%2X ch[1]%2X  ch[2]%2X FPort2b:%d\n", ch[0], ch[1], ch[2], FPort2);   
          if (FPort2>=10) {
              found = 5;
              exit; 
          }             
        } 
        /*
        if (chr==0x24) {    //  'S'
          chr = nextByte(); // start2 should be 0x54 ('T')
          if (chr==0x54) LTM++;                     // Found candidate LTM
        }
        */

        if ((ch[0]=='$') && (ch[1]=='M') && (ch[2]=='<')) 
        {
          MSP++;       // Found candidate MSP 
          if (MSP>=10) 
          {
              found = 7;
              exit;
          }
        }

        if ((ch[0]=='$') && (ch[1]=='G') && ((ch[2]=='N') || (ch[2]=='P') ) ){
          NMEA_GPS++; // Found candidate GPS
          if (NMEA_GPS>=10) 
          {
              found = 8;
              exit;     
          }
        }

        if ((ch[0]=='$') && (ch[1]=='G') && ((ch[2]=='N') || (ch[2]=='P') ) ){
          NMEA_GPS++; // Found candidate GPS
          if (NMEA_GPS>=10) {
              found = 8;
              exit;  
          }
        }  
        
        if ( (ch[0]==CRSF_TEL_SYNC_BYTE) && ((ch[2]== 0x02) || (ch[2]== 0x08) || (ch[2]== 0x14) || (ch[2]== 0x1E) || (ch[2]== 0x21) ) ) 
        {
          CRSF++;       // Found candidate CRSF
          //log.printf("ch[0]%2X ch[1]%2X  ch[2]%2X CRSF:%d\n", ch[0], ch[1], ch[2], CRSF);  
          if (CRSF>=10) 
          {
            found = 9;
            exit;
          }
        }

      #if defined DEBUG_ALL || defined DEBUG_PROTOCOL
        if (Mav1)
        {
          log.print("Mav1=");  log.println(Mav1);
        }
        if (Mav2)
        {
          log.print("Mav2=");  log.println(Mav2);
        } 
        if (SPort)
        {
          log.print("SPort=");  log.println(SPort);
        } 
        if (FPort1)    
        {
              log.print("FPort1=");  log.println(FPort1);  
        } 
        if (FPort2)    
        {
          log.print("FPort2=");  log.println(FPort2); 
        }   
        if (LTM)    
        {
          log.print("LTM=");  log.println(LTM);
        }   
        if (MSP)    
        {
        log.print("MSP=");  log.println(MSP);
        }             
        if (NMEA_GPS)    
        {
          log.print("NMEA_GPS=");  log.println(NMEA_GPS);
        } 
        if (CRSF)    
        {
          log.print("CRSF=");  log.println(CRSF);
        } 
      #endif

      chr = nextByte(); 
      ch[0] = ch[1];   // rolling input bytes
      ch[1] = ch[2];
      ch[2] = chr;   
      } // loop until 10 found

      delay(5); 
      #if (MEDIUM_IN == 1)
        delay(5);
        inSerial.end();
        delay(5);
      #endif
      return found;
    }  //  end of detectProtocol()

    //=======================================================
    uint32_t getBaud(uint8_t pin) {
      log.print("autoBaud - sensing pin "); log.println(pin);
    //log.printf("autoBaud - sensing pin %2d \n", pin);    
      uint8_t i = 0;
      uint8_t col = 0;
      pinMode(pin, INPUT);       
      digitalWrite (pin, HIGH); // for noise reduction ?

      uint32_t gb_baud = GetConsistent(pin);
      while (gb_baud == 0) {
        if(ftgetBaud) {
          ftgetBaud = false;
        }

        i++;
        if ((i % 5) == 0) {
          log.print(".");
          col++; 
        }
        if (col > 60) {
          log.println(); 
            log.print("No telemetry found on pin "); log.println(pin);
      //    log.printf("No telemetry found on pin %2d\n", pin); 
          col = 0;
          i = 0;
        }
        gb_baud = GetConsistent(pin);
      } 
      if (!ftgetBaud) {
        log.println();
      }

      //log.print("Telem found at "); log.print(gb_baud);  log.println(" b/s");
      //logScreenPrintln("Telem found at " + String(gb_baud));
      return(gb_baud);
    }
  #endif  // end of protocol inclusion
#endif  // end of #if (PROTOCOL == 0)  //  AUTO  

//======================================================
pol_t getPolarity(uint8_t pin) {
  uint32_t pw_hi = 0;
  uint32_t pw_lo = 0;   
  
  for (int i = 0; i < 1000; i++) {
    pw_lo += (digitalRead(pin) == LOW); 
    pw_hi += (digitalRead(pin) == HIGH);         
    delayMicroseconds(500); // so test for 0.5 seconds
  }  

  //log.printf("hi:%d  lo:%d\n", pw_hi, pw_lo);  
  if ( ( (pw_lo == 1000) && (pw_hi == 0) ) || ((pw_lo == 0) && (pw_hi == 1000) ) ) 
  {
    return no_traffic;
  }
  if (pw_hi > pw_lo) {
    return idle_high;
  } else {
    return idle_low;        
  }
}   
