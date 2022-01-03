    //    0x400 iNav Flight Mode
    uint32_t pt400_flight_mode;
    uint8_t  pt400_arm_flag = 0;
    
    //    0x410 iNav GPS Status
    uint32_t pt410_gps_status;
    uint8_t pt_gps_fix;
    uint8_t pt_gps_homefix;
    uint8_t pt_gps_homereset;
    uint8_t pt_gps_accuracy;     // 0 thru 9 highest accuracy
    uint16_t pt_gps_numsats;
       
    // 0x800 GPS
    uint32_t pt_latlong;
    int32_t  pt_lat = 0;
    int32_t  pt_lon = 0;

    //FrSky Variables
    short ms2bits;
    uint32_t pt_payload;

    // 0x5002 GPS Status
    uint8_t pt_numsats;
    uint8_t pt_gps_status;           // part a
    uint8_t pt_gps_adv_status;       // part b
    uint16_t pt_hdop;                // @rotorman  2021/01/18
    int32_t pt_amsl;                // decimetres
    float   pt_famsl;               // float decimetres
    uint8_t neg;
    
    //0x5003 Batt
    int16_t pt_bat1_volts;     // dV (V * 10)
    int16_t pt_bat1_amps;      // dA )A * 10)
    uint16_t pt_bat1_mAh;
    
    // 0x5004 Home
    uint32_t pt_home;   
    uint16_t pt_home_dist;
    int16_t  pt_home_angle;       // degrees
    int16_t  pt_home_arrow;       // 0 = heading pointing to home, unit = 3 degrees
    int16_t  pt_home_alt;    
    float fHomeDist;   
    
    // 0x5005 Velocity and yaw
    uint32_t pt_velyaw;
    float pt_vy;    // climb in decimeters/s
    float pt_vx;    // groundspeed in decimeters/s
    float pt_yaw;   // heading units of 0.2 degrees
       
    // 0x5006 Attitude and range
    uint16_t pt_roll;
    uint16_t pt_pitch;
    uint16_t pt_range;
    float    pt_froll;
    float    pt_fpitch;
    float    pt_frange; 
       
    //0x5008 Batt    
    uint16_t pt_bat2_mAh;
    
    //0xF101 RSSI 
    uint32_t pt_rssi; 


#if (Telemetry_In == 0) || (Telemetry_In == 3) ||  (Telemetry_In == 4)    //  FrSky (Serial, UDP or BT)

  bool lonGood = false;
  bool latGood = false;
  bool altGood = false;

  bool Passthru = false;
  bool d_dia = false;
  bool x_dia = false;

  bool iNav = false;

  // General FrSky
  boolean FT = true;

  short crc=0;  
  boolean crc_bad; 
  
  // variables for iNav 0x410 decode
  uint16_t d1234;
  uint8_t  d1;
  uint16_t dr1;
  uint8_t  d2;
  uint16_t dr2;
  uint8_t  d3;
  uint16_t dr3;
  uint8_t  d4;
  
  uint8_t d14;      // say 7 / 4 = 1
  uint8_t dr12;     //     7 % 4 = 3
  uint8_t d12;      //     3 / 2 = 1
  uint8_t d11;      //     3 % 2 = 1

 
  uint16_t lonDDMM;
  uint16_t latDDMM;
  uint16_t DD;
  uint16_t MM;
  uint16_t mmmm;
  float MMmmmm;
  char NS;   // No kidding!
  char EW; 
     
  frport_t      frport;
  
  uint32_t pt_heading;
  uint32_t pt_altitude;

  short pt_pwr;
  uint32_t pt_gps;

  int16_t gpsAlt;
  
  uint16_t  fr_idx = 0;
  uint8_t   fr_offset = 0;

    bool parseGood = false;
  
    static const uint8_t max_ch = 26;         // 24 + 2 digi ch
    int16_t   pwm_ch[max_ch];                 // PWM Channels
    uint8_t   pwm_rssi = 0;
  
    int16_t   crcin = 0;                      // CRC of inbound frsky frame   
    int16_t   crcout = 0;   
    uint16_t  lth = 0;

    uint8_t   fr_lth = 0;
    uint8_t   fr_type = 0;
    uint8_t   fr_prime = 0;
    
    // Forward Declarations
    void CheckStatusAndTimeouts();

     
    //=======================================================================
    void FrSky_Receive(uint8_t proto) {
      #if (Telemetry_In == 0) ||  ((defined btBuiltin) &&  (Telemetry_In == 4))       //   FrSky Serial or BT
        Frs_Receive_Serial_BT(proto);
      #endif
   
      #if (defined wifiBuiltin) &&  (Telemetry_In == 3)  //   FrSky UDP 
        Frs_Receive_UDP(proto);
      #endif
      
      
    }
    //=======================================================================
    #if (defined wifiBuiltin) &&  (Telemetry_In == 3)          // UDP
    void Frs_Receive_UDP(uint8_t proto) {  // proto S.Port only for now

      uint16_t len = frs_udp_object.parsePacket();   // packet to in buffer
      if (len == 0) return;
      for (int i = 0 ; i < len ; i++) {
        inBuf[i] = frs_udp_object.read();
        //snprintf(snprintf_buf, snp_max, "byte:%X  i:%d\n", inBuf[i], i);
        //Log.print(snprintf_buf);
      }
  
      //Log.print("A " );  PrintFrsBuffer(&inBuf[0], len-fr_offset); 
    
      if (len == 12) { // S.Port (0x7E + 0x1B) + (0x7E + 0x1B) + frame
        fr_offset = 4;
      } else
      if (len == 10) {  // S.Port 0x7E + 0x1B) + frame
        fr_offset = 2;
      } else
      if (len == 8) {  // F.Port frame
        fr_offset = 0;
      }
      //snprintf(snprintf_buf, snp_max, "B ");  PrintFrsBuffer(&inBuf[fr_offset], len-fr_offset);
      //Log.print(snprintf_buf);
      bool mycrcGood = crcGood(&inBuf[fr_offset], len-fr_offset); 

      if (mycrcGood) {  
        frGood = true;
        frGood_millis = millis();  
        #if defined Debug_All || defined Debug_FrSky_Messages_UDP
          Log.print("CRC Good C "); PrintFrsBuffer(&inBuf[fr_offset], len-fr_offset);
        #endif 
        Frs_Decode(&inBuf[fr_offset]);   
      }
    }
    #endif

    //================================================================
  #if (Telemetry_In == 0) || ( (defined btBuiltin) &&  (Telemetry_In == 4) )  // Serial or BT

    void Frs_Receive_Serial_BT(uint8_t proto){

      if (proto == 3) {  // proto for protocol detect
        frport = s_port;  
      } else
      if (proto == 4) {
        frport = f_port1;  
      } else      
      if (proto == 5) {
        frport = f_port2;  
      }  
      
      if (frport == s_port) {                            // S.Port
        if ( SPort_Read_A_Frame(&inBuf[0]) ) {
          #if (defined Debug_FPort_Buffer) 
            Log.print("Good FrSky Frame Read: ");
            PrintFrsBuffer(inBuf, 10); //magic, lth, type, prime, payload[6], crc
          #endif           
          Frs_Decode(&inBuf[2]);  
        }
      }
  
      if ( (frport == f_port1) || (frport == f_port2) ) {       // F.Port
        if (FPort_Read_A_Frame(&inBuf[0], frport) ) {
          #if (defined Debug_FPort_Buffer) 
            Log.print("Good FrSky Frame Read: ");
            PrintFrsBuffer(inBuf, 11);    // null, lth, type, prime, payload[6], crc
          #endif            
          Frs_Decode(&inBuf[2]);     
        }    
      }    
    }
    
    //===================================================================  
    bool SPort_Read_A_Frame(uint8_t *buf) {

      static uint8_t i = 0;
      byte b;
      
      #if defined Report_Packetloss
       uint32_t period = (Report_Packetloss * 60000);
       if (millis() - packetloss_millis > period) {
         packetloss_millis = millis();
         float packetloss = float(float(badFrames * 100) / float(goodFrames + badFrames));
         snprintf(snprintf_buf, snp_max, "S.Port goodFrames:%d   badFrames:%d   frame loss:%.3f%%\n", goodFrames, badFrames, packetloss);
         Log.print(snprintf_buf);
       }      
      #endif  
  
      delay(1);            // I am important!
      
      #if (Telemetry_In == 0)   // serial UART
        while (inSerial.available()) {
      #elif (Telemetry_In == 4) // serial BT  
        while (frsSerialBT.available()) {        
      #endif
        if (b == 0x7E) {  // end of frame parse
          if (i == 3) {
            memset(&buf[2], 0x00, inMax-2); // clear the rest
          }
          //snprintf(snprintf_buf, snp_max, "0x7E found  i:%d  ", i);   PrintFrsBuffer(buf, 10);
          //Log.print(snprintf_buf);
          buf[0] = b;
          i = 1;  

          if (buf[2] == 0x10) {
            if (buf[9] == (0xFF-crcin)){  // Test CRC
              frGood = true;            
              frGood_millis = millis();  
              goodFrames++;               
              crcin = 0;
              return true;              // RETURN
            } else {
              badFrames++;              
              //Log.print(" CRC Bad!: "); 
              //PrintFrsBuffer(buf, 10);        
            }
          }
          crcin = 0;
        }  // end of b == 0x7E
         
        b = SafeRead();
        //Printbyte(b, true, ','); snprintf(snprintf_buf, snp_max, ":i[%d] ", i); 
        //Log.print(snprintf_buf);
        if (b != 0x7E) {  // if next start/stop don't put it in the buffer
          if ((i > 1) && (i < 9))  crcStepIn(b);           
        }
        buf[i] = b;              
        if (i<inMax-1) i++;          
      }
      return false;     
    }
    //===================================================================
    
    bool FPort_Read_A_Frame(uint8_t *buf, frport_t  frport_type) {
      /*
       Master arranges timing of transaction, slave responds
       Master sends downlink frame just after channel (control) frame
       We are a slave, and default to receiving status      
       Slave responds with uplink frame immediately if matching ID received
      */
      #if (Telemetry_In == 0)   // serial UART
        lth=inSerial.available();
      #elif (Telemetry_In == 4) // serial BT  
        lth=frsSerialBT.available();  
      #endif  

      if (lth < 10) {
        if (lth > 0) {
          //snprintf(snprintf_buf, snp_max, "lth=%d\n", lth); 
          //Log.print(snprintf_buf);
        }
        return false;       // prevent 'wait-on-read' blocking
      }
          
      #if defined Report_Packetloss
        uint32_t period = (Report_Packetloss * 60000);
        if (millis() - packetloss_millis > period) {
          packetloss_millis = millis();
          float packetloss = float(float(badFrames * 100) / float(goodFrames + badFrames));
          snprintf(snprintf_buf, snp_max, "F.Port goodFrames:%d   badFrames:%d   frame loss:%.3f%%\n", goodFrames, badFrames, packetloss);
        }
      #endif

      // ======================= F.Port1 ==========================
      
      if (frport_type == f_port1) {           // find start of frame 
        while (!(chr==0x7E)) {     // find first 0x7E, should be start, but could be previous stop
          chr = ReadByte();
        }
        buf[0] = chr;
        chr = ReadByte();    // could be start 0x7E or len 0x08, 0x0D, 0x18, 0x20, 0x23  
        while (chr == 0x7E) {           // if start 0x7E, then the first one was a stop so ignore it
          *buf = chr;
          chr = ReadByte();
        }
        fr_lth = *(buf+1) = chr;                  // lth
        fr_type = *(buf+2) = ReadByte();          // frame_type  
        
        if ((fr_lth == 0x08) || (fr_lth == 0x19) ) {  // downlink or control frame
          frGood = true;            
          frGood_millis = millis();  
  
          switch(fr_type){
            case 0x00:      // F.Port v1.0 Control Frame (RC Channels)
              parseGood = ParseFrame(buf, frport_type, fr_lth);
              if (parseGood) {
                #if defined Derive_PWM_Channesl           
                  pwmGood = BytesToPWM(buf+3, &pwm_ch[0], fr_lth);
                  if (pwmGood) {
                    #if defined Debug_PWM_Channels
                      Print_PWM_Channels(&pwm_ch[0], num_of_channels);
                    #endif  
                    pwmGood_millis = millis();
                  }
                #endif  
                #if defined Support_SBUS_Out 
                #endif
                return true;    
              }  
              return false;
  
            case 0x01:      // F.Port v1.0 downlink frame from master  -  match on our sensor byte ( range 0x0~0x1B or 0x1E (FC) )              
              parseGood = ParseFrame(buf, frport_type, fr_lth); 
              if (parseGood) {    
                fr_prime = buf[3];                         // if prime == 0x00, reply prime = 0x00
                                                           // if prime == 0x10, reply prime = 0x10 (slave ready)                                                       
                return true;  
              }
              return false;  
              
            case 0x0D:   // mavlite downlink frame from master, match on sensor id 0x0D                          
              parseGood = ParseFrame(buf,  frport_type, fr_lth); 
              if (parseGood) {
                fr_prime = buf[3];                         // should be 0x30                                                                                                              
                // Mavlite   do something
                return false;      
              }
              return false;
                    
            default: 
              //   snprintf(snprintf_buf, snp_max, "Unknown frame type = %X\n", fr_type);  
              return false;     
          }       // end of switch   
        } else {  // end of length ok
          badFrames++; // frame length error due to fail length test
          //snprintf(snprintf_buf, snp_max, "Bad FPort frame length = %X\n", fr_lth); 
          //Log.print(snprintf_buf);
          return false; 
        }     
      }          // end of FPort1

      // ======================= F.Port2 ==========================
      
      if (frport_type == f_port2) {                // find start of frame
        bool ctl = false;
        bool ota = false; 
        bool dlink = false;
        while ( (!(ctl)) && (!(ota)) && (!(dlink)) ) {     // find valid lth + type combo
          prev_chr = chr;
          chr = ReadByte();
          ctl = (((prev_chr == 0x0D) || (prev_chr == 0x18)|| (prev_chr == 0x23)) && (chr == 0xFF)); 
          ota = (((prev_chr == 0x0D) || (prev_chr == 0x18)|| (prev_chr == 0x23)) && (chr == 0xF1)); 
          dlink = ((prev_chr == 0x08)  && (chr == 0x1B));         
        }  
            
        *(buf) = 0;                     // not used for fp2
        fr_lth = *(buf+1) = prev_chr;   // lth 
        fr_type = *(buf+2) = chr;       // frame_type 
                                        
        if ((fr_lth == 0x08) || (fr_lth == 0x0d) || (fr_lth == 0x18) || (fr_lth == 0x20) ) {  // 
          frGood = true;            
          frGood_millis = millis();  
          //snprintf(snprintf_buf, snp_max, "fr_lth:%d   fr_type:%X\n", fr_lth, fr_type);  
          //Log.print(snprintf_buf); 
    
          switch(fr_type){
            
            case 0x0D:   // mavlite downlink frame from master, match on sensor id 0x0D            
               
              parseGood = ParseFrame(buf+1,  frport_type, fr_lth); 
              if (parseGood) {
                fr_prime = buf[3];                         // should be 0x30                                                                                                              
                return false;  // ignore mavlite for now
              }
              return false; 
                    
            case 0x1B:   // F.Port v2.3.7 downlink frame from master, match on sensor id 0x0~0x1B or 0x1E (FC)                     
              parseGood = ParseFrame(buf,  frport_type, fr_lth); // + CRC
              //printf("fr:type:%X parseGood:%d=================<\n", fr_type, parseGood);
              if (parseGood) {
                fr_prime = buf[3];                         // if prime == 0x00, reply prime = 0x00
                                                           // if prime == 0x10, reply prime = 0x10 (slave ready)                                                                                                                 
               return true; 
              }
              return false;

            case 0xff:      // F.Port v2.3.7 Control Frame (RC Channels)  
              parseGood = ParseFrame(buf,  frport_type, fr_lth+1); // + CRC
              //printf("fr:type:%X parseGood:%d\n", fr_type, parseGood);              
              if (parseGood) {
                #if defined Derive_PWM_Channesl           
                  pwmGood = BytesToPWM(buf+3, &pwm_ch[0], fr_lth);
                  if (pwmGood) {
                    #if defined Debug_PWM_Channels
                      Print_PWM_Channels(&pwm_ch[0], num_of_channels);
                    #endif  
                    pwmGood_millis = millis();
                  }
                #endif  
                #if defined Support_SBUS_Out      
                #endif
                return true;     
              }
 
              return false;

            case 0xf0:      // OTA start frame
              return false;
            case 0xf1:      // OTA data frame
              return false;
            case 0xf2:      // OTA end frame
              return false;    
            default: 
           //   snprintf(snprintf_buf, snp_max, "Unknown frame type = %X\n", fr_type);  
           //Log.print(snprintf_buf);
              break;           
          }       // end of switch   
        } else {  // end of length ok
          badFrames++; // due to fail length test
          return false;
          // snprintf(snprintf_buf, snp_max, "Bad FPort frame length = %X\n", fr_lth);  
          //Log.print(snprintf_buf);
        }     
      }          // end of FPort2

      // No start/stop 

    }

    //===================================================================   
 
    bool ParseFrame(uint8_t *buf, frport_t  frport_type, uint16_t lth) {
      //snprintf(snprintf_buf, snp_max, "buf[0]:%X  buf[1]:%X  buf[2]:%X\n", buf[0], buf[1], buf[2]);    
      //Log.print(snprintf_buf);          
      uint8_t crc_lo, crc_hi;
         
      int i = 0;
      for (i = 3 ; i < lth+2 ; i++) {        
        chr = SafeRead();          // f_port2 ignores start byte[0]
        //if (*(buf+2) == 0x1b) {
        //  Log.printf("i:%d  chr:%X\n", i, chr);
        //}
        *(buf+i) = chr;
      }
      
       chr = SafeRead();           // this is the crc byte
       *(buf+i) = chr;      
       
      #if (defined Debug_FPort_Buffer) 
        PrintFrsBuffer(buf, lth+4);
      #endif 

      bool mycrcGood =  0; 
      if (frport_type == f_port1) {
        mycrcGood =  crcGood(buf+1, lth+1); // CRC range set here, include len
      } else
      if (frport_type == f_port2) {
        mycrcGood =  crcGood(buf+2, lth);  // CRC range set here, exclude len
      //    Log.printf("mycrcGood:%d\n", mycrcGood);      
      }
      
      if (mycrcGood) {
        goodFrames++;
      } else {
        badFrames++; // due to crc
      }
      #if defined Debug_CRC
        snprintf(snprintf_buf, snp_max, "mycrcGood=%d\n\n", mycrcGood);    
        Log.print(snprintf_buf);            
      #endif  
      return mycrcGood;   
   
    }     
 
    //===================================================================

    byte ReadByte() {
    byte b;

      if (lth == 0) {
        while (lth==0) {
          CheckStatusAndTimeouts();

          #if (Telemetry_In == 0)   // serial UART
            lth=inSerial.available();
          #elif (Telemetry_In == 4) // serial BT  
            lth=frsSerialBT.available();  
          #endif  
          
        }
     //    Log.printf("\nlen=%3d\n",len); 
      } 

      // Data is available
      serGood = true;            // We have a good serial connection!
      serGood_millis = millis();
      #if (Telemetry_In == 0)   // serial UART
        b = inSerial.read();
      #elif (Telemetry_In == 4) // serial BT  
        b = frsSerialBT.read();    
      #endif     
      lth--;
      
      #if (defined Debug_FrPort_Stream)  
        Printbyte(b, true, '<');
      #endif 
      delay(0); // yield to rtos for wifi & bt to get a sniff      
      return b;
    }

    //===================================================================

    byte SafeRead() {
      byte b;  

      b = ReadByte();     
      
      //  if 0x7D is received it should be omitted, and the next byte should 
      //  be XOR or ADD with 0x20
      //  0x5D => 0x7D, 0x5E => 0x7E
    
      if (b == 0x7D) {
        b = ReadByte();
        b ^= 0x20;
      }
      #if (defined Debug_FrPort_Safe_Read)  
        Printbyte(b, true, '<');
      #endif 
      delay(0); // yield to rtos for wifi & bt to get a sniff 
      return b;
    } 
   #endif  // End of Serial   
    //===================================================================

    void crcEnd(int16_t *mycrc)  {
      *mycrc = 0xFF - *mycrc;                  // final 2s complement
      #if defined Debug_CRC
        snprintf(snprintf_buf, snp_max, "crcEnd=%3X %3d\n", *mycrc, *mycrc);    
        Log.print(snprintf_buf);         
      #endif  
    }
    //===================================================================

    void crcStepIn(uint8_t b) {
       crcin += b;          // add in new byte
       crcin += crcin >> 8;   // add in high byte overflow if any
       crcin &= 0xff;  // mask all but low byte, constrain to 8 bits 
       #if defined Debug_CRC       
         snprintf(snprintf_buf, snp_max, "AddIn %3d %2X\tcrcin_now=%3d %2X\n", b, b, crcin, crcin);    
         Log.print(snprintf_buf);           
       #endif  
    }  
    //===================================================================
    
    void crcStep(int16_t *mycrc, uint8_t b) {
       *mycrc += b;             // add in new byte
       *mycrc += *mycrc >> 8;   // add in high byte carry if any
       *mycrc &= 0xff;          // mask all but low byte, constrain to 8 bits

      #if defined Debug_CRC
         snprintf(snprintf_buf, snp_max, "CRC Step: b=%3X %3d\  crc=%3X %3d\n", *mycrc, *mycrc);    
         Log.print(snprintf_buf);           
      #endif
    }    
    //===================================================================   
       
    uint8_t crcGet(uint8_t *buf, uint8_t lth)  {

      int16_t mycrc = 0;
      for (int i = 0; i < lth; i++) {
        crcStep(&mycrc, *buf++);
      }
      crcEnd(&mycrc);
      return mycrc;
    }
    //=======================================================================  
    bool crcGood(uint8_t *buf, uint8_t lth)  {
      
      uint8_t mycrc = crcGet(buf, lth);   
      uint8_t fpcrc = *(buf+lth);
      #if defined Debug_CRC    
        snprintf(snprintf_buf, snp_max, "mycrc=%3X %3d  fpcrc=%3X\ %3d\n", mycrc, mycrc, fpcrc, fpcrc);    
        Log.print(snprintf_buf);        
      #endif
    return (mycrc == fpcrc);

   }  

    //===================================================================
    void Frs_Decode(uint8_t *buf) {
    // Do the sensor packets according to appID
        uint16_t appID = uint16Extract(buf, 1 );
        pt_payload = uint32Extract(buf, 3);
        //snprintf(snprintf_buf, snp_max, "appID:%4X  cur.alt:%.0f  hom.alt:%.0f  cur.alt_ag:%.0f\n", appID, cur.alt, hom.alt, cur.at_agc);    
        //Log.print(snprintf_buf);            
        switch(appID) {

                // One byte ID old D Style Hub/legacy protocol below 
                  case 0x01:                         // GPS Alt BP
                    if (!d_dia) {
                      d_dia=true;
                         LogScreenPrintln("D dialect"); 
                    }           
                    cur.alt = uint16Extract(buf, 3);
                    if (!(cur.alt==0.0000)) {
                      altGood=true; 
                      new_GPS_data = true;     
                    }
                    #if defined Debug_All || defined Debug_FrSky_Messages             
                      Log.print(" GPS Altitude 0x01=");
                      Log.println(cur.hdg,0);
                    #endif
                    break;
                  case 0x12:                        // Lon BP - before point
                    lonDDMM = uint32Extract(buf, 3);
                    #if defined Debug_All || defined Debug_FrSky_Messages             
                      Log.print(" lonDDMM 0x12=");
                      Log.println(lonDDMM);
                    #endif             
                    break;
                  case 0x13:                       // Lat BP
                    latDDMM = uint32Extract(buf, 3);
                    #if defined Debug_All || defined Debug_FrSky_Messages             
                      Log.print(" latDDMM 0x13=");
                      Log.println(latDDMM);
                    #endif           
                    break;
                  case 0x14:        
                    cur.hdg = uint16Extract(buf, 3);      // Course / Heading BP
                    if (!(cur.hdg==0.000)) hdgGood=true;
                    #if defined Debug_All || defined Debug_FrSky_Messages             
                      Log.print(" Heading 0x14=");
                      Log.println(cur.hdg,0);
                    #endif
                    break;               
                  case 0x1A:                      // Lon AP
                    mmmm = uint32Extract(buf, 3);
                    DD = lonDDMM/100;
                    MM = lonDDMM -(DD*100);
                    MMmmmm = MM + (mmmm/1E4);       
                    cur.lon = DD + (MMmmmm/60);
                    if (EW==0x57)  cur.lon = 0-cur.lon; //  "W", as opposed to "E"
                    lonGood=true;
                    new_GPS_data = true;  
                    #if defined Debug_All || defined Debug_FrSky_Messages             
                      Log.print(" Lon After Point 0x1A=");
                      Log.println(cur.lon,0);
                    #endif
                     
                    break;
                  case 0x1B:                      // Lat AP
                    mmmm = uint32Extract(buf, 3);
                    DD = latDDMM/100;
                    MM = latDDMM -(DD*100);
                    MMmmmm = MM + (mmmm/1E4);
                    cur.lat = DD + (MMmmmm/60);     
                    if (NS==0x53) cur.lat = 0-cur.lat;  //  "S", as opposed to "N" 
                    latGood=true;
                    new_GPS_data = true;
                    #if defined Debug_All || defined Debug_FrSky_Messages             
                      Log.print(" Lat After Point 0x1B=");
                      Log.println(cur.lat,0);
                    #endif
                    break;
                  case 0x22:                      // Lon E/W
                    EW = uint8Extract(buf, 3);
                    #if defined Debug_All || defined Debug_FrSky_Messages             
                      Log.print(" Lon E/W 0x22=");
                      Log.println(EW);
                    #endif
                    break;
                  case 0x23:                      // Lat N/S
                    NS = uint8Extract(buf, 3);  
                    #if defined Debug_All || defined Debug_FrSky_Messages             
                      Log.print(" Lon Lat N/S 0x23=");
                      Log.println(NS);
                    #endif
                    break;

                    
                // *****************************************************************
                //   Two byte ID, D mode here, X mode below    
                
                  case 0x100:              // Altitude
                    if (!x_dia) {
                      x_dia=true;
                         LogScreenPrintln("X dialect"); 
                    }
                    pt_altitude= uint32Extract(buf, 3);
                    cur.alt  = pt_altitude / 100;
                    if (!(cur.alt ==0)) {
                      altGood=true; 
                      new_GPS_data = true;
                    }

                    break; 
                    
                  case 0x400:              // Tmp1 FLIGHT_MODE
                    pt400_flight_mode = uint32Extract(buf, 3); 
                    d1234 = (uint16_t)(pt400_flight_mode * 0.1);                  
                    pt400_arm_flag =  pt400_flight_mode - (d1234 * 10);                    
                    motArmed = (pt400_arm_flag == 5);                   
                    
                  /*
                   * NOTE from observation, (LSD == 5) appears to signal armed status - zs6buj 
                    Extract from BetaFlight source code below appears to be obsolete
                    Actual flight mode, sent as 4 digits. Number is sent as (1)1234. 
                    Please ignore the leading 1, it is just there to ensure the number 
                    as always 5 digits (the 1 + 4 digits of actual data) the numbers 
                    are aditives (for example, if first digit after the leading 1 is 
                    6, it means GPS Home and Headfree are both active)
                    1 is GPS Hold, 2 is GPS Home, 4 is Headfree
                    1 is mag enabled, 2 is baro enabled, 4 is sonar enabled
                    3. 1 is angle, 2 is horizon, 4 is passthrough
                    4. 1 is ok to arm, 2 is arming is prevented, 4 is armed
                    */
                    
                    #if defined Debug_All || defined Debug_FrSky_Messages || defined Debug_FrSkyD_Flight_Mode
                      snprintf(snprintf_buf, snp_max, "FrSky 0x400 payload=%u pt400_arm_flag=%u  motArmed=%u\n", pt400_flight_mode, pt400_arm_flag, motArmed);    
                      Log.print(snprintf_buf);                         
                    #endif  
                    break;   
                                      
                  case 0x410:              // Tmp2 - iNav GPS status 
                    //* **Tmp2** : GPS lock status, accuracy, home reset trigger, and number of satellites. 
                    //  Number is sent as **ABCD** detailed below. Typical minimum GPS 3D lock value is 3906 
                    // (GPS locked and home fixed, HDOP highest accuracy, 6 satellites).
                    // * **A** : 1 = GPS fix, 2 = GPS home fix, 4 = home reset (numbers are additive)
                    // * **B** : GPS accuracy based on HDOP (0 = lowest to 9 = highest accuracy)
                    // * **C** : number of satellites locked (digit C & D are the number of locked satellites)
                    // * **D** : number of satellites locked (if 14 satellites are locked, C = 1 & D = 4)                 
                    iNav=true;
                    pt410_gps_status = uint32Extract(buf, 3);
                    // decode to digits 1 thru 4
                    d1 = (pt410_gps_status / 1000);
                    dr1 = d1 * 1000;
                    d2 = (pt410_gps_status - dr1) / 100;
                    dr2 = dr1 + (d2 * 100);
                    d3 = (pt410_gps_status - dr2) / 10;
                    dr3 = dr2 + (d3 * 10);
                    d4 = pt410_gps_status - dr3;
                    
                    // decode to sub-digits of d1
                    d14 = d1 / 4;      // say 7 / 4 = 1
                    dr12 = d1 % 4;     //     7 % 4 = 3
                    d12 = dr12 / 2;    //     3 / 2 = 1
                    d11 = dr12 % 2;    //     3 % 2 = 1

                    pt_gps_fix = d11;
                    pt_gps_homefix = d12;
                    pt_gps_homereset = d14;
                    pt_gps_accuracy = d2;   // 0 thru 9 highest accuracy
                    pt_gps_numsats = (d3*10) + d4;
              
                    gpsfixGood = (pt_gps_accuracy > 7);  // 0 thru 9 - 9 best  
                      
                    #if defined Debug_All || defined Debug_FrSky_Messages
                      Log.print("pt_gp_fix="); Serial.print(pt_gps_fix);     
                      Log.print(" pt_gps_homefix ="); Log.print(pt_gps_homefix);
                      Log.print(" pt_gp_homereset="); Log.print(pt_gps_homereset);     
                      Log.print(" pt_gps_accuracy ="); Log.print(pt_gps_accuracy);
                      Log.print(" pt_gps_numsats="); Log.println(pt_gps_numsats); 
                    #endif  
                    break;                    

                 case 0x800:                      // Latitude and Longitude
                 
                   pt_latlong= uint32Extract(buf, 3); 
                   ms2bits = pt_latlong >> 30;
                   pt_latlong = pt_latlong & 0x3fffffff; // remove ms2bits
                   #if defined Debug_All     
                     Log.print(" ms2bits=");
                     Log.println(ms2bits);
                   #endif   
                   switch(ms2bits) {
                     case 0:   // Latitude Positive
                       pt_lat = pt_latlong;     // lon always comes first   
                       cur.lat = (float)(pt_lat / 6E5);     // lon always comes first                                           
                       #if defined Debug_All || defined Debug_FrSky_Messages                
                         Log.print(" FrSky 0x800 latitude=");
                         Log.println(cur.lat,7);
                       #endif
                       latGood=true;
                       new_GPS_data = true; 
                       break;
                     case 1:   // Latitude Negative 
                       pt_lat = pt_latlong;                         
                       cur.lat = (float)(0-(pt_lat / 6E5)); 
                       #if defined Debug_All || defined Debug_FrSky_Messages           
                         Log.print(" FrSky 0x800 latitude=");
                         Log.println(cur.lat,7);  
                       #endif   

                       if (!(cur.lat==0.000000) && !(cur.lon==0.000000)){
                         latGood=true;
                         new_GPS_data = true;                        
                       }
                       break;
                     case 2:   // Longitude Positive
                       pt_lon = pt_latlong;    
                       cur.lon = (float)(pt_lon / 6E5);                                         
                       #if defined Debug_All || defined Debug_FrSky_Messages   
                         Log.print(" FrSky 0x800 longitude=");
                         Log.println(cur.lon,7); 
                       #endif                       
                       lonGood=true;
                       new_GPS_data = true;                         
                       break;
                     case 3:   // Longitude Negative
                       pt_lon = pt_latlong; 
                       cur.lon = (float)(0-(pt_lon / 6E5));                         
                       #if defined Debug_All                        
                         Log.print(" FrSky 0x800 longitude=");
                         Log.println(cur.lon,7); 
                       #endif                   
                       lonGood=true;
                       new_GPS_data = true;                       
                       break;
                    }
                    break;
                  case 0x820:              // Altitude
                    pt_altitude= uint32Extract(buf, 3);
                    cur.alt  = (float)(pt_altitude / 100);
                    if (!(cur.alt ==0.0000)){
                      altGood=true; 
                      new_GPS_data = true;
                    }
                    #if defined Debug_All || defined Debug_FrSky_Messages || defined Debug_FrSky_GPS
                       Log.print(" FrSky 0x820 altitude=");
                       Log.println(cur.alt,1); 
                     #endif    
                    
                    break;          
                  case 0x840:              // Heading
                    pt_heading= uint32Extract(buf, 3);
                    cur.hdg = (float)(pt_heading / 100);
                    if (!(cur.hdg==0.0000)) hdgGood=true;
                    #if defined Debug_All || defined Debug_FrSky_Messages   
                       Log.print(" FrSky 0x840 heading=");
                       Log.println(cur.hdg,1); 
                     #endif               
                    break;

  
                 //   Two byte X mode, Mavlink Passthrough Protocol below    
                 //===================================================================
                 case 0xF000:   
                   break;
                 case 0xF101:                        // RSSI  
                   pt_rssi = pt_payload;
                   #if defined Debug_FrSky
                     Log.print(" FrSky F101: RSSI=");
                     Log.println(pt_rssi);
                   #endif  
                   break;
                 case 0xF103:   
                   break;
                 case 0xF104:   
                   break;
                 case 0xF105:   
                   break;  
                                  
                 case 0x5002:
                  
                  // GPS Status &  gpsAlt
                    Passthru=true;

                    pt_gps = uint32Extract(buf, 3);
                    pt_numsats = bit32Extract(pt_gps, 0, 4);
                    pt_gps_status = bit32Extract(pt_gps, 4, 2) + bit32Extract(pt_gps, 14, 2);
                    pt_hdop = bit32Extract(pt_gps, 7, 7) * TenToPwr(bit32Extract(pt_gps, 6, 1));  
                    gpsAlt = bit32Extract(pt_gps,24,7) * TenToPwr(bit32Extract(pt_gps,22,2)); //-- dm                                    
                
                    neg = bit32Extract(pt_gps, 31, 1);
                    if (neg==1) cur.alt = 0 - cur.alt;
                    new_GPS_data = true;
                    
                    gpsfixGood=(pt_hdop>=3) && (pt_numsats>9);
                    
                    #if defined Debug_All || defined Debug_FrSky_Messages || defined Debug_FrSky_GPS
                      Log.print(" FrSky 0x5002 Num sats=");
                      Log.print(pt_numsats);
                      Log.print(" gpsStatus=");
                      Log.print(pt_gps_status);                
                      Log.print(" HDOP=");
                      Log.print(pt_hdop);                    
                      Log.print(" gpsAlt=");
                      Log.print((float)(gpsAlt/10), 1);
                      Log.print(" gpsfixGood=");
                      Log.print(gpsfixGood);                      
                      Log.print(" neg=");
                      Log.println(neg);   
                    #endif

                    break;
                    
                  case 0x5003:                         // Battery 1 Hz
                   pt_bat1_volts = (float)(bit32Extract(pt_payload,0,9));  // dv -> V
                   //Log.printf("mantissa:%d  10exponent:%d mutiplier:%d \n", bit32Extract(pt_payload,10,7), bit32Extract(pt_payload,9,1), TenToPwr(bit32Extract(pt_payload,9,1)) );
                   //snprintf(snprintf_buf, snp_max, "mantissa:%d  10exponent:%d mutiplier:%d \n", bit32Extract(pt_payload,10,7), bit32Extract(pt_payload,9,1), TenToPwr(bit32Extract(pt_payload,9,1)));    
                   //Log.print(snprintf_buf);
                   
                   pt_bat1_amps = (bit32Extract(pt_payload,10,7) * TenToPwr(bit32Extract(pt_payload,9,1) ) );  // rounded to nearest whole A
                   pt_bat1_mAh = bit32Extract(pt_payload,17,15);
                   pt_bat1_amps *= 10;  // prep_number() divided by 10 to get A, but we want dA for consistency
                   #if defined Debug_FrSky
                     Log.print(" FrSky 5003: Battery Volts=");
                     Log.print(pt_bat1_volts, 1);
                     Log.print("  Battery Amps=");
                     Log.print((float)pt_bat1_amps, 0);
                     Log.print("  Battery mAh=");
                     Log.println(pt_bat1_mAh); 
                   #endif       
                   break;                          
                   
                  case 0x5004:                         // Home
                    pt_home = uint32Extract(buf, 3);
                    pt_home_dist = bit32Extract(pt_home,2,10) * TenToPwr(bit32Extract(pt_home,0,2));                   
                    fHomeDist = (float)pt_home_dist * 0.1;  // Not used here 
                    pt_home_alt = bit32Extract(pt_home,14,10) * TenToPwr(bit32Extract(pt_home,12,2)); // decimetres
                    cur.alt  = (float)(pt_home_alt) / 10;
                    if (finalHomeStored) {
                      cur.alt_ag = cur.alt - hom.alt;
                    } else {
                      cur.alt_ag = 0;
                    }                   
                    if (bit32Extract(pt_home,24,1) == 1) 
                      pt_home_alt *= -1;
                    altGood=true; 
                    #if (defined Debug_All) || (defined Debug_FrSky_Messages) || (defined Debug_FrSky_Home)
                      Log.print(" FrSky 0x5004 Dist to home=");
                      Log.print(fHomeDist, 1);  
                      Log.print(" pt_home_alt=");
                      Log.print(pt_home_alt/10);  
                      Log.print(" cur.alt=");
                      Log.println(cur.alt, 0);                                           
                    #endif
                    break;
                      
                  case 0x5005:                      
                  // Vert and Horiz Velocity and Yaw angle (Heading)
                    pt_velyaw = uint32Extract(buf, 3);      
                    pt_yaw = pt_home_dist = bit32Extract(pt_velyaw, 17, 11);
                    cur.hdg = pt_yaw * 0.2F;
      
                    hdgGood=true;
                    #if defined Debug_All || defined Debug_FrSky_Messages
                      Log.print(" FrSky 0x5005 Yaw raw=");
                      Log.print(pt_yaw);
                      Log.print("   Heading=");
                      Log.println(cur.hdg,1);                     
                    #endif
                    break;   
                    
                  case 0x5006:                         // Roll, Pitch and Range - Max Hz                   
                   pt_roll = bit32Extract(pt_payload,0,11);        
                   pt_roll = (pt_roll - 900) * 0.2;             //  -- roll [0,1800] ==> [-180,180] 
                   pt_pitch = bit32Extract(pt_payload,11,10);   
                   pt_pitch = (pt_pitch - 450) * 0.2;           //  -- pitch [0,900] ==> [-90,90]
                   pt_range = bit32Extract(pt_payload,22,10) * TenToPwr(bit32Extract(pt_payload,21,1));
                   pt_froll = pt_roll * 0.001F;
                   pt_fpitch = pt_pitch * 0.001F;
                   #if defined Debug_All || defined Debug_FrSky_Messages
                     Log.print("Frsky 5006: Range=");
                     Log.print(pt_range,2);
                     Log.print(" Roll=");
                     Log.print(pt_roll);
                     Log.print("deg   Pitch=");
                     Log.print(pt_pitch);   
                     Log.println("deg");               
                   #endif
                   break;
                  default:                                  
                    #if defined Debug_All || defined Debug_FrSky_Messages
                      Log.print("Frsky un-handeled appID=0x");
                      Log.println(appID, HEX);             
                    #endif
                    break;                   
        }

        gpsGood = hbGood = gpsfixGood & lonGood & latGood & altGood;    

        if (gpsGood) gpsGood_millis = millis();     // Time of last good GPS packet 
        hbGood_millis= millis();                    // good GPS data is equivalent to a mavlink heartbeat
        
        #if defined Debug_Frsky_GPS_Status
          snprintf(snprintf_buf, snp_max, "gpsGood:%u  gpsfixGood:%u  lonGood:%u  latGood:%u  altGood:%u  hdgGood:%u  boxhdgGood:%u \n", gpsGood, gpsfixGood, lonGood, latGood, altGood, hdgGood, boxhdgGood);    
          Log.print(snprintf_buf);         
        #endif
        
         
    }

    //========================================================
    uint32_t TenToPwr(uint8_t pwr) {
      uint32_t ttp = 1;
      for (int i = 1 ; i<=pwr ; i++) {
        ttp*=10;
      }
      return ttp;
    }  
    //========================================================
    uint32_t bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth) {
      uint32_t r = (dword & createMask(displ,(displ+lth-1))) >> displ;
      //  Log.print(" Result=");
      // Log.println(r);
      return r;
    }
    uint32_t createMask(uint8_t lo, uint8_t hi) {
      uint32_t r = 0;
      for (unsigned i=lo; i<=hi; i++) r |= 1 << i;
        //  Log.print(" Mask 0x=");
        //  Log.println(r, HEX);      
      return r;
     }  
    //========================================================

    uint32_t uint32Extract(uint8_t *buf, int posn){
  
      //  The number starts at byte "posn" of the received packet and is four bytes long.
      //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
      byte b1 = buf[posn+3];
      byte b2 = buf[posn+2];
      byte b3 = buf[posn+1];
      byte b4 = buf[posn]; 
   
      unsigned long highWord = b1 << 8 | b2;
      unsigned long lowWord  = b3 << 8 | b4;
    
        // Now combine the four bytes into an unsigned 32bit integer

      uint32_t myvar = highWord << 16 | lowWord;
      return myvar;
    }
    //========================================================
    int32_t int32Extract (uint8_t *buf, int posn){
  
      //  The number starts at byte "posn" of the received packet and is four bytes long.
      //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
      byte b1 = buf[posn+3];
      byte b2 = buf[posn+2];
      byte b3 = buf[posn+1];
      byte b4 = buf[posn]; 
   
      unsigned long highWord = b1 << 8 | b2;
      unsigned long lowWord  = b3 << 8 | b4;
   
      // Now combine the four bytes into an unsigned 32bit integer
 
      int32_t myvar = highWord << 16 | lowWord;
      return myvar;
    }
    //========================================================
    uint16_t uint16Extract(uint8_t *buf, int posn){
  
      //  The number starts at byte "posn" of the received packet and is two bytes long
      //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap

       byte b1 = buf[posn+1];
       byte b2 = buf[posn];  
    
        // Now convert the 2 bytes into an unsigned 16bit integer
    
        uint16_t myvar = b1 << 8 | b2;
        return myvar;
    }
    //========================================================
    int16_t int16Extract(uint8_t *buf, int posn){
  
      //  The number starts at byte "posn" of the received packet and is two bytes long
      //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
      byte b1 = buf[posn+1];
      byte b2 = buf[posn];
    
      // Now convert the 2 bytes into a signed 16bit integer
    
       int16_t myvar = b1 << 8 | b2;
        return myvar;
    }
    //========================================================
    uint8_t uint8Extract(uint8_t *buf, int posn){
  
      //  The number starts at byte "posn" of the received packet and is one byte long

      byte b1 = buf[posn];
    
      // Now convert the byte into an unsigned 8 bit integer
    
      uint8_t myvar = b1;
      return myvar;
    }
    //========================================================
    int8_t int8Extract(uint8_t *buf, int posn){
  
      //  The number starts at byte "posn" of the received packet and is one byte long

      byte b1 = buf[posn];
    
      // Now convert the byte into an unsigned 8 bit integer
    
      int8_t myvar = b1;
      return myvar;
    }
    //========================================================
   
#endif
