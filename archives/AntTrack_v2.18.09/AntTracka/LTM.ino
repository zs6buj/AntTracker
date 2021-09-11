#if (Telemetry_In == 0)    //  Serial

int16_t gpsNumSats = 0;

//  LTM variables
int16_t   iSpd = 0;
int16_t   iSat = 0;
int16_t   iFix = 0;
int16_t   iPitch = 0;
int16_t   iRoll = 0;
int16_t   iBat = 0;
int16_t   iRssi = 0;
int16_t   iAirspeed = 0;
int16_t   iFltMode = 0;
float fVBat = 0;
float fCur = 0;

bool bArmed = false;
bool bFailsafe = false;

void LTM_Receive()  {
  
TryAgain:
  chr = NextChr();
  while (!(chr==0x24)) {   // Read byte stream until you find a candidate LTM packet start character 
    chr = NextChr();
  }
  
  // Candidate found! However, it might also be payload data
  inBuf[0] = chr;           // packet start char
  chr = NextChr();                // start2 should be 0x54 
  if (!chr==0x54) goto TryAgain;   // otherwise reject the start signals
  inBuf[1] = chr;
  
  chr = NextChr();    // Packet type
  inBuf[2] = chr;
  switch (chr) {
  case 0x41:
    pLth=10;
    UnpackAttitude(pLth);
    break;
  case 0x47:
    pLth=18;
    gpsGood = UnpackGPS(pLth);
    break;
  case 0x53:
    pLth=11;
    UnpackSensors(pLth);
    break;
  default:
    goto TryAgain;    // Don't know this packet type
  }

 // PrintMavBuffer(pLth); 
  
  
}
//***************************************************

byte NextChr() {
byte x;
  iLth=Serial1.available();     //   wait for more data
  while (iLth==0) {
 //   CheckForTimeouts();
    iLth=Serial1.available();
  }  
  // Data is available
  hbGood = 1;                     // We have a good serial connection!
  x = Serial1.read();
// Printbyte(x, false, ' ');  
  return x;
}
//***************************************************
void ParsePacket(int lth){
   for (int i = 3; i < lth; i++) {   // Read the payload into the packet buffer 
    inBuf[i] = NextChr();
  } 
}
//***************************************************
boolean UnpackAttitude(int lth) {
  ParsePacket(lth);
  int16_t jPitch = int16Extract(inBuf, 3);             // degrees
  int16_t jRoll = int16Extract(inBuf, 5);              // degrees 
  int16_t jHdg = int16Extract(inBuf, 7);  
  uint8_t Crc = uint8Extract(inBuf, 9); 
  uint8_t LTCrc=LTChecksum(lth);
  
  if ((!Crc==LTCrc) || (Crc==0)) return false;  // Drop empty or CRC failed packets

  iPitch = jPitch;
  iRoll = jRoll;
  if (jHdg<0) jHdg+=360;
  cur.hdg = jHdg;
  hdgGood=1;
  #if defined Debug_All || defined Debug_LTM
    PrintMavBuffer(lth); 
    Log.print(" Pitch = ");
    Log.print(iPitch);
    Log.print(" Roll = ");
    Log.print(iRoll);
    Log.print(" Heading = ");
    Log.print(cur.hdg,0); 
    Log.print(" CRC = ");
    Log.print(Crc); 
    Log.print(" Calc CRC = ");
    Log.println(LTCrc); 
  #endif

  return true;
}
//***************************************************
boolean UnpackGPS(int lth) {
  ParsePacket(lth);
  int32_t Lat = int32Extract(inBuf, 3);              // degrees * 1E7
  int32_t Lon = int32Extract(inBuf, 7);              // degrees * 1E7
  uint8_t Spd = uint8Extract(inBuf, 11);  
  int32_t Alt = int32Extract(inBuf, 12);             // centimetres
  uint8_t Sat = uint8Extract(inBuf, 16); 
  int Fix = (Sat & 0b00000011);     // or 0x03  mask the first 6 bits
  Sat = (Sat & 0b11111100) >> 2;    // mask the last 2 bits and shift right 2 
  uint8_t Crc = uint8Extract(inBuf, 17); 
  uint8_t LTCrc=LTChecksum(lth);
  if ((!Crc==LTCrc) || (Crc==0)) return false;  // Drop empty or CRC failed packets
  
  cur.lat = Lat / 1E7;
  cur.lon = Lon / 1E7;
  cur.alt = Alt / 1E2;
  
  iSpd = Spd;
  iSat = Sat;
  iFix = Fix;
  
  if ((!(Lat == 0)) && (!(Lon == 0))) {
    gpsGood=1;
    new_GPS_data = true;
  }

  
  gpsGood_millis = millis();                 // Time of last good GPS packet

  #if defined Debug_All || defined Debug_LTM
    PrintMavBuffer(lth); 
    Log.print(" Lat = ");
    Log.print(cur.lat,7);
    Log.print(" Lon = ");
    Log.print(cur.lon,7);
    Log.print(" Speed = ");
    Log.print(iSpd); 
    Log.print(" Alt = ");
    Log.print(cur.alt,0); 
    Log.print(" Sats = ");
    Log.print(iSat); 
    Log.print(" Fix type = ");
    Log.print(iFix); 
    Log.print(" CRC = ");
    Log.print(Crc); 
    Log.print(" Calc CRC = ");
    Log.print(LTCrc); 
    Log.print(" gpsGood = ");
    Log.println(gpsGood); 
  #endif
  return gpsGood;
}
//***************************************************
boolean UnpackSensors(int lth) {
 ParsePacket(lth);
  int16_t jVBat = int16Extract(inBuf, 3);        // mV     
  int16_t jCur = int16Extract(inBuf, 5);         // mA   
  int8_t jRssi = int8Extract(inBuf, 7);  
  uint8_t jAirspeed = uint8Extract(inBuf, 8); 
  uint8_t byt = uint8Extract(inBuf, 9);   //// last 6 bits: flight mode, 2nd bit: failsafe, 1st bit: Arm status

  iFltMode =  (byt & 0b11111100) >> 2;
  bFailsafe = (byt & 0b00000010);
  bArmed =    (byt & 0b00000001);

  uint8_t Crc = uint8Extract(inBuf, 10); 
  uint8_t LTCrc=LTChecksum(lth);
  
  if ((!Crc==LTCrc) || (Crc==0)) return false;  // Drop empty or CRC failed packets

  iRssi = jRssi;
  iAirspeed = jAirspeed;
  fVBat = jVBat / 1E3;
  fCur = jCur / 1E3;
  #if defined Debug_All || defined Debug_LTM
    PrintMavBuffer(lth); 
    Log.print(" Bat Volts = ");
    Log.print(fVBat,1);
    Log.print(" Current = ");
    Log.print(fCur, 1);
    Log.print(" RSSI = ");
    Log.print(iRssi); 
    Log.print(" Airspeed = ");
    Log.print(iAirspeed);
    Log.print(" Flight Mode = ");
    Log.print(iFltMode); 
    Log.print(" ");
    Log.print(FlightMode(iFltMode));
    Log.print(" ");
    Log.print(" CRC = ");
    Log.print(Crc); 
    Log.print(" Calc CRC = ");
    Log.println(LTCrc); 
  #endif
  return true;
}
//***************************************************
uint8_t LTChecksum(int lth) {
  uint8_t LTCrc = 0x00;                      // calculate the checksum
  for (int i = 3; i < lth-1; i++) {          // exclude the crc itself
    LTCrc ^= inBuf[i];
    }
  return LTCrc;  
}
//***************************************************
String FlightMode(int FM) {
    switch (FM) {
  case 0:
    return "Manual";
    break;  
  case 1:
    return "Rate";
    break;
  case 2:
    return "Attitude/Angle";
    break;
  case 3:
      return "Horizon";
    break;
  case 4:
    return "Acro";
    break;
  case 5:
    return "Stabilized 1";
    break;
  case 6:
    return "Stabilized 2";
    break;
  case 7:
    return "Stabilized 3";
    break;
  case 8:
    return "Altitude Hold";
    break;
  case 9:
    return "Loiter/GPS Hold";
    break;
  case 10:
    return "Auto/Waypoints";
    break;
  case 11:
    return "Heading Hold/Head Free";
    break;
  case 12:
    return "Circle";
    break;
  case 13:
    return "RTH";
    break;
  case 14:
    return "Follow Me";
    break;
  case 15:
    return "Land";
    break;
  case 16:
    return "FlybyWire A";
    break;   
   case 17:
    return "FlybyWire B";
    break;
  case 18:
    return "Cruise";
    break;  
  case 19:
    return "Unknown";
    break;               
  default:
    return "Unknown Code";
    }
}
#endif
