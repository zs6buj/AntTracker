
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
  packetBuffer[0] = chr;           // packet start char
  chr = NextChr();                // start2 should be 0x54 
  if (!chr==0x54) goto TryAgain;   // otherwise reject the start signals
  packetBuffer[1] = chr;
  
  chr = NextChr();    // Packet type
  packetBuffer[2] = chr;
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

 // DisplayTheBuffer(pLth); 
  
  
}
//***************************************************

byte NextChr() {
byte x;
  iLth=Serial1.available();     //   wait for more data
  while (iLth==0) {
    CheckForTimeouts();
    iLth=Serial1.available();
  }  
  // Data is available
  telGood = 1;                     // We have a good serial connection!
  x = Serial1.read();
//  DisplayByte(x);
  return x;
}
//***************************************************
void ParsePacket(int lth){
   for (int i = 3; i < lth; i++) {   // Read the payload into the packet buffer 
    packetBuffer[i] = NextChr();
  } 
}
//***************************************************
boolean UnpackAttitude(int lth) {
  ParsePacket(lth);
  int16_t jPitch = Unpack_int16(3);             // degrees
  int16_t jRoll = Unpack_int16(5);              // degrees 
  int16_t jHdg = Unpack_int16(7);  
  uint8_t Crc = Unpack_uint8(9); 
  uint8_t LTCrc=LTChecksum(lth);
  
  if ((!Crc==LTCrc) || (Crc==0)) return false;  // Drop empty or CRC failed packets

  iPitch = jPitch;
  iRoll = jRoll;
  if (jHdg<0) jHdg+=360;
  cur.hdg = jHdg;
  hdgGood=1;
  #if defined Debug_All || defined Debug_LTM
    DisplayTheBuffer(lth); 
    Debug.print(" Pitch = ");
    Debug.print(iPitch);
    Debug.print(" Roll = ");
    Debug.print(iRoll);
    Debug.print(" Heading = ");
    Debug.print(cur.hdg,0); 
    Debug.print(" CRC = ");
    Debug.print(Crc); 
    Debug.print(" Calc CRC = ");
    Debug.println(LTCrc); 
  #endif

  return true;
}
//***************************************************
boolean UnpackGPS(int lth) {
  ParsePacket(lth);
  int32_t Lat = Unpack_int32(3);              // degrees * 1E7
  int32_t Lon = Unpack_int32(7);              // degrees * 1E7
  uint8_t Spd = Unpack_uint8(11);  
  int32_t Alt = Unpack_int32(12);             // centimetres
  uint8_t Sat = Unpack_uint8(16); 
  int Fix = (Sat & 0b00000011);     // or 0x03  mask the first 6 bits
  Sat = (Sat & 0b11111100) >> 2;    // mask the last 2 bits and shift right 2 
  uint8_t Crc = Unpack_uint8(17); 
  uint8_t LTCrc=LTChecksum(lth);
  if ((!Crc==LTCrc) || (Crc==0)) return false;  // Drop empty or CRC failed packets
  
  cur.lat = Lat / 1E7;
  cur.lon = Lon / 1E7;
  cur.alt = Alt / 1E2;

  if (Heading_Source==1 && (!homSaved)) AutoStoreHome();  // Only need this when Heading_Source is GPS
  
  iSpd = Spd;
  iSat = Sat;
  iFix = Fix;
  if ((!(Lat == 0)) || (!(Lon == 0))) {
    gpsGood=1;
    new_GPS_data = true;
  }
  gps_millis = millis();                 // Time of last good GPS packet

  #if defined Debug_All || defined Debug_LTM
    DisplayTheBuffer(lth); 
    Debug.print(" Lat = ");
    Debug.print(cur.lat,7);
    Debug.print(" Lon = ");
    Debug.print(cur.lon,7);
    Debug.print(" Speed = ");
    Debug.print(iSpd); 
    Debug.print(" Alt = ");
    Debug.print(cur.alt,0); 
    Debug.print(" Sats = ");
    Debug.print(iSat); 
    Debug.print(" Fix type = ");
    Debug.print(iFix); 
    Debug.print(" CRC = ");
    Debug.print(Crc); 
    Debug.print(" Calc CRC = ");
    Debug.print(LTCrc); 
    Debug.print(" gpsGood = ");
    Debug.println(gpsGood); 
  #endif
  return gpsGood;
}
//***************************************************
boolean UnpackSensors(int lth) {
 ParsePacket(lth);
  int16_t jVBat = Unpack_int16(3);        // mV     
  int16_t jCur = Unpack_int16(5);         // mA   
  int8_t jRssi = Unpack_int8(7);  
  uint8_t jAirspeed = Unpack_uint8(8); 
  uint8_t byt = Unpack_uint8(9);   //// last 6 bits: flight mode, 2nd bit: failsafe, 1st bit: Arm status

  iFltMode =  (byt & 0b11111100) >> 2;
  bFailsafe = (byt & 0b00000010);
  bArmed =    (byt & 0b00000001);

  uint8_t Crc = Unpack_uint8(10); 
  uint8_t LTCrc=LTChecksum(lth);
  
  if ((!Crc==LTCrc) || (Crc==0)) return false;  // Drop empty or CRC failed packets

  iRssi = jRssi;
  iAirspeed = jAirspeed;
  fVBat = jVBat / 1E3;
  fCur = jCur / 1E3;
  #if defined Debug_All || defined Debug_LTM
    DisplayTheBuffer(lth); 
    Debug.print(" Bat Volts = ");
    Debug.print(fVBat,1);
    Debug.print(" Current = ");
    Debug.print(fCur, 1);
    Debug.print(" RSSI = ");
    Debug.print(iRssi); 
    Debug.print(" Airspeed = ");
    Debug.print(iAirspeed);
    Debug.print(" Flight Mode = ");
    Debug.print(iFltMode); 
    Debug.print(" ");
    Debug.print(FlightMode(iFltMode));
    Debug.print(" ");
    Debug.print(" CRC = ");
    Debug.print(Crc); 
    Debug.print(" Calc CRC = ");
    Debug.println(LTCrc); 
  #endif
  return true;
}
//***************************************************
uint8_t LTChecksum(int lth) {
  uint8_t LTCrc = 0x00;                      // calculate the checksum
  for (int i = 3; i < lth-1; i++) {          // exclude the crc itself
    LTCrc ^= packetBuffer[i];
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
