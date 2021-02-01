  // 9600 NMEA
#if (Telemetry_In == 0)    //  Serial    
#include <TroykaGPS.h>

#define GPS_SERIAL    Serial1

GPS gps(GPS_SERIAL);

struct compdate {
 uint16_t yyyy;  
 uint8_t mm, dd, h, m, s;
};

struct compdate dt = {
  0,0,0,0,0,0};




// **********************************************************
void set_up_GPS() {
  #if defined Debug_All || defined Debug_GPS
     Debug.println("set_up_GPS");
  #endif  

 GPS_SERIAL.begin(9600);
}
// **********************************************************
void GPS_Receive() {

  //CheckForTimeouts();

  if (gps.available()) {
    hbGood = true;
    gps.readParsing();

    switch(gps.getState()) {

      case GPS_OK:
        cur.lat = gps.getLatitudeBase10();
        cur.lon = gps.getLongitudeBase10();
        cur.alt = gps.getAltitude();
        gpsGood = (gps.getSat() >= 9);
        new_GPS_data = true;
        millisGPS = millis();     // Time of last good GPS packet

        dt.yyyy = gps.getYear() - 1900;
        dt.mm = gps.getMonth();
        dt.dd = gps.getDay();
        dt.h = gps.getHour();
        dt.m = gps.getMinute();
        dt.s = gps.getSecond();

        if(GetEpoch(dt)) {
          timeGood = true;
          }
          
        LostPowerCheckAndRestore(GetEpoch(dt));  // Within 5 minutes, then restore from EEPROM

       if (Heading_Source==1 && (gpsGood) && (!homSaved)) AutoStoreHome();  // Only need this when Heading_Source is GPS 
        
        #if defined Debug_All || defined Debug_GPS
          Debug.print("gpsGood: "); Debug.print(gpsGood);
          Debug.print("  Lat: "); Debug.print(cur.lat, 6);
          Debug.print("  ");
          Debug.print("Lon: "); Debug.print(cur.lon, 6);
          Debug.print("  Alt: "); Debug.print(cur.alt);
          Debug.print("  Sats: "); Debug.print(gps.getSat());
          Debug.print("  State: ");Debug.print(gps.getState());     // GPS_OK=(State>0)
          Debug.print("  Speed: ");Debug.print(gps.getSpeedKm());  

          Debug.print("yyyy:");  Serial.print(dt.yyyy);
          Debug.print("  mm:");  Serial.print(dt.mm);        
          Debug.print("  dd:");  Serial.print(dt.dd);
          Debug.print("  h:");  Serial.print(dt.h);
          Debug.print("  m:");  Serial.print(dt.m);        
          Debug.print("  s:");  Serial.println(dt.s);

          Serial.print("UTC Time:");  Debug.println(TimeString(GetEpoch(dt)));

        #endif  
        break;
 
      case GPS_ERROR_DATA:
        Debug.println("GPS error data");
        break;

      case GPS_ERROR_SAT:
        Debug.println("GPS - no satellites yet");
        break;
    }
 }
}

 //***************************************************
uint32_t GetEpoch(struct compdate &dt) 
{
  
uint32_t  epoch;

 epoch = dt.yyyy * (365 * 86400L);  // 24 * 60 * 60 = 86400
    
 for (int i= 1 ; i<=dt.mm ; i++) {
   epoch += (DaysInMonth(dt.mm, dt.yyyy) * 86400L);
 }
        
 epoch += (dt.dd * 86400L);
 epoch += (dt.h * 3600);
 epoch += (dt.m * 60);
 epoch += dt.s;
 return epoch;
 }  

//***************************************************
uint8_t DaysInMonth(uint8_t mth, uint8_t yr ) {
uint8_t days;  
  if (mth == 4 || mth == 6 || mth == 9 || mth == 11)
    days = 30;
    
  else if (mth == 02)
  {
    bool leapyear = (yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0);

    if (leapyear == 0)
        days = 28;
    else 
        days = 29;
  }

  else 
  days = 31;

  return days;   
 }
#endif 
