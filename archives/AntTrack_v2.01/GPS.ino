  // 9600 NMEA
     
#include <TroykaGPS.h>

#define Debug         Serial
#define GPS_SERIAL    Serial1
GPS gps(GPS_SERIAL);

// **********************************************************
void set_up_GPS() {
  #if defined Debug_All || defined Debug_GPS
     Debug.println("set_up_GPS");
  #endif  

 GPS_SERIAL.begin(9600);
}
// **********************************************************
void GPS_Receive() {

  CheckForTimeouts();

  if (gps.available()) {
    telGood = true;
    gps.readParsing();

    switch(gps.getState()) {

      case GPS_OK:
        cur.lat = gps.getLatitudeBase10();
        cur.lon = gps.getLongitudeBase10();
        cur.alt = gps.getAltitude();
        gpsGood = (gps.getSat() >= 9);
        new_GPS_data = true;
        gpsMillis = millis();     // Time of last good GPS packet

        if (Heading_Source==1 && (!homStored)) AutoStoreHome();  // Only need this when Heading_Source is GPS
                  
        #if defined Debug_All || defined Debug_GPS
          Debug.print("gpsGood: "); Debug.print(gpsGood);
          Debug.print("  Lat: "); Debug.print(cur.lat, 6);
          Debug.print("  ");
          Debug.print("Lon: "); Debug.print(cur.lon, 6);
          Debug.print("  Alt: "); Debug.print(cur.alt);
          Debug.print("  Sats: "); Debug.print(gps.getSat());
          Debug.print("  State: ");Debug.print(gps.getState());     // GPS_OK=(State>0)
          Debug.print("  Speed: ");Debug.print(gps.getSpeedKm());  
                  
          Debug.println();
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
