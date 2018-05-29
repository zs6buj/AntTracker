long GetAzEl(float lat1, float lon1, float alt1, float lat2, float lon2, float alt2){
/*
 * 
 *  Aviation Formulary V1.33, by Ed Williams
 *  http://williams.best.vwh.net/avform.htm   
 *
 *  The algorithm it gives for azimuth bearing between two points is 
 *  this:
 *
 *   tc1=mod(atan2(sin(lon2-lon1)*cos(lat2),
 *          cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1)),
 *          2*pi)
 */
  
  float a, tc1, R, c, d, dLat, dLon;
/*
    Serial.print("lat1 = ");
    Serial.print(lat1,7);
    Serial.print(" lon1 = ");
    Serial.print(lon1,7);
    Serial.print(" alt1 = ");
    Serial.print(alt1,0);
    Serial.print(" lat2 = ");
    Serial.print(lat2,7); 
    Serial.print(" lon2 = ");
    Serial.print(lon2,7); 
    Serial.print(" alt2 = ");
    Serial.print(alt2,0); 
    Serial.println();
*/
  
  lon1=lon1/180*PI;
  lat1=lat1/180*PI;
  lon2=lon2/180*PI;
  lat2=lat2/180*PI;

  //Calculate azimuth
  a=atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1));
  Azimuth=a*180/PI;
  if (Azimuth<0) Azimuth=360+Azimuth;
 
  // Calculate the distance from home to craft
  dLat = (lat2-lat1);
  dLon = (lon2-lon1);
  a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
  c = 2* asin(sqrt(a));  
  d = 6371000 * c;    // Radius of the Earth is 6371km
  Distance = d;

  // Calculate elevation
  long altR = alt2-alt1;
  Elevation=atan(altR/d);
  Elevation=Elevation*360/(2*PI);
  if (Distance >= MinDisplacement) {  // Otherwise calculations are unreliable
    
    Serial.print("  Azimuth= "); Serial.print(Azimuth); 
    Serial.print("  Elevation= "); Serial.print(Elevation);
    Serial.print("  Distance= "); Serial.print(Distance);
    float elapsed = millis();
    elapsed /=1000;
    Serial.print("  Elapsed= "); Serial.print(elapsed, 3);
    Serial.println();
   
  }
  return Azimuth, Elevation, Distance ;
}
