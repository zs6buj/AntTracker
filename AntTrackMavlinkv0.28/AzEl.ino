void GetAzEl(float lat1, float lon1, float alt1, float lat2, float lon2, float alt2){
/*
 * 
 *  Aviation Formulary V1.33, by Ed Williams
 *  http://williams.best.vwh.net/avform.htm   
 *
 *  The algorithm it gives for hc_vector.az bearing between two points is 
 *  this:
 *
 *   tc1=mod(atan2(sin(lon2-lon1)*cos(lat2),
 *          cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1)),
 *          2*pi)
 */
  
  float a, tc1, R, c, d, dLat, dLon;
  
  #if defined Debug_All || defined Debug_AzEl
    Debug.print("lat1 = "); Debug.print(lat1,7);
    Debug.print(" lon1 = "); Debug.print(lon1,7);
    Debug.print(" alt1 = "); Debug.print(alt1,0);
    Debug.print(" lat2 = "); Debug.print(lat2,7); 
    Debug.print(" lon2 = "); Debug.print(lon2,7); 
    Debug.print(" alt2 = "); Debug.println(alt2,0); 
 #endif
  
  lon1=lon1/180*PI;  // Degrees to radians
  lat1=lat1/180*PI;
  lon2=lon2/180*PI;
  lat2=lat2/180*PI;

  //Calculate azimuth
  a=atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1));
  hc_vector.az=a*180/PI;
  if (hc_vector.az<0) hc_vector.az=360+hc_vector.az;
 
  // Calculate the distance from home to craft
  dLat = (lat2-lat1);
  dLon = (lon2-lon1);
  a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
  c = 2* asin(sqrt(a));  
  d = 6371000 * c;    // Radius of the Earth is 6371km
  hc_vector.dist = d;

  // Calculate elevation
  int16_t altR = alt2-alt1;  // Relative alt

  altR = altR * LimitCloseToHomeError(d, altR);
  
  hc_vector.el=atan(altR/d);
  hc_vector.el=hc_vector.el*360/(2*PI);     // Radians to degrees
  
  #if defined Debug_All || defined Debug_AzEl
  if (hc_vector.dist >= minDist) {  // Otherwise calculations are unreliable
    float elapsed = millis() - startup_millis;
    elapsed /=1000;
    Debug.print("  hc_vector.az= "); Debug.print(hc_vector.az); 
    Debug.print("  hc_vector.el= "); Debug.print(hc_vector.el);
    Debug.print("  hc_vector.dist= "); Debug.print(hc_vector.dist);
    Debug.print("  Elapsed= "); Debug.println(elapsed, 3);
  }
  else {
     Debug.print(" hc_vector.dist = "); Debug.print(hc_vector.dist); 
     Debug.print("  < minDist = "); Debug.println(minDist);
  }
  #endif
  
  return;
}
//********************************************************
//  Limit close-to-home elevation error due to poor vertical GPS accuracy 
float LimitCloseToHomeError(float dist, int16_t alt) {  
  
float h_norm = 10.0;     // Normal at 10m dist
float v_norm = 5.0;      // Normal at 5m alt
float h_ratio, v_ratio, t_ratio;  

  h_ratio = pow((dist / h_norm), 2);  
  v_ratio = pow((float)(alt / v_norm), 2); 
  t_ratio = h_ratio * v_ratio;
  if (t_ratio > 1.0) 
    t_ratio = 1;
  return t_ratio;  
}

