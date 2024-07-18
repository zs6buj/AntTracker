void getAzEl(const struct Location &hom, const struct Location &cur){
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
  
  float a, c, d, dLat, dLon;
  
  #if defined DEBUG_ALL || defined DEBUG_AZEL
    log.print("hom.lat = "); log.print(hom.lat,7);
    log.print(" hom.lon = "); log.print(hom.lon,7);
    log.print(" hom.alt = "); log.print(hom.alt,0);
    log.print(" cur.lat = "); log.print(cur.lat,7); 
    log.print(" cur.lon = "); log.print(cur.lon,7); 
    log.print(" cur.alt = "); log.print(cur.alt,0); 
    log.print(" cur.alt_ag = "); log.println(cur.alt_ag,0);    
 #endif
  
  float lon1 = hom.lon / 180 * PI;  // Degrees to radians
  float lat1 = hom.lat / 180 * PI;
  float lon2 = cur.lon / 180 * PI;
  float lat2 = cur.lat / 180 * PI;

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
  int16_t altR = (int)cur.alt_ag;      //cur.alt - hom.alt;  // Relative alt or delta between tracker box and craft

  if (headingsource != 4) altR = altR * LimitCloseToHomeError(d, altR);
  
  hc_vector.el=atan(altR/d);
  hc_vector.el=hc_vector.el*360/(2*PI);     // Radians to degrees
  
  #if defined DEBUG_ALL || defined DEBUG_AZEL

  if ( (hc_vector.dist >= minDist) || ((int)cur.alt_ag >= minAltAg) ) { // Craft must be near otherwise calculations are unreliable
    float elapsed = millis() - millisStartup;
    elapsed /=1000;
    log.print("  hc_vector.az= "); log.print(hc_vector.az);             // just debug printing here
    log.print("  hc_vector.el= "); log.print(hc_vector.el);
    log.print("  hc_vector.dist= "); log.print(hc_vector.dist);
    log.print("  Elapsed= "); log.println(elapsed, 3);
  }
  else {
     log.printf(" (hc_vector.dist=%u < minDist=%u) AND (cur.alt_ag=%u < minAlt%u)\n", hc_vector.dist, minDist, (int)cur.alt_ag, minAltAg); 
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
