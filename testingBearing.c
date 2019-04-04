#include <math.h>
#include <stdio.h>


float to_degrees(float radians) {
    float deg = radians * (180.0 / M_PI);
    if (deg < 0){
      deg += 360;
    }
    return 360-deg;
}
/*
  comfortably stolen from https://www.movable-type.co.uk/scripts/latlong.html
*/
float getBearing(float lat1, float long1, float lat2, float long2){
  float y = sin(long2-long1) * cos(lat2);
  float x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(long2-long1);
  float bearing = to_degrees(atan2(y, x));
  return bearing;
}


int main(){
  float startLat = 40.4428;
  float startLong = -79.9385;

  float northPointLat = 40.44305;
  float northPointLong = -79.9385;

  float eastPointLat = 40.44293;
  float eastPointLong = -79.93834;

  float startToNorthBearing = getBearing(startLat,startLong,northPointLat,northPointLong);
  printf("startToNorthBearing: %.6f\n", startToNorthBearing);

  float startToEastBearing = getBearing(startLat,startLong,eastPointLat,eastPointLong);
  printf("startToEastBearing:%.6f\n", startToEastBearing);

  float eastToNorthBearing = getBearing(eastPointLat,eastPointLong,northPointLat,northPointLong);
  printf("eastToNorthBearing: %.6f\n", eastToNorthBearing);
  return 1;
}
