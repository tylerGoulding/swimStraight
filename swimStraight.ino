#include <Adafruit_GPS.h>
#include "IntervalTimer.h"
#include "math.h"

#define HWSERIAL Serial1

#define MARGIN_RANGE 10 //todo
#define RANGE_RING_1 100
#define RANGE_RING_2 50
#define RANGE_RING_3 25
#define DELTA_DEGREE 14                       //todo

#define RIGHT_LED 11
#define LEFT_LED 12
#define ONBOARD_LED 13


/* iterate by 2s. (lat,long),(lat,long)
 * e.g. the first point in the path is (currentPath[0],currentPath[1])
 *
 * path can be viewed here: https://onthegomap.com/?m=r&u=mi&d=284&f=fb7d14bf6a&n=1&dm=1&context=share&r2=kpus7pg_dFCZ7s4Vy3q5
 * this is just to avoid using a 2d array
 *
 * TODO: move to seperate file and set as extern in this file.
*/


// hexagon around
//const float current                                                                                                                                                                                                                                                           Path[] ={40.44306,-79.93847,40.44292,-79.93884,40.4426,-79.93875,40.44262,-79.93834,40.44289,-79.93817,40.44306,-79.93847};
//const float currentPath[] ={40.44385,-79.94635,40.44378,-79.94598};
const float currentPath[] ={40.44573,-79.93652,40.44589,-79.93618,40.44607,-79.93657};


const int sizeOfCurrentPath = 6;
 //  the below path is for disc golf course for when we are board
 // {40.43063F, -79.94538F, 40.43069F, -79.94652F, 40.43144F, -79.94668F, 40.43206F, -79.94578F};

static float previousPosition0[2] = {0,0};
static float previousPosition1[2] = {0,0};
static float previousPosition2[2] = {0,0};
static float* previousPositions[3] = {previousPosition0,previousPosition1,previousPosition2};
const int sizeOfpreviousPositions = 3;

Adafruit_GPS GPS(&HWSERIAL);
IntervalTimer myTimer;
IntervalTimer freqTimer;


static int finished = 0;
static int currentStartPos = 0;
static int currentEndPos = 1;
static int updated = 1;

float currentStartLat = 0;
float currentStartLong = 0;
float currentEndLat = 0;
float currentEndLong = 0;
float curr_GPS_lat = 0;
float curr_GPS_long = 0;

uint32_t timer = millis();

boolean at_start = false;
boolean pG = false;
void setup() {
  // put your setup code here, to run once:
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);
  pinMode(LEFT_LED, OUTPUT);
  digitalWrite(ONBOARD_LED,LOW);
  digitalWrite(RIGHT_LED,HIGH);
  digitalWrite(LEFT_LED,HIGH);
  delay(1000);
  digitalWrite(RIGHT_LED,LOW);
  digitalWrite(LEFT_LED,LOW);
  Serial.begin(115200);
  delay(1000);

  /* TODO: if we have left over memory, parse currentPath into 2d array for ease of use */

  Serial.println("SwimStraight Prototype V0");
  Serial.print("Starting GPS... ");
  setupGPS();
  Serial.println("Done");
  at_start = false;

  freqTimer.begin(updateStartAndEnd, 20000000);

  /* Lets set current start and end lat,long*/
  Serial.println("getting current lat and long");
  currentStartLat = currentPath[getLatIndex(currentStartPos)];
  currentStartLong = currentPath[getLongIndex(currentStartPos)];
  currentEndLat = currentPath[getLatIndex(currentEndPos)];
  currentEndLong = currentPath[getLongIndex(currentEndPos)];
  Serial.println("Leaving Start");
  Serial.flush();
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println("checking for fix");

//  //int status = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())){ // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
  }

  if (timer > millis()) timer = millis();

  // first we gotta make sure that the GPS has SATs before we can begin to do anything
  if (GPS.fix){
    digitalWrite(ONBOARD_LED,HIGH);

    // Latitude conversion
    int GPS_lat_integer = GPS.latitude/100;
    float GPS_lat_shifting = GPS.latitude - (GPS_lat_integer * 100);
    float GPS_lat_decimal_conversion = (GPS_lat_shifting / 60);
    curr_GPS_lat = GPS_lat_integer + GPS_lat_decimal_conversion;

    // Longitude conversion
    int GPS_long_integer = GPS.longitude/100;
    float GPS_long_shifting = GPS.longitude - (GPS_long_integer * 100);
    float GPS_long_decimal_conversion = (GPS_long_shifting / 60);
    curr_GPS_long = (-1)*(GPS_long_integer + GPS_long_decimal_conversion);

    if (at_start == false) {
      Serial.println(calcDist(currentStartLat, currentStartLong, curr_GPS_lat, curr_GPS_long));
      Serial.print("curr: ");
      Serial.printf("%.10f , " ,curr_GPS_lat); Serial.printf("%.10f\n" ,curr_GPS_long);
//      Serial.print("bearing between current and goal: ");
//      Serial.println(getBearing(curr_GPS_lat, curr_GPS_long, currentEndLat, currentEndLong));
//      Serial.print("bearing between start and goal: ");
//      Serial.println(getBearing(currentStartLat, currentStartLong, currentEndLat, currentEndLong));
//      Serial.print("bearing between current and start: ");
//      Serial.println(getBearing(curr_GPS_lat, curr_GPS_long, currentStartLat, currentStartLong));
//      Serial.print("angle to move:");
//      Serial.println(calcDir(currentStartLat, currentStartLong, curr_GPS_lat, curr_GPS_long, currentEndLat, currentEndLong));
      at_start = toBegin(curr_GPS_lat, curr_GPS_long);
      if (at_start == true){
        Serial.print("we are within ");
        Serial.print(MARGIN_RANGE);
        Serial.println(" meters of start");
      }
    } else {
      if (finished){
        Serial.printf("bruh, you are done.");
        exit(0);
      }
      float relative_pos = toMove(curr_GPS_lat, curr_GPS_long);

      if (1) {
        if (millis() - timer > 1200) {
          timer = millis();
          Serial.print("Remaining Distance: ");
          Serial.println(calcDist(currentEndLat, currentEndLong, curr_GPS_lat, curr_GPS_long));
          Serial.print("prev_1: ");
          Serial.printf("%.10f , " ,previousPositions[1][0]); Serial.printf("%.10f\n" ,previousPositions[1][1]);
          Serial.print("prev_2: ");
          Serial.printf("%.10f , " ,previousPositions[2][0]); Serial.printf("%.10f\n" ,previousPositions[2][1]);
          Serial.print("curr: ");
          Serial.printf("%.10f , " ,curr_GPS_lat); Serial.printf("%.10f\n" ,curr_GPS_long);
          Serial.print("Distance between curr and previous: ");
          Serial.println(calcDist(previousPositions[2][0], previousPositions[2][1], curr_GPS_lat, curr_GPS_long));
          if (relative_pos > DELTA_DEGREE) {
            Serial.print("move right: ");
            Serial.println(relative_pos);
            digitalWrite(RIGHT_LED, HIGH);
            digitalWrite(LEFT_LED, LOW);  
          } else if (relative_pos < -DELTA_DEGREE) {
            Serial.print("move left: ");
            Serial.println(relative_pos);
            digitalWrite(LEFT_LED, HIGH); 
            digitalWrite(RIGHT_LED, LOW);

          } else {
            Serial.println("on path");
            digitalWrite(RIGHT_LED, LOW);
            digitalWrite(LEFT_LED, LOW);
          }
        }
      }

    }
    if (calcDist(previousPositions[2][0], previousPositions[2][1], curr_GPS_lat, curr_GPS_long) > 1) {
      shiftandAddPosition(previousPositions, sizeOfpreviousPositions, curr_GPS_lat, curr_GPS_long);
      updated = 1;
    } else {
      updated = 0;
    }
  } else {
    Serial.println("looking for fix");
  }

}

void readGPS(){
  char c = GPS.read();
//  Serial.print(c);
}


void setupGPS(){
  GPS.begin(9600);
  Serial1.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
//  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  myTimer.begin(readGPS, 1000);
return;
}

//// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
//SIGNAL(TIMER0_COMPA_vect) {
//  char c = GPS.read();
//  // if you want to debug, this is a good time to do it!
//  if (GPSECHO)
//    if (c) UDR0 = c;
//    // writing direct to UDR0 is much much faster than Serial.print
//    // but only one character can be written at a time.
//}

void updateStartAndEnd() {

  if (calcDist(currentEndLat, currentEndLong, curr_GPS_lat, curr_GPS_long) < MARGIN_RANGE) {
    /* update interm 'start' values */
    currentStartPos++;
    currentStartLat = currentEndLat;
    currentStartLong = currentEndLong;

    /* update interm 'end' values */
    if (currentStartPos != sizeOfCurrentPath){
      currentEndPos++;
      currentEndLat = currentPath[getLatIndex(currentEndPos)];
      currentEndLong = currentPath[getLongIndex(currentEndPos)];
    }
    else {
      finished = 1;
      freqTimer.update(20000000);
    }
  } else if (calcDist(currentEndLat, currentEndLong, curr_GPS_lat, curr_GPS_long) < RANGE_RING_3) {
    freqTimer.update(2000000);
    //Serial.println("within 25 metres from goal");

  } else if (calcDist(currentEndLat, currentEndLong, curr_GPS_lat, curr_GPS_long) < RANGE_RING_2) {
    freqTimer.update(5000000);
    //Serial.println("within 50 metres from goal");
  } else if (calcDist(currentEndLat, currentEndLong, curr_GPS_lat, curr_GPS_long) < RANGE_RING_1) {
    freqTimer.update(10000000);
    //Serial.println("within 100 metres from goal");
  }

}

bool toBegin(float curr_lat, float curr_long) {
  if (calcDist(currentStartLat, currentStartLong, curr_lat, curr_long) < MARGIN_RANGE) {
    return true;
  } else {
    return false;
  }
}

float toMove(float curr_lat, float curr_long) {
  //populate array of current two points, not sure how we'll keep track yet

  // benefits of fixed size arrays
  //float oldPositionLat = (previousPositions[0][0]);//+previousPositions[1][0]+previousPositions[2][0])/3;
  //float oldPositionLong = (previousPositions[0][1]);//+previousPositions[1][1]+previousPositions[2][1])/3;

  float angle_1 = calcDir(curr_lat, curr_long, previousPositions[0][0], previousPositions[0][1],
   currentEndLat, currentEndLong);
  float angle_2 = calcDir(curr_lat, curr_long, previousPositions[1][0], previousPositions[1][1],
   currentEndLat, currentEndLong);
  float angle_3 = calcDir(curr_lat, curr_long, previousPositions[2][0], previousPositions[2][1],
   currentEndLat, currentEndLong);
  float angle = (angle_1 + angle_2 + angle_3)/3.0;
   return angle;
}

inline int getLatIndex(int index){
  return index*2;
}
inline int getLongIndex(int index){
  return (index*2)+1;
}
/* Store the latest positions incase we get around to calculating direction
 *
*/
void shiftandAddPosition(float **a,int n, float flat, float flong){

//  for(int i=0;i<n-1;i++)
//    {
//        a[i][0]=a[i+1][0];
//        a[i][1]=a[i+1][1];
//    }
    a[0][0]= a[1][0];
    a[0][1]= a[1][1];
    a[1][0]= a[2][0];
    a[1][1]= a[2][1];
    a[2][0]=flat;
    a[2][1]=flong;
}

/*************************************************************************
 * //Function to calculate the distance between two waypoints
 *************************************************************************/
float calcDir(float flat0, float flon0, float flat1, float flon1, float flat2, float flon2) {

  float goal_bearing = getBearing(flat0, flon0, flat2, flon2); //TODO: change points
  float curr_bearing = getBearing(flat1, flon1, flat0, flon0); //TODO: change points
  float diff = abs(goal_bearing - curr_bearing);
  float angle;

  if (goal_bearing >= curr_bearing) {
    if (diff < 180) {
      angle = diff;
    } else {
      angle = -(360 - diff);
    }
  } else {
    if (diff < 180) {
      angle = -diff;
    } else {
      angle = 360 - diff;
    }
  }

  return angle;
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

float to_degrees(float radians) {
    float deg = radians * (180.0 / M_PI);
    if (deg < 0){
      deg += 360;
    }
    return 360-deg;
}

float calcDist(float flat1, float flon1, float flat2, float flon2)
{
float dist_calc =0;
float dist_calc2 =0;
float diflat =0;
float diflon =0;

//I've to split all the calculation in several steps.
//If i try to do it in a single line the arduino will explode.
diflat=radians(flat2-flat1);
flat1=radians(flat1);
flat2=radians(flat2);
diflon=radians((flon2)-(flon1));

dist_calc =(sin(diflat/2.0)*sin(diflat/2.0));
dist_calc2 =cos(flat1);
dist_calc2*=cos(flat2);
dist_calc2*=sin(diflon/2.0);
dist_calc2*=sin(diflon/2.0);
dist_calc +=dist_calc2;

dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

dist_calc*=6371000.0; //Converting to meters
//Serial.println(dist_calc);
return dist_calc;
}
