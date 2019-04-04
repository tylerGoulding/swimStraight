#include <EEPROM.h>

typedef struct{
    uint8_t minutes;
    uint8_t seconds;
    float lat;
    float lon;
} latLongStruct;

void setup(){
  latLongStruct lls;
  int eeAddress = 0; //EEPROM address to start reading from

  Serial.begin(115200);
  delay(10000);
  Serial.println( "Read from EEPROM: " );

  //Get the float data from the EEPROM at position 'eeAddress'
  while (eeAddress < 2048){
    EEPROM.get( eeAddress, lls );
    Serial.print(lls.minutes); Serial.print('.');
    Serial.print(lls.seconds); Serial.print(", ");
    Serial.printf("%.12f , ", lls.lat);  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
    Serial.printf("%.12f\n", lls.lon);
    Serial.flush();
    eeAddress += sizeof(latLongStruct); //Move address to the next byte after float 'f'.
  }

  Serial.println( "Done Reading from EEPROM " );
  
}

void loop(){ /* Empty loop */ }
