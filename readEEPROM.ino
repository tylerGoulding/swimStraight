#include <EEPROM.h>

typedef struct{
    float lat;
    float long;
} latLongStruct;

void setup(){
  latLongStruct lls;
  int eeAddress = 0; //EEPROM address to start reading from

  Serial.begin(115200);
  Serial.println( "Read from EEPROM: " );

  //Get the float data from the EEPROM at position 'eeAddress'
  while (eeAddress < 2048){
    EEPROM.get( eeAddress, lls );
    Serial.println(lls.lat + ", " + lls.long);  //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
    eeAddress += sizeof(latLongStruct); //Move address to the next byte after float 'f'.
  }

  Serial.println( "Done Reading from EEPROM " );
}

void loop(){ /* Empty loop */ }
