#include <Wire.h>

int i=0;

void setup() {
  Serial.begin(9600);
  Wire.begin(2);  //Initialize I2C comunication
  bitSet(TWAR, TWGCE);  //Enable general call
  Wire.onReceive(receiveEvent);
}

void loop() {
  String msg="";
  msg="v T var\n";
  Serial.print(msg);
  respond_to_raspberry(msg);
  delay(500);
  i++;

}
void respond_to_raspberry(String response){
   Wire.beginTransmission(9);
   Wire.write(response.c_str());
   Wire.endTransmission(true);

}
void receiveEvent(int numBytes){
    
}

