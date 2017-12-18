// Sender

#include<Wire.h>

int N_ELEMENTS = 2;

union float_as_bytes {byte b[4]; float fval;};

void setup() {
Serial.begin(9600);
Wire.begin();

float pi = 3.14;

int control = 1;
while(control!=0){
    Serial.println("Trying...");
    Wire.beginTransmission(1);
    Wire.write((byte*) &pi,4);
    control = Wire.endTransmission(); 
}
control = 1;
}

void loop() {   
}
