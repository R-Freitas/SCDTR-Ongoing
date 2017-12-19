// Sender

#include<Wire.h>

int N_ELEMENTS = 2;

void setup() {
Serial.begin(9600);
Wire.begin();

float data_send[N_ELEMENTS] = {1.23, 4.56};

int control = 1;
while(control!=0){
    Serial.println("Trying...");
    Wire.beginTransmission(8);
    Wire.write((byte*) &data_send, 4*N_ELEMENTS);
    control = Wire.endTransmission(); 
}
control = 1;
}

void loop() {   
}
