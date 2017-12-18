// Receiver

#include<Wire.h>

int N_ELEMENTS = 2;

union float_as_bytes {byte b[4]; float fval;} u;


void receiveEvent(int numBytes){
    char data[4];
    int i = 0;
    while(Wire.available())    // slave may send less than requested
   { 
     data[i] = Wire.read(); // receive a byte as character 
     Serial.print("B: ");
     Serial.println(data[i]); 
     i = i + 1;
   }
    if(i==4){
        u.b[0] = data[0];    
        u.b[1] = data[1];
        u.b[2] = data[2];
        u.b[3] = data[3];
    
        float aux = u.fval;
        Serial.print("Pi = ");
        Serial.println(aux); 
    }
}

void setup() {
Serial.begin(9600);

Wire.begin(1);
Wire.onReceive(receiveEvent);
}

void loop() {  

}
