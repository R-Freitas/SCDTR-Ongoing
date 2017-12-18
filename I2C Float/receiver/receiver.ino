// Receiver

#include<Wire.h>

const int N_ELEMENTS = 2;

union float_as_bytes {byte b[4]; float fval;};

float_as_bytes u[N_ELEMENTS];

void receiveEvent(int numBytes){
    char data_receive[N_ELEMENTS][4];
    int i = 0;
    while(Wire.available()){    // slave may send less than requested   
        for(int j=0; j<4; j++){
        data_receive[i][j] = Wire.read(); // receive a byte as character 
        u[i].b[j] = data_receive[i][j];
        Serial.print("B: ");
        Serial.println(data_receive[i]); 
        }
    i++;
    }
    
   if(i==2){      
        for(int i=0; i<N_ELEMENTS; i++){
        float aux = u[i].fval;
        Serial.print("float(");
        Serial.print(i);
        Serial.print(") = ");
        Serial.println(aux); 
        }
    }
}

void setup() {
Serial.begin(9600);

Wire.begin(8);
Wire.onReceive(receiveEvent);
}

void loop() {  

}
