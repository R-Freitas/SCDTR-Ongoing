#include <Wire.h>
#include <stdlib.h>

char data[10];
int serial_index=0;

void setup() {
  Serial.begin(9600);
  
}

void loop() {
  if Serial.available(){
    analyse_serial();
  }
}

void analyse_serial(){
  char data[10];
  int index=0;
  while(Serial.available()>0){
    data[index]=(char)Serial.read();
    index++;
  }

  Serial.println(data);  
  //data = Serial.readString();
  /*
  if(sizeof(data)>1){
    int received_ad = data[2] - '0';
    if(received_ad == desk_number){
      analyse_request(data);
    }
    else{
      int send_to = elements[received_ad-1].endereco;
      while(control !=0){
        Wire.beginTransmission(send_to);
        Wire.write(data.c_str());
        control= Wire.endTransmission(true);
      }
      control = 1;
    }
  } 
  */
}
