#include <Wire.h>
#include <stdlib.h>
#define N_ELEMENTS 4

const int LedPin = 9;
const int LDRPin = A0;


struct arduino_info {
  int endereco;
  double ganho;
};



arduino_info elements[N_ELEMENTS];
int address=0;
int pi_address = 9;
int found_elements=1;
byte control=1;
bool acende = 0;
int desk_number;
int stream =0;
unsigned long t_0,diff,t0;
int recolhe_valores=0, calibre_count=0;
int PWM_Calibre=255;
int occupancy = 0; //0 for false and 1 for true


void setup() {

  int n=0;
  
  
  Serial.begin(9600);
  Serial.println("Begin Setup");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);
  pinMode(7,INPUT_PULLUP);
  pinMode(8,INPUT_PULLUP);
  
  digitalWrite(LED_BUILTIN,HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN,LOW);

  
  // Various initialization processes needed (such as reading adress pins, enabling general call, and using builtin led for control purposes)

  for (int i=2; i <= 8; i++){
    bitWrite(address, n, !digitalRead(i));
    n++;
  }//reading the address coded in digital ports from 2 to 8
  
  elements[0].endereco=address;  //Saving own address
  Wire.begin(address);  //Initialize I2C comunication
  bitSet(TWAR, TWGCE);  //Enable general call
  Wire.onReceive(receiveEvent);

  delay(100);
  
  propagate_address(address);               //Propagates the address so its know by everyone
  
  t_0=micros();
  while(diff<500000)
  {
    diff=micros()-t_0;
  }
  
  sort_copy(&elements[0],found_elements);   //Sort found addresses for use in calibration
  calibracao(&elements[0],found_elements);  //Calibration
  Serial.println("Setup ended");
  //analogWrite(LedPin,PWM_Calibre);
  t0 = millis();
   
}

void loop() {

  if(Serial.available()> 0){
    analyse_serial();
  }
  
  if(acende==true){
  digitalWrite(LED_BUILTIN,HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN,LOW);    
  acende = false;
  }

  /*
  Serial.print(" \n");
  for (int j=0;j<found_elements;j++){
    Serial.print("Endereco: ");
    Serial.print(elements[j].endereco);
    Serial.print("\t");
    Serial.print("Ganho: ");
    Serial.print(elements[j].ganho);
    Serial.print("\t");
    Serial.println(j);
    Serial.println(transform_ADC_in_lux(analogRead(LDRPin)));
  }*/
  
  delay(500);

  if(stream !=0){
    streaming();
  }
  
}

void analyse_serial(){

  String data = "";
  while(Serial.available()){
    data += (char) Serial.read();
  }

  Serial.println(data);  
  //data = Serial.readString();

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
 
 }

void receiveEvent(int numBytes){
  String request="";
  
  while(Wire.available()){
    request += (char) Wire.read();
  }

  analyse_request(request);
}

void sort_copy (arduino_info* arr, int size){
  for (int k=0; k<(size-1);k++){
    for (int w=0; w<(size-(k+1));w++){
      if (arr[w].endereco > arr[w+1].endereco){
        arduino_info temp; 
        temp.endereco= arr[w].endereco;
        arr[w].endereco=arr[w+1].endereco;
        arr[w+1].endereco=temp.endereco;
      }
    }
  }

}

void sampleADC(arduino_info* arr)
{ 
  double Value = 0;
  for (int k=0;k<10;k++)
  {
    Value += analogRead(LDRPin);
  } 
  Value /= 10;
  arr[0].ganho=Value;
}

void calibracao (arduino_info* elements, int found_elements){
  
  
  while (calibre_count<found_elements){
    if (elements[calibre_count].endereco == address){
      desk_number = calibre_count +1;
      analogWrite(LedPin,PWM_Calibre);  //liga led
      
      t_0=micros();//Waits to ensure everything is read (se calhar considerar resposta para garantir comunicação)
      diff=0;
      while(diff<1000000){
        diff=micros()-t_0;
      }
      
      while (control != 0){ //Transmite mensagem para os outros colocarem a escuta
        Wire.beginTransmission(0);
        Wire.write("CS");        
        control = Wire.endTransmission(true); 
      }
      control=1;
      
      sampleADC(&elements[calibre_count]);  //Lê valores
      
      t_0=micros();//Waits to ensure everything is read (se calhar considerar resposta para garantir comunicação)
      diff=0;
      while(diff<1000000){
        diff=micros()-t_0;
      }
      
      analogWrite(LedPin,0);//Desliga Led
      
      while (control != 0){ //Transmite mensagem de flag para sairem de escuta
        Wire.beginTransmission(0);
        Wire.write("CE");        
        control = Wire.endTransmission(true);
      }
      control=1;
      
      calibre_count++; //incrementa

    }
    else{
      if (recolhe_valores == 1){
        digitalWrite(LED_BUILTIN,HIGH);
        sampleADC(&elements[calibre_count]);
        //talvez apenas evocar uma vez, discutir com o duarte ou seja apos o adc meter recolhe_valores=0
      }
      digitalWrite(LED_BUILTIN,LOW);
    }
  
  }
  for (int i=0;i<found_elements;i++){
    elements[i].ganho=transform_ADC_in_lux(elements[i].ganho)/PWM_Calibre;
  }
}

void propagate_address(int address){
  while (control != 0){
    Wire.beginTransmission(0);
    Wire.write('E');
    Wire.write(address);      
    control = Wire.endTransmission(true);
    
  }
  control=1;
  
  
}

double transform_ADC_in_lux(double sensorValue){
  double V_sensorValue=0, R_LDR=0, lux=0;
  double m = -1.2618595, b = 2.54480706;

  V_sensorValue = (5*sensorValue/1023);
  R_LDR = (10/V_sensorValue)*(5-V_sensorValue);
  lux = pow(10,b)*pow(R_LDR,m);
  return lux;
  
}

void analyse_request(String data){

  switch(data[0]){

    //CHANGE STATE
    
    //ANSWER RASPBERRY PI
    char send_desk;
    send_desk = desk_number + '0';
    char send_occupancy;
    char temp[6];
    case 'c': //send stream
      if(data[1]== 'l'){
        if (stream ==0 || stream ==1)
          stream = 1; //initialize lux streaming
       else
          stream = 3; //initialize both lux and duty cycle streaming
      }
      else if(data[1]== 'd'){
      if (stream ==0 || stream == 2)
          stream = 2; //streams duty cycle values only
       else
          stream = 3; //streams both lux and duty cycle
      }      
      break;
      
      case 'd':
      if(data[1]== 'l'){
        if (stream ==0 || stream ==1)
          stream = 0; //stops streaming
       else
          stream = 2; //streams duty cycle only
      }
      else if(data[1]== 'd'){
      if (stream ==0 || stream == 2)
          stream = 0; //stops all streaming
       else
          stream = 1; //streams lux only
      }      
      break;

      case 'g':
      if(data[1]== 'l'){ //get lux
      double lux_value = transform_ADC_in_lux(analogRead(LDRPin));
      dtostrf(lux_value,4,2,temp);  
      while(control !=0){
        Wire.beginTransmission(pi_address);
        Wire.write("l");
        Wire.write(send_desk);
        Wire.write(temp, sizeof(temp)); //sends 6 bytes: size of array
        Wire.write('\0');
        control= Wire.endTransmission(true);
      }
      control = 1;  
      }
      else if(data[1]== 'o'){
        send_occupancy = occupancy + '0';
        while(control !=0){
        Wire.beginTransmission(pi_address);
        Wire.write("o");
        Wire.write(send_desk);
        Wire.write(send_occupancy); //sends 6 bytes: size of array
        Wire.write('\0');
        control= Wire.endTransmission(true);
      }
      control = 1;
      }

      break;

      case 't': //test if pi is receiving
        Wire.beginTransmission(pi_address);
        Wire.write("OK\0");
        control= Wire.endTransmission(true);
      break;
      
   //ANSWER ARDUINO
   case 'E':
      elements[found_elements].endereco= data[1];
      found_elements++;
      t_0=micros();
      break;
    
    case 'C':
      if (data[1]=='S'){
        recolhe_valores=1;
      }
      if (data[1]=='E'){
        recolhe_valores=0;
        calibre_count++;
      }
      break;

    case 'A':   
      acende = true;
      break; 
  }
}

void streaming(){
  double lux_value;
    char pwm_value;
    char temp[6];
    char temp2[6];
    char pwm[6];
    char send_desk;
   
   switch(stream){
    case 1: //streams lux values to raspberry pi
      lux_value = transform_ADC_in_lux(analogRead(LDRPin));
      dtostrf(lux_value,4,2,temp);
      dtostrf(millis(),4,0,temp2);
      send_desk = desk_number + '0';  
      while(control !=0){
        Wire.beginTransmission(pi_address);
        Wire.write("cl");
        Wire.write(send_desk);
        Wire.write(temp, sizeof(temp)); //sends 6 bytes: size of array
        Wire.write(temp2, sizeof(temp2)); //sends 6 bytes: size of array
        Wire.write('\0');
        control= Wire.endTransmission(true);
      }
      control = 1;
      break;

    case 2:
      //pwm_value = (int) 300 + '0';
      dtostrf(300,4,1,pwm);
      dtostrf(millis(),4,0,temp2);
      send_desk = desk_number + '0';
      while(control !=0){
        Wire.beginTransmission(pi_address);
        Wire.write("cd");
        Wire.write(send_desk);
        Wire.write(pwm, sizeof(pwm)); //sends 6 bytes
        //Wire.write((byte *)&pwm_value, sizeof(int));
        Wire.write(temp2, sizeof(temp2)); //sends 6 bytes: size of array
        Wire.write('\0');
        control= Wire.endTransmission(true);
      }
      control = 1;
      break;
    
    case 3:
      //First sends lux
      lux_value = transform_ADC_in_lux(analogRead(LDRPin));
      dtostrf(lux_value,4,2,temp);
      dtostrf(millis(),4,0,temp2);
      send_desk = desk_number + '0';  
      while(control !=0){
        Wire.beginTransmission(pi_address);
        Wire.write("cl");
        Wire.write(send_desk);
        Wire.write(temp, sizeof(temp)); //sends 6 bytes: size of array
        Wire.write(temp2, sizeof(temp2)); //sends 6 bytes: size of array
        Wire.write('\0');
        control= Wire.endTransmission(true);
      }
      control = 1;
      //Then sends pwm
      dtostrf(300,3,0,pwm);
      dtostrf(millis(),4,0,temp2);
      send_desk = desk_number + '0';
      while(control !=0){
        Wire.beginTransmission(pi_address);
        Wire.write("cd");
        Wire.write(send_desk);
        Wire.write(pwm, sizeof(pwm)); //sends 6 bytes
        //Wire.write((byte *)&pwm_value, sizeof(int));
        Wire.write(temp2, sizeof(temp2)); //sends 6 bytes: size of array
        Wire.write('\0');
        control= Wire.endTransmission(true);
      }
      control = 1;
      break;
    
 }
  
}

