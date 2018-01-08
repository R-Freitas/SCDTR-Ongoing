#include <Wire.h>
#include <stdlib.h>

const int N_ELEMENTS = 2;
const int LedPin = 9;
const int LDRPin = A0;

union float_as_bytes {
  byte b[4]; 
  float fval;};

struct arduino_info {
  int endereco;
  double ganho;
};

//Sample time
double T_s  = 0.03;


//CONTROLER VARIABLES
float error_0 = 0;     // (k)-th error value [lux]
float error_1 = 0;     // (k-1)-th error value [lux] 
float outputValue_1 = 0; // (k-1)-th value output to the PWM (analog out)
float I = 0;
float I_1 = 0;
float D = 0;
float D_1 = 0;
float P = 0;
float C = 0;
float ff_value = 0;

//CONTROLER CONSTANTS
double Kp = 0.7;
double Ki = 25;
double Kc = 1;
double Kd = 0;
int a = 10;
int u =0;

//LOCAL CONTROLER
int ref_LOW  = 90;
int ref_HIGH = 170;
double feed_gain= 1.03;

float Lref;
float pwm;
float current_lux;
bool distributed_control = false;

//I2C VARIABLES
int address=0;
int pi_address = 9;
int found_elements=1;
byte control=1;
int stream =0;
int desk_number;
int my_index= -1;

//CALIBRATION VARIABLES
int recolhe_valores=0, calibre_count=0;
int PWM_Calibre=255;
arduino_info elements[N_ELEMENTS];

// -------------------------- New variables --------------------------------- //

float energy = 0;
float error = 0;
float errorBuffer = 0;
float errorCurrent = 0;
float variance = 0;
float varianceBuffer = 0;
float Li, Li_1, Li_2;
int metricsSampleCounter = 0;
//float K[2];


//Others
bool acende = 0;
unsigned long t_0,diff,t0; //time variables
float_as_bytes buf[N_ELEMENTS*N_ELEMENTS];

//STATE VARIABLES
int occupancy[2] = {0,0}; //0 for false and 1 for true


//Consensus global variables
int d_broadcast_count= 0;
int changed_occupancy= 1; //To compute new LUX ref in consensus. Set to zero after running consensus

// System definition
float Lmin[N_ELEMENTS];
float o[N_ELEMENTS];

// Cost function definition
float c[N_ELEMENTS] = {1,1};
float Q[N_ELEMENTS] = {1,1};


// ADMM auxiliary variables
float rho = 0.6;                       // ADMM penalty parameter
float y[N_ELEMENTS];                    // Lagrange multipliers vector
float d[N_ELEMENTS];                    // Final (optimal) duty cycles, for local arduino
float d_av[N_ELEMENTS];                 // Duty cycles average, for all arduinos
float d_copies[N_ELEMENTS][N_ELEMENTS]; // Duty cycles buffer, for all arduinos


void analyse_serial(){

  String data = "";
  while(Serial.available()){
    data += (char) Serial.read();
  }

  Serial.println(data);
  Serial.print(sizeof(data));  
  //data = Serial.readString();

  if(sizeof(data)>1){
    int received_ad = data[2] - '0';
    if(received_ad == desk_number){
      analyse_request(data);
    }
    else{
      if(elements[received_ad-1].endereco){ //checks if the desk_number exists
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

double sampleADC(){ 
  double Value = 0;
  for (int k=0;k<10;k++)
  {
    Value += analogRead(LDRPin);
  } 
  Value /= 10;
  return Value;
}

void calibracao (arduino_info* elements, int found_elements){
  
  
  while (calibre_count<found_elements){
    if (elements[calibre_count].endereco == address){
      desk_number = calibre_count +1;
      my_index = calibre_count;
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

      elements[calibre_count].ganho = sampleADC();  //Lê valores
      
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
        elements[calibre_count].ganho = sampleADC();
        //talvez apenas evocar uma vez, discutir com o duarte ou seja apos o adc meter recolhe_valores=0
      }
      digitalWrite(LED_BUILTIN,LOW);
    }
  
  }
  for (int i=0;i<found_elements;i++){
    elements[i].ganho= transform_ADC_in_lux(elements[i].ganho)/100;
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

void receiveEvent(int numBytes){
  String request="";   
  
  while(Wire.available()){
    request += (char) Wire.read();
  }

  analyse_request(request);
}

void analyse_request(String data){

  switch(data[0]){

    //CHANGE STATE
    case 's':
    int new_occupancy;
    new_occupancy = data[1] - '0';
    if(occupancy[my_index] != new_occupancy){
     occupancy[my_index] = new_occupancy;
     changed_occupancy= 1;
     for(int i=0; i< found_elements; i++){
      if(i != my_index){
        char send_index;
        char send_occupancy;
        send_occupancy = new_occupancy + '0';
        send_index = my_index + '0';
        while(control !=0){
          Wire.beginTransmission(elements[i].endereco);
          Wire.write("O");
          Wire.write(send_index);
          Wire.write(send_occupancy);
        control= Wire.endTransmission(true);
      }
      control = 1; 
    }
    }
    }
        Wire.beginTransmission(pi_address);
        Wire.write("ack");
        Wire.write('\0');
    break;

    case 'O':
    int receive_index;
    int state_occupancy;
    receive_index = data[1] - '0';
    state_occupancy = data[2] - '0';
    occupancy[receive_index] = state_occupancy;
    
    break;

    //Save offset values from other desks
    case 'F':

    break;

    
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

    case 'D':
        int index = (data[1] - '0') -1; //origin desk number - 1

        char buffer[7];
        for(int i=0; i<found_elements; i++){
            String received_value = data.substring(i*6+2, i*7+2+5);
            received_value.toCharArray(buffer,6);
            d_copies[index][i] = atof(buffer);
            // ----------- DEBUG Top ------------ //
            Serial.print("Received value: d = ");
            Serial.println(d_copies[index][i]);
            Serial.flush();
            // ---------------------------------- //
        }
        Serial.println("");
        d_broadcast_count++;
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

// ---------------------------- Consensus functions -------------------------------- //

float local_lux(float *d){
    float L = o[my_index];    // unconstrained solution illuminance at local desk
    for(int i=0; i<found_elements; i++){
        L += (elements[i].ganho)*d[i];
    }
    return L;
}

float cost_function(float *d){
    float f = 0;
    f = 0.5*Q[my_index]*d[my_index]*d[my_index]
    + c[my_index]*d[my_index];
    for(int i=0; i<found_elements; i++){
        f += y[i]*(d[i]-d_av[i])
        + 0.5*rho*(d[i]-d_av[i])*(d[i]-d_av[i]);
    }
    return f;
}

void feedforwardConsensus(){

    // ---------------------------------------------------------------------- //
    //                  1) Compute unconstrained solution
    // ---------------------------------------------------------------------- //

    float z[found_elements];
    float d_unconstrained[found_elements];
    for(int i=0; i<found_elements; i++){
        if(i!=my_index){
            z[i] = rho*d_av[i]-y[i];
            d_unconstrained[i] = z[i]/rho;
        }
        else if(i==my_index){
            z[i] = rho*d_av[i]-y[i]-c[i];
            d_unconstrained[i] = z[i]/(Q[i]+rho);
        }
    }

    // ------------------- Check solution feasibility ----------------------- //

    float L_unconstrained = local_lux(d_unconstrained);
    if ((d_unconstrained[my_index]>=0) &&
    (d_unconstrained[my_index]<=100) &&
    (L_unconstrained>=Lmin[my_index]-o[my_index]) ){
        for(int i=0; i<found_elements; i++){
            d[i] = d_unconstrained[i];    // The unconstrained solution is the
        }
        return;                       // optimal solution - all done!
    }

    // ---------------------- DEBUG (unconstrained)  ------------------------ //
    //
    // float f_unconstrained = cost_function(d_unconstrained);
    // for(int i=0; i<found_elements; i++){
    //     Serial.println(d_unconstrained[i]);
    // }
    // Serial.println(f_unconstrained);
    // Serial.println(" ");
    // Serial.flush();
    //
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    //       2) Compute best solution on the boundary - Initialization
    // ---------------------------------------------------------------------- //

    // Cost function minima for each of the boundary regions
    float f_linear = 0;
    float f_dcmin = 0;
    float f_dcmax = 0;
    float f_linear_dcmin = 0;
    float f_linear_dcmax = 0;
    float f_best = 100000;
    float d_best[found_elements];

    // ---------------------------------------------------------------------- //
    //    2.1) Compute best solution on the boundary - Linear constraint
    // ---------------------------------------------------------------------- //

    float n = 0;
    float w1 = 0;
    for(int i=0; i<found_elements; i++){
        if(i!=my_index){
            n += (elements[i].ganho)*(elements[i].ganho)/rho;
            w1 -= (elements[i].ganho)*z[i]/rho;
        }
        else if(i==my_index){
            n += (elements[i].ganho)*(elements[i].ganho)/(Q[i]+rho);
            w1 -= (elements[i].ganho)*z[i]/(Q[i]+rho);
        }
    }

    float d_linear[found_elements];
    for(int i=0; i<found_elements; i++){
        if(i!=my_index){
            d_linear[i] = z[i]/rho - (elements[i].ganho)*(o[my_index]-Lmin[my_index]-w1)/(n*rho);
        }
        else if(i==my_index){
            d_linear[i] = z[i]/(Q[i]+rho) - (elements[i].ganho)*(o[my_index]-Lmin[my_index]-w1)/(n*(Q[i]+rho));
        }
    }

    // ----- Check solution feasibility / get cost function value ----------- //

    float L_linear = local_lux(d_linear);
    if ((d_linear[my_index]>=0) &&
    (d_linear[my_index]<=100)){
        f_linear = cost_function(d_linear);
        if(f_linear<f_best){
            f_best = f_linear;
            for(int i=0; i<found_elements; i++){
                d_best[i] = d_linear[i];
            }
        }
    }

    // ------------------------  DEBUG (linear)  ---------------------------- //
    //
    // f_linear = cost_function(d_linear);
    // for(int i=0; i<found_elements; i++){
    //     Serial.println(d_linear[i]);
    // }
    // Serial.println(f_linear);
    // Serial.println(" ");
    // Serial.flush();
    //
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    //     2.2) Compute best solution on the boundary - dcmin constraint
    // ---------------------------------------------------------------------- //

    float d_dcmin[found_elements];
    for(int i=0; i<found_elements; i++){
        if(i!=my_index){
            d_dcmin[i] = z[i]/rho;
        }
        else if(i==my_index){
            d_dcmin[i] = 0;
        }
    }

    // ----- Check solution feasibility / get cost function value ----------- //

    float L_dcmin = local_lux(d_dcmin);
    if ((d_dcmin[my_index]<=100) &&
    (L_dcmin>=Lmin[my_index]-o[my_index]) ){
        f_dcmin = cost_function(d_dcmin);
        if(f_dcmin<f_best){
            f_best = f_dcmin;
            for(int i=0; i<found_elements; i++){
                d_best[i] = d_dcmin[i];
            }
        }
    }

    // -------------------------  DEBUG (dcmin)  ---------------------------- //
    //
    // f_dcmin = cost_function(d_dcmin);
    // for(int i=0; i<found_elements; i++){
    //     Serial.println(d_dcmin[i]);
    // }
    // Serial.println(f_dcmin);
    // Serial.println(" ");
    // Serial.flush();
    //
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    //     2.3) Compute best solution on the boundary - dcmax constraint
    // ---------------------------------------------------------------------- //

    float d_dcmax[found_elements];
    for(int i=0; i<found_elements; i++){
        if(i!=my_index){
            d_dcmax[i] = z[i]/rho;
        }
        else if(i==my_index){
            d_dcmax[i] = 100;
        }
    }

    // ------------------- Check solution feasibility ----------------------- //

    float L_dcmax = local_lux(d_dcmax);
    if ((d_dcmax[my_index]>=0) &&
    (L_dcmax>=Lmin[my_index]-o[my_index]) ){
        f_dcmax = cost_function(d_dcmax);
        if(f_dcmax<f_best){
            f_best = f_dcmax;
            for(int i=0; i<found_elements; i++){
                d_best[i] = d_dcmax[i];
            }
        }
    }

    // -------------------------  DEBUG (dcmax)  ---------------------------- //
    //
    // f_dcmax = cost_function(d_dcmax);
    // for(int i=0; i<found_elements; i++){
    //     Serial.println(d_dcmax[i]);
    // }
    // Serial.println(f_dcmax);
    // Serial.println(" ");
    // Serial.flush();
    //
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    // 2.4) Compute best solution on the boundary: linear and dcmin constraints
    // ---------------------------------------------------------------------- //

    float g = (rho+Q[my_index]) / (n*(rho+Q[my_index]) - (elements[my_index].ganho)*(elements[my_index].ganho));
    float v = 0;
    float kr2 = 0;
    for(int i=0; i<found_elements; i++){
        if(i==my_index){
            continue;
        }
        else{
            v   += z[i]*elements[i].ganho;
            kr2 += (elements[i].ganho)*(elements[i].ganho);
        }
    }

    float d_linear_dcmin[found_elements];
    for(int i=0; i<found_elements; i++){
        if(i!=my_index){
            d_linear_dcmin[i] = z[i]/rho + g*((elements[i].ganho)*(Lmin[my_index]-o[my_index]-v/rho))/rho;
        }
        else if(i==my_index){
            d_linear_dcmin[i] = z[i]/(Q[i]+rho) - g*(kr2*z[i]/(Q[i]+rho))/rho; // <----- BUG: solved, wrong sign in the handout
        }
    }

    // ------------------- Check solution feasibility ----------------------- //

    float L_linear_dcmin = local_lux(d_linear_dcmin);
    if (d_linear_dcmin[my_index]<=100){
        f_linear_dcmin = cost_function(d_linear_dcmin);
        if(f_linear_dcmin<f_best){
            f_best = f_linear_dcmin;
            for(int i=0; i<found_elements; i++){
                d_best[i] = d_linear_dcmin[i];
            }
        }
    }

    // ----------------------- DEBUG (linear, dcmin)  ----------------------- //
    //
    // f_linear_dcmin = cost_function(d_linear_dcmin);
    // for(int i=0; i<found_elements; i++){
    //     Serial.println(d_linear_dcmin[i]);
    // }
    // Serial.println(f_linear_dcmin);
    // Serial.println(" ");
    // Serial.flush();
    //
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    // 2.5) Compute best solution on the boundary: linear and dcmax constraints
    // ---------------------------------------------------------------------- //

    float d_linear_dcmax[found_elements];
    for(int i=0; i<found_elements; i++){
        if(i!=my_index){
            d_linear_dcmax[i] = d_linear_dcmin[i] - 100*(elements[i].ganho)*(elements[my_index].ganho)*g/rho;
        }
        else if(i==my_index){
            d_linear_dcmax[i] = d_linear_dcmin[i] + 100*kr2;
        }
    }

    // ------------------- Check solution feasibility ----------------------- //

    float L_linear_dcmax = local_lux(d_linear_dcmax);
    if (d_linear_dcmax[my_index]>=0){
        f_linear_dcmax = cost_function(d_linear_dcmax);
        if(f_linear_dcmax<f_best){
            f_best = f_linear_dcmax;
            for(int i=0; i<found_elements; i++){
                d_best[i] = d_linear_dcmax[i];
            }
        }
    }

    // ----------------------- DEBUG (linear, dcmax)  ----------------------- //
    //
    // f_linear_dcmax = cost_function(d_linear_dcmax);
    // for(int i=0; i<found_elements; i++){
    //     Serial.println(d_linear_dcmax[i]);
    // }
    // Serial.println(f_linear_dcmax);
    // Serial.println(" ");
    // Serial.flush();
    //
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    //              2.6) Retrieve best solution on the boundary
    // ---------------------------------------------------------------------- //

    for(int i=0; i<found_elements; i++){
        d[i] = d_best[i];
        //Serial.print("(after feedforwardConsensus() ) d[i] = ");
        //Serial.println(d[i]);
    }

    // Serial.println(f_best);
    return;
}

void compute_d_av(){
    for(int j=0; j<found_elements; j++){
        d_av[j] = 0;
        for(int i=0; i<found_elements; i++){
            if(i!=my_index){
                d_av[j] += d_copies[i][j];
            }
            else if(i==my_index){
                d_av[j] += d[j];
            }
        }
        d_av[j] /= found_elements;
        // ----------- DEBUG ------------ //
        // Serial.print("d(");
        // Serial.print(j);
        // Serial.print(") = ");
        // Serial.print(d[j]);
        // Serial.print("\t\t d_av(");
        // Serial.print(j);
        // Serial.print(") = ");
        // Serial.println(d_av[j]);
        // ----------------------------- //
    }
    // Serial.println("");
}

void update_y(){
    for(int i=0; i<found_elements; i++){
        y[i] += rho*(d[i]-d_av[i]);
        // ----------- DEBUG ------------ //
        // Serial.print("y(");
        // Serial.print(i);
        // Serial.print(") = ");
        // Serial.println(y[i]);
        // ----------------------------- //
    }
    // Serial.println("");
}

void update_d_copies(){
    for(int i=0; i<found_elements; i++){
        d_copies[my_index][i] = d[i];
    }
}

void broadcast_d(){

    while(d_broadcast_count<found_elements){
        t_0=micros();
        diff=0;
        while(diff<10){
            diff=micros()-t_0;
        }
        if (elements[d_broadcast_count].endereco == address){
            char send_mydesk;
            int aux = my_index + 1;
            send_mydesk = aux + '0';
            while(control !=0){
                Wire.beginTransmission(0);
                Wire.write('D');
                Wire.write(send_mydesk);
                    for(int i=0; i<found_elements; i++){
                        char temp[6];
                        dtostrf(d[i],6,3,temp);
                        Wire.write(temp, sizeof(temp));
                    }
                control = Wire.endTransmission(true);
            }
            control = 1;
            d_broadcast_count++;
            // ----------- DEBUG Top ------------ //
            for(int i=0; i<found_elements; i++){
                Serial.print("Sent value: d = ");
                Serial.println(d[i]);
            }
            Serial.println("");
            // --------------------------------- //
        }
    }
    d_broadcast_count = 0;
    // ----------- DEBUG Top ------------ //
    Serial.println("d_copies = ");
    for(int i=0; i<found_elements; i++){
        for(int j=0; j<found_elements; j++){
            Serial.print(d_copies[i][j]);
            Serial.print("\t");
        }
        Serial.println("");
    }
    // --------------------------------- //
}

// --------------------------- Other functions ------------------------------ //


float compute_Lref(){
    Lref = 0;
    for(int i=0; i<found_elements; i++){
        Lref += (elements[i].ganho)*d[i];
    }
    Lref += o[my_index];
    return Lref;
}

void compute_stats(){
    metricsSampleCounter++;
    double Lmeasured = transform_ADC_in_lux(analogRead(LDRPin));

    // Accumulated comfort error (since last system restart)
    double errorCurrent = Lref - Lmeasured;
    if(error > 0){
        errorBuffer += errorCurrent;
    }
    error = errorBuffer/metricsSampleCounter;

    // Accumulated comfort variance (since last system restart)
    if(metricsSampleCounter==1){
        Li_2 = Lmeasured;
    }
    else if (metricsSampleCounter==2){
        Li_1 = Lmeasured;
    }
    else if(metricsSampleCounter>=3){
        Li_2 = Li_1;
        Li_1 = Li;
        Li = Lmeasured;
        varianceBuffer += abs(Li - 2*Li_1 + Li_2);
    }
    variance = varianceBuffer/(metricsSampleCounter*T_s*T_s);
    // Accumulated energy consumption (since last system restart)
    energy += d[my_index]*T_s;
}

void check_occupancy(){
  for(int i=0; i<found_elements; i++)
    if(occupancy[i] == 0){
      Lmin[i] = ref_LOW;
    }
    else if(occupancy[i] == 1){
      Lmin[i] = ref_HIGH;     
    }
 }

float PI_controler(float Lref, float measured_lux, float ff_value){

  float outputValue_0;
  error_0 = Lref - measured_lux;

if (abs(error_0) < 2)
{
  error_0 = 0;
}

//P Controller

P = Kp*error_0;

//PI Controller

I = I_1 + 0.5*Kp*Ki*T_s*(error_0 + error_1);

//PD Controller

D = (Kd/(Kd+a*T_s))*D_1 - Kp*Kd*a*(outputValue_0-outputValue_1)/(Kd + a*T_s);

//Complete Controller

C = P + I ;

outputValue_0 = C + ff_value;

if(outputValue_0  < 0)
{
  outputValue_0 = 0;
  u = 0;
  I_1 = I + Kc*(u-C-ff_value);
}
else if(outputValue_0 > 255)
{
  outputValue_0 = 255;
  u = 255;
  I_1 = I + Kc*(u-C-ff_value);
}
else{
  I_1 = I;
}


outputValue_1 = outputValue_0;
error_1 = error_0;
D_1 = D;

return outputValue_0;  //Value to write in the led
}


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
  Wire.begin(address);  //Initialize I2C comunications
  Wire.onReceive(receiveEvent);
  bitSet(TWAR, TWGCE);  //Enable general call
  delay(100);
  
  propagate_address(address);               //Propagates the address so its know by everyone
  
  t_0=micros();
  while(diff<500000)
  {
    diff=micros()-t_0;
  }
  
  sort_copy(&elements[0],found_elements);   //Sort found addresses for use in calibration
  calibracao(&elements[0],found_elements);  //Calibration


     // CONSENSUS: System / cost function variables initialization
     for(int i=0; i<found_elements; i++){
         //Lmin[i] = 160;
         c[i] = 1;           // Unit cost, equal for all luminaires
         Q[i] = 1;           // Unit cost, equal for all luminaires
     }
      
    // CONSENSUS : ADMM auxiliary variables initialization
    for(int i=0; i<found_elements; i++){
        y[i] = 0;
        d[i] = 0;
        d_av[i] = 0;
        for(int j=0; j<found_elements; j++){
            d_copies[i][j] = 0;
        }
    }

  analogWrite(LedPin,0);
  delay(1000);
  o[my_index] = transform_ADC_in_lux(sampleADC());
  
  //Serial.println(o[my_index]);  
  Serial.println("Setup ended");
  //analogWrite(LedPin,PWM_Calibre);
  t0 = millis();
}


void loop() {

check_occupancy();
 
  if(Serial.available()> 0){
    analyse_serial();
  }

if(distributed_control == false){
  if (occupancy[my_index] == 1){
    ff_value = ref_HIGH* feed_gain;
    Lref = ref_HIGH;  
  }
  else if(occupancy[my_index] == 0){
  ff_value = ref_LOW * feed_gain;
  Lref= ref_LOW;
}


current_lux = transform_ADC_in_lux(sampleADC());

pwm =  PI_controler(Lref, current_lux, ff_value);

analogWrite(LedPin, pwm);
}
else{

  if(changed_occupancy){
  
  //CONSENSUS

    int iterations = 20;
    for(int i=1; i<=iterations; i++){

        // ----------- DEBUG Top ------------ //
        Serial.println("\n--------------------------------------------");
        Serial.print("\t\tIteration: ");
        Serial.println(i);
        Serial.println("--------------------------------------------\n");
        // ---------------------------------- //

        // compute duty-cycles vector, d
        feedforwardConsensus();
        // update duty cycle array, d
        update_d_copies();
        // compute average duty-cycles vector, d_av
        compute_d_av();
        // update local lagrangian, y
        update_y();
        // broadcast / receive computed 'd' vectors
        broadcast_d();
    }
  
  Lref = compute_Lref();
  ff_value = d[my_index];
  changed_occupancy = 0;
  }
  
  current_lux = transform_ADC_in_lux(sampleADC());
  pwm =  PI_controler(Lref, current_lux, ff_value);
  analogWrite(LedPin, 2.55*pwm);
  
}

//Serial.println(pwm);


  if(acende==true){
  digitalWrite(LED_BUILTIN,HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN,LOW);    
  acende = false;
  }

  //CONSENSUS CALCULATIONS

/*
  Serial.print("My index is: ");
  Serial.println(my_index);
  Serial.println(elements[0].ganho);
  Serial.println(elements[1].ganho); */

  


 
  
  if(stream !=0){
    streaming();
  }

  
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
  }
  
  delay(T_s);
  
}


