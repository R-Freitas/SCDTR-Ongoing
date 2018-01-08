// Library inclusions
#include <Wire.h>

// ------------------------ My global variables ----------------------------- //

const int N_ELEMENTS = 2;
int found_elements = 2;

struct arduino_info {int endereco; double ganho;};

// System definition
float Lmin[N_ELEMENTS] = {150,80};
float o[N_ELEMENTS] = {30,0};

// Cost function definition
float c[N_ELEMENTS] = {1,1};
float Q[N_ELEMENTS] = {1,1};

// ADMM auxiliary variables
float rho = 0.6;                       // ADMM penalty parameter
float y[N_ELEMENTS];                    // Lagrange multipliers vector
float d[N_ELEMENTS];                    // Final (optimal) duty cycles, for local arduino
float d_av[N_ELEMENTS];                 // Duty cycles average, for all arduinos
float d_copies[N_ELEMENTS][N_ELEMENTS]; // Duty cycles buffer, for all arduinos

// MUDAR PARA CADA ARDUINO              <-------------------
int my_index = 1;                                           // 1
arduino_info elements[N_ELEMENTS] = {{20, 1}, {21, 2}};     // {{20, 1}, {21, 2}}

// Other variables
union float_as_bytes {  byte b[4];   float fval;};
float_as_bytes u[N_ELEMENTS*N_ELEMENTS];
byte control = 1;

int address = elements[my_index].endereco;
int d_broadcast_count = 0;
unsigned long t_0,diff,t0;

// ---------------------------- My functions -------------------------------- //

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
    float f_unconstrained = cost_function(d_unconstrained);
    for(int i=0; i<found_elements; i++){
        Serial.println(d_unconstrained[i]);
    }
    Serial.println(f_unconstrained);
    Serial.println(" ");
    Serial.flush();
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
    f_linear = cost_function(d_linear);
    for(int i=0; i<found_elements; i++){
        Serial.println(d_linear[i]);
    }
    Serial.println(f_linear);
    Serial.println(" ");
    Serial.flush();
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
    f_dcmin = cost_function(d_dcmin);
    for(int i=0; i<found_elements; i++){
        Serial.println(d_dcmin[i]);
    }
    Serial.println(f_dcmin);
    Serial.println(" ");
    Serial.flush();
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
    f_dcmax = cost_function(d_dcmax);
    for(int i=0; i<found_elements; i++){
        Serial.println(d_dcmax[i]);
    }
    Serial.println(f_dcmax);
    Serial.println(" ");
    Serial.flush();
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
    f_linear_dcmin = cost_function(d_linear_dcmin);
    for(int i=0; i<found_elements; i++){
        Serial.println(d_linear_dcmin[i]);
    }
    Serial.println(f_linear_dcmin);
    Serial.println(" ");
    Serial.flush();
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
    f_linear_dcmax = cost_function(d_linear_dcmax);
    for(int i=0; i<found_elements; i++){
        Serial.println(d_linear_dcmax[i]);
    }
    Serial.println(f_linear_dcmax);
    Serial.println(" ");
    Serial.flush();
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
        Serial.print("d(");
        Serial.print(j);
        Serial.print(") = ");
        Serial.print(d[j]);
        Serial.print("\t\t d_av(");
        Serial.print(j);
        Serial.print(") = ");
        Serial.println(d_av[j]);
        // ----------------------------- //
    }
    Serial.println("");
}

void update_y(){
    for(int i=0; i<found_elements; i++){
        y[i] += rho*(d[i]-d_av[i]);
        // ----------- DEBUG ------------ //
        Serial.print("y(");
        Serial.print(i);
        Serial.print(") = ");
        Serial.println(y[i]);
        // ----------------------------- //
    }
    Serial.println("");
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

void receiveEvent(int numBytes){
    String data="";

    while(Wire.available()){
        data += (char) Wire.read();
    }

    analyse_request(data);
}

void analyse_request(String data){

    switch(data[0]){
        //
        //  //CHANGE STATE
        //  case 's':
        //  int new_occupancy;
        //  new_occupancy = data[1] - '0';
        //  occupancy = new_occupancy;
        //  while(control !=0){
        //      Wire.beginTransmission(pi_address);
        //      Wire.write("ack");
        //      Wire.write('\0');
        //      control= Wire.endTransmission(true);
        //    }
        //  break;
        //
        //  //ANSWER RASPBERRY PI
        //  char send_desk;
        //  send_desk = desk_number + '0';
        //  char send_occupancy;
        //  char temp[6];
        //  case 'c': //send stream
        //    if(data[1]== 'l'){
        //      if (stream ==0 || stream ==1)
        //        stream = 1; //initialize lux streaming
        //     else
        //        stream = 3; //initialize both lux and duty cycle streaming
        //    }
        //    else if(data[1]== 'd'){
        //    if (stream ==0 || stream == 2)
        //        stream = 2; //streams duty cycle values only
        //     else
        //        stream = 3; //streams both lux and duty cycle
        //    }
        //    break;
        //
        //    case 'd':
        //    if(data[1]== 'l'){
        //      if (stream ==0 || stream ==1)
        //        stream = 0; //stops streaming
        //     else
        //        stream = 2; //streams duty cycle only
        //    }
        //    else if(data[1]== 'd'){
        //    if (stream ==0 || stream == 2)
        //        stream = 0; //stops all streaming
        //     else
        //        stream = 1; //streams lux only
        //    }
        //    break;
        //
        //    case 'g':
        //    if(data[1]== 'l'){ //get lux
        //    double lux_value = transform_ADC_in_lux(analogRead(LDRPin));
        //    dtostrf(lux_value,4,2,temp);
        //    while(control !=0){
        //      Wire.beginTransmission(pi_address);
        //      Wire.write("l");
        //      Wire.write(send_desk);
        //      Wire.write(temp, sizeof(temp)); //sends 6 bytes: size of array
        //      Wire.write('\0');
        //      control= Wire.endTransmission(true);
        //    }
        //    control = 1;
        //    }
        //    else if(data[1]== 'o'){
        //      send_occupancy = occupancy + '0';
        //      while(control !=0){
        //      Wire.beginTransmission(pi_address);
        //      Wire.write("o");
        //      Wire.write(send_desk);
        //      Wire.write(send_occupancy); //sends 6 bytes: size of array
        //      Wire.write('\0');
        //      control= Wire.endTransmission(true);
        //    }
        //    control = 1;
        //    }
        //
        //    break;
        //
        //    case 't': //test if pi is receiving
        //      Wire.beginTransmission(pi_address);
        //      Wire.write("OK\0");
        //      control= Wire.endTransmission(true);
        //    break;
        //
        // //ANSWER ARDUINO
        // case 'E':
        //    elements[found_elements].endereco= data[1];
        //    found_elements++;
        //    t_0=micros();
        //    break;
        //
        //  case 'C':
        //    if (data[1]=='S'){
        //      recolhe_valores=1;
        //    }
        //    if (data[1]=='E'){
        //      recolhe_valores=0;
        //      calibre_count++;
        //    }
        //    break;
        //
        //  case 'A':
        //    acende = true;
        //    break;

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

// --------------------------- Setup / Loop --------------------------------- //

void setup() {

    // Initialize Serial / I2C comunications
    Serial.begin(9600);
    Wire.begin(address);
    Wire.onReceive(receiveEvent);
    bitSet(TWAR, TWGCE);            //Enable general call
    delay(100);

    // System / cost function variables initialization
    // for(int i=0; i<found_elements; i++){
    //     Lmin[i] = 160;
    //     o[i] = 0;
    //     c[i] = 1;               // Unit cost, equal for all luminaires
    //     for(int j=0; j<found_elements; j++){
    //         if(i==j){
    //             Q[i][j] = 1;    // Unit cost, equal for all luminaires
    //         }
    //         else{
    //             Q[i][j] = 0;    // Unit cost, equal for all luminaires
    //         }
    //     }
    // }

    // ADMM auxiliary variables initialization
    for(int i=0; i<found_elements; i++){
        y[i] = 0;
        d[i] = 0;
        d_av[i] = 0;
        for(int j=0; j<found_elements; j++){
            d_copies[i][j] = 0;
        }
    }

}

void loop() {
    int iterations = 5;
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

    while(1);
}
