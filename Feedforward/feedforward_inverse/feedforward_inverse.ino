
#include <MatrixMath.h>

const int N_ELEMENTS = 2;

int dim_d[N_ELEMENTS] = {N_ELEMENTS, 1};
int dim_o[N_ELEMENTS] = {N_ELEMENTS, 1};
int dim_Lref[N_ELEMENTS] = {N_ELEMENTS, 1};
int dim_K[N_ELEMENTS] = {N_ELEMENTS, N_ELEMENTS};

float d[N_ELEMENTS] = {};
float o[N_ELEMENTS] = {1,1};                           
float Lref[N_ELEMENTS] = {160, 160};
float K[N_ELEMENTS][N_ELEMENTS] = {{2, 1}, {1, 2}};

void feedforwardInvMat(float (&d) [N_ELEMENTS], 
                      const float (&Lref) [N_ELEMENTS],
                      const float (&o)    [N_ELEMENTS], 
                      const float (&K)    [N_ELEMENTS][N_ELEMENTS]){

    float aux   [N_ELEMENTS][N_ELEMENTS] = {{},{}};
    float K_inv [N_ELEMENTS][N_ELEMENTS] = {{},{}};   

    Matrix.Copy((float*) K, N_ELEMENTS, N_ELEMENTS, (float*) K_inv);
    Matrix.Invert((float*) K_inv, N_ELEMENTS);

    Matrix.Subtract((float*) Lref, (float*) o, N_ELEMENTS, N_ELEMENTS, (float*) aux);

    Matrix.Multiply((float*) K_inv, (float*) aux, N_ELEMENTS, N_ELEMENTS, 1, (float*) d);

    return;
}

void setup() {
    Serial.begin(9600);
         
    feedforwardInvMat(d, Lref, o, K);
    
    Matrix.Print((float*) o, dim_o[0], dim_o[1], "o:");
    Matrix.Print((float*) Lref, dim_Lref[0], dim_Lref[1], "Lref:");
    Matrix.Print((float*) K, dim_K[0], dim_K[1], "K:");    
    Serial.print("\n------------------------------\n");
    Matrix.Print((float*) d, dim_d[0], dim_d[1], "d:");
}

void loop() {
    // put your main code here, to run repeatedly:
}
