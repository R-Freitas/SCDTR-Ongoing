// COMANDOS QUE PODEM DAR JEITO:
// float aux   [N_ELEMENTS][N_ELEMENTS] = {{},{}};
// float K_inv [N_ELEMENTS][N_ELEMENTS] = {{},{}};
// Matrix.Copy((float*) K, N_ELEMENTS, N_ELEMENTS, (float*) K_inv);
// Matrix.Invert((float*) K_inv, N_ELEMENTS);
// Matrix.Subtract((float*) Lref, (float*) o, N_ELEMENTS, N_ELEMENTS, (float*) aux);
// Matrix.Multiply((float*) K_inv, (float*) aux, N_ELEMENTS, N_ELEMENTS, 1, (float*) d);
// Matrix.Print((float*) K, N_ELEMENTS, N_ELEMENTS, "K:");

#include <MatrixMath.h>

// ----------------------- Cenas locais, teste -------------------------- //

const int N_ELEMENTS = 2;
const int my_index = 0;

struct arduino_info {int endereco; double ganho;};
arduino_info elements[N_ELEMENTS] = {{0, 2}, {1, 1}};

// ---------------------------------------------------------------------- //

// float lux_local(float (&K) [N_ELEMENTS][N_ELEMENTS],
//                     float (&d) [N_ELEMENTS],
//                     float (&o) [N_ELEMENTS],
//                     arduino_info (&elements) [N_ELEMENTS]){
// }

float cost_function(float (&d) [N_ELEMENTS],
                    float (&c) [N_ELEMENTS],
                    float (&Q) [N_ELEMENTS],
                    float (&y) [N_ELEMENTS],
                    float (&d_av) [N_ELEMENTS],
                    float &rho){
    float f = 0;
    f = 0.5*Q[my_index]*d[my_index]*d[my_index]
      + c[my_index]*d[my_index];
    for(int i=0; i<N_ELEMENTS; i++){
        f += y[i]*(d[i]-d_av[i])
           + 0.5*rho*(d[i]-d_av[i])*(d[i]-d_av[i]);
    }
    return f;
}

void feedforwardConsensus(float (&K) [N_ELEMENTS][N_ELEMENTS],
                          float (&Lref) [N_ELEMENTS],
                          float (&o) [N_ELEMENTS],
                          float (&d) [N_ELEMENTS],
                          float (&c) [N_ELEMENTS],
                          float (&Q) [N_ELEMENTS],
                          float (&y) [N_ELEMENTS],
                          float (&d_av) [N_ELEMENTS],
                          float &rho,
                          arduino_info (&elements) [N_ELEMENTS]){

    // ###################################################################### //    <-- Code between these lines has been tested, adn is working
    // ---------------------------------------------------------------------- //
    //                  1) Compute unconstrained solution
    // ---------------------------------------------------------------------- //

	float z[N_ELEMENTS];
    float d_unconstrained[N_ELEMENTS];
    for(int i=0; i<N_ELEMENTS; i++){
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

    float L_unconstrained = o[my_index];    // unconstrained solution illuminance at local desk
    for(int i=0; i<N_ELEMENTS; i++){
        L_unconstrained += (elements[i].ganho)*d_unconstrained[i];
    }
    if ((d_unconstrained[my_index]>=0) &&
        (d_unconstrained[my_index]<=255) &&
        (L_unconstrained>=Lref[my_index]-o[my_index]) ){
        for(int i=0; i<N_ELEMENTS; i++){
            d[i] = d_unconstrained[i];
            return;             // The unconstrained solution is the optimal solution - all done.
        }
    }

    // ---------------------- DEBUG (unconstrained)  ------------------------ //
    //
    float f_unconstrained = cost_function(d_unconstrained, c, Q, y, d_av, rho);
    for(int i=0; i<N_ELEMENTS; i++){
        Serial.println(d_unconstrained[i]);
    }
    Serial.println(f_unconstrained);
    Serial.println(" ");

    // ---------------------------------------------------------------------- //

    // ###################################################################### //    <-- Code between these lines has been tested, and is working

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
    float d_best[N_ELEMENTS];

    // ---------------------------------------------------------------------- //
    //    2.1) Compute best solution on the boundary - Linear constraint
    // ---------------------------------------------------------------------- //

    float n = 0;
    float w1 = 0;
    for(int i=0; i<N_ELEMENTS; i++){
        if(i!=my_index){
            n += (elements[i].ganho)*(elements[i].ganho)/rho;
            w1 -= (elements[i].ganho)*z[i]/rho;
        }
        else if(i==my_index){
            n += (elements[i].ganho)*(elements[i].ganho)/(Q[i]+rho);
            w1 -= (elements[i].ganho)*z[i]/(Q[i]+rho);
        }
    }

    float d_linear[N_ELEMENTS];
    for(int i=0; i<N_ELEMENTS; i++){
        if(i!=my_index){
            d_linear[i] = (z[i]/rho) - (elements[i].ganho)*(o[i]-Lref[i]-w1)/(n*rho);
        }
        else if(i==my_index){
            d_linear[i] = (z[i]/(Q[i]+rho)) - (elements[i].ganho)*(o[i]-Lref[i]-w1)/(n*(Q[i]+rho));
        }
	}

    // ----- Check solution feasibility / get cost function value ----------- //


    float L_linear = o[my_index];    // linear solution illuminance at local desk
    for(int i=0; i<N_ELEMENTS; i++){
        L_linear += (elements[i].ganho)*d_unconstrained[i];
    }
    if ((d_linear[my_index]>=0) && (d_linear[my_index]<=255)){
        f_linear = cost_function(d_linear, c, Q, y, d_av, rho);
        if(f_linear<f_best){
            for(int i=0; i<N_ELEMENTS; i++){
                d_best[i] = d_linear[i];
            }
        }
    }

    // ------------------------  DEBUG (linear)  ---------------------------- //
    //
    f_linear = cost_function(d_linear, c, Q, y, d_av, rho);
    for(int i=0; i<N_ELEMENTS; i++){
        Serial.println(d_linear[i]);
    }
    Serial.println(f_linear);
    Serial.println(" ");

    return;
}

void setup() {
    Serial.begin(9600);
 }

void loop() {

    // System definition
    static float K[N_ELEMENTS][N_ELEMENTS] = {{2,1},{1,2}};
    static float Lref[N_ELEMENTS] = {150,80};
    static float o[N_ELEMENTS] = {30,0};

    // Cost function definition
    static float c[N_ELEMENTS] = {1,1};
    static float Q[N_ELEMENTS] = {1,1};
    // for(int i=0; i<N_ELEMENTS; i++){
    //     Lref[i] = 160;
    //     o[i] = 0;
    //     c[i] = 1;               // Unit cost, equal for all luminaires
    //     for(int j=0; j<N_ELEMENTS; j++){
    //         if(i==j){
    //             Q[i][j] = 1;    // Unit cost, equal for all luminaires
    //         }
    //         else{
    //             Q[i][j] = 0;    // Unit cost, equal for all luminaires
    //         }
    //     }
    // }

    // ADMM auxiliary variables
    static float rho = 0.01;                       // ADMM penalty parameter
    static float y[N_ELEMENTS];                    // Lagrange multipliers vector
    static float d[N_ELEMENTS];                    // Final (optimal) duty cycles, for local arduino
    static float d_av[N_ELEMENTS];                 // Duty cycles average, for all arduinos
    static float d_copies[N_ELEMENTS][N_ELEMENTS]; // Duty cycles buffer, for all arduinos
    for(int i=0; i<N_ELEMENTS; i++){
        y[i] = 0;
        d[i] = -1;
        d_av[i] = 0;
        for(int j=0; j<N_ELEMENTS; j++){
            d_copies[i][j] = 0;
        }
    }
    int iterations = 50;
    for(int i=0; i<iterations; i++){
        feedforwardConsensus(K, Lref, o, d, c, Q, y, d_av, rho, elements);        // compute local duty cycle vector
        // compute average, d_av
        // update local lagrangian, y
        // broadcast / receive computed 'd' vectors
    }

    Serial.print("Done.");
    while(1){}
}
