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

// ---------------------- Reused functions ------------------------------ //


// ------------------------ My functions -------------------------------- //

float local_lux(float (&K) [N_ELEMENTS][N_ELEMENTS],
                float (&d) [N_ELEMENTS],
                float (&o) [N_ELEMENTS],
                arduino_info (&elements) [N_ELEMENTS]){
    float L = o[my_index];    // unconstrained solution illuminance at local desk
    for(int i=0; i<N_ELEMENTS; i++){
        L += (elements[i].ganho)*d[i];
    }
    return L;
}

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

    float L_unconstrained = local_lux(K,d_unconstrained,o,elements);
    if ((d_unconstrained[my_index]>=0) &&
        (d_unconstrained[my_index]<=100) &&
        (L_unconstrained>=Lref[my_index]-o[my_index]) ){
        for(int i=0; i<N_ELEMENTS; i++){
            d[i] = d_unconstrained[i];    // The unconstrained solution is the
            return;                       // optimal solution - all done!
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
            d_linear[i] = z[i]/rho - (elements[i].ganho)*(o[my_index]-Lref[my_index]-w1)/(n*rho);
        }
        else if(i==my_index){
            d_linear[i] = z[i]/(Q[i]+rho) - (elements[i].ganho)*(o[my_index]-Lref[my_index]-w1)/(n*(Q[i]+rho));
        }
	}

    // ----- Check solution feasibility / get cost function value ----------- //

    float L_linear = local_lux(K,d_linear,o,elements);
    if ((d_linear[my_index]>=0) &&
        (d_linear[my_index]<=100)){
        f_linear = cost_function(d_linear, c, Q, y, d_av, rho);
        if(f_linear<f_best){
            f_best = f_linear;
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
    //
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    //     2.2) Compute best solution on the boundary - dcmin constraint
    // ---------------------------------------------------------------------- //

    float d_dcmin[N_ELEMENTS];
    for(int i=0; i<N_ELEMENTS; i++){
        if(i!=my_index){
            d_dcmin[i] = z[i]/rho;
        }
        else if(i==my_index){
            d_dcmin[i] = 0;
        }
	}

    // ----- Check solution feasibility / get cost function value ----------- //

    float L_dcmin = local_lux(K,d_dcmin,o,elements);
    if ((d_dcmin[my_index]<=100) &&
        (L_dcmin>=Lref[my_index]-o[my_index]) ){
        f_dcmin = cost_function(d_dcmin, c, Q, y, d_av, rho);
        if(f_dcmin<f_best){
            f_best = f_dcmin;
            for(int i=0; i<N_ELEMENTS; i++){
                d_best[i] = d_dcmin[i];
            }
        }
    }

    // -------------------------  DEBUG (dcmin)  ---------------------------- //
    //
    f_dcmin = cost_function(d_dcmin, c, Q, y, d_av, rho);
    for(int i=0; i<N_ELEMENTS; i++){
        Serial.println(d_dcmin[i]);
    }
    Serial.println(f_dcmin);
    Serial.println(" ");
    //
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    //     2.3) Compute best solution on the boundary - dcmax constraint
    // ---------------------------------------------------------------------- //

    float d_dcmax[N_ELEMENTS];
    for(int i=0; i<N_ELEMENTS; i++){
        if(i!=my_index){
            d_dcmax[i] = z[i]/rho;
        }
        else if(i==my_index){
            d_dcmax[i] = 100;
        }
	}

    // ------------------- Check solution feasibility ----------------------- //

    float L_dcmax = local_lux(K,d_dcmax,o,elements);
    if ((d_dcmax[my_index]>=0) &&
        (L_dcmax>=Lref[my_index]-o[my_index]) ){
        f_dcmax = cost_function(d_dcmax, c, Q, y, d_av, rho);
        if(f_dcmax<f_best){
            f_best = f_dcmax;
            for(int i=0; i<N_ELEMENTS; i++){
                d_best[i] = d_dcmax[i];
            }
        }
    }

    // -------------------------  DEBUG (dcmax)  ---------------------------- //
    //
    f_dcmax = cost_function(d_dcmax, c, Q, y, d_av, rho);
    for(int i=0; i<N_ELEMENTS; i++){
        Serial.println(d_dcmax[i]);
    }
    Serial.println(f_dcmax);
    Serial.println(" ");
    //
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    // 2.4) Compute best solution on the boundary: linear and dcmin constraints
    // ---------------------------------------------------------------------- //

    float g = (rho+Q[my_index]) / (n*(rho+Q[my_index]) - (elements[my_index].ganho)*(elements[my_index].ganho));
    float v = 0;
    float kr2 = 0;
    for(int i=1; i<N_ELEMENTS; i++){
        if(i==my_index){
            continue;
        }
        else{
            v   += z[i]*elements[i].ganho;
            kr2 += (elements[i].ganho)*(elements[i].ganho);
        }
    }

    float d_linear_dcmin[N_ELEMENTS];
    for(int i=0; i<N_ELEMENTS; i++){
        if(i!=my_index){
            d_linear_dcmin[i] = z[i]/rho + g*((elements[i].ganho)*(Lref[my_index]-o[my_index]-v/rho))/rho;
        }
        else if(i==my_index){
            d_linear_dcmin[i] = z[i]/(Q[i]+rho) - g*(kr2*z[i]/(Q[i]+rho))/rho; // <----- BUG: solved, wrong sign in the handout
        }
	}

    // ------------------- Check solution feasibility ----------------------- //

    float L_linear_dcmin = local_lux(K,d_linear_dcmin,o,elements);
    if (d_linear_dcmin[my_index]<=100){
        f_linear_dcmin = cost_function(d_linear_dcmin, c, Q, y, d_av, rho);
        if(f_linear_dcmin<f_best){
            f_best = f_linear_dcmin;
            for(int i=0; i<N_ELEMENTS; i++){
                d_best[i] = d_linear_dcmin[i];
            }
        }
    }

    // ----------------------- DEBUG (linear, dcmin)  ----------------------- //
    //
    f_linear_dcmin = cost_function(d_linear_dcmin, c, Q, y, d_av, rho);
    for(int i=0; i<N_ELEMENTS; i++){
        Serial.println(d_linear_dcmin[i]);
    }
    Serial.println(f_linear_dcmin);
    Serial.println(" ");
    //
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    // 2.5) Compute best solution on the boundary: linear and dcmax constraints
    // ---------------------------------------------------------------------- //

    float d_linear_dcmax[N_ELEMENTS];
    for(int i=0; i<N_ELEMENTS; i++){
        if(i!=my_index){
            d_linear_dcmax[i] = d_linear_dcmin[i] - 100*(elements[i].ganho)*(elements[my_index].ganho)*g/rho;
        }
        else if(i==my_index){
            d_linear_dcmax[i] = d_linear_dcmin[i] + 100*kr2;
        }
	}

    // ------------------- Check solution feasibility ----------------------- //

    float L_linear_dcmax = local_lux(K,d_linear_dcmax,o,elements);
    if (d_linear_dcmax[my_index]>=0){
        f_linear_dcmax = cost_function(d_linear_dcmax, c, Q, y, d_av, rho);
        if(f_linear_dcmax<f_best){
            f_best = f_linear_dcmax;
            for(int i=0; i<N_ELEMENTS; i++){
                d_best[i] = d_linear_dcmax[i];
            }
        }
    }

    // ----------------------- DEBUG (linear, dcmax)  ----------------------- //
    //
    f_linear_dcmax = cost_function(d_linear_dcmax, c, Q, y, d_av, rho);
    for(int i=0; i<N_ELEMENTS; i++){
        Serial.println(d_linear_dcmax[i]);
    }
    Serial.println(f_linear_dcmax);
    Serial.println(" ");
    //
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    //              2.6) Retrieve best solution on the boundary
    // ---------------------------------------------------------------------- //

    for(int i=0; i<N_ELEMENTS; i++){
        d[i] = d_best[i];
    }

    // Serial.println(f_best);
    return;
}

void compute_av(float (&d) [N_ELEMENTS],
                float (&d_av) [N_ELEMENTS],
                float (&d_copies) [N_ELEMENTS][N_ELEMENTS]){
    for(int j=0; j<N_ELEMENTS; j++){
        d_av[j] = 0;
        for(int i=0; i<N_ELEMENTS; i++){
            if(i!=my_index){
                d_av[j] += d_copies[i][j];
            }
            else if(i==my_index){
                d_av[j] += d[j];
            }
        }
        d_av[j] /= N_ELEMENTS;
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

void update_y(float (&y) [N_ELEMENTS],
              float rho,
              float (&d) [N_ELEMENTS],
              float (&d_av) [N_ELEMENTS]){
    for(int i=0; i<N_ELEMENTS; i++){
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

// ------------------------ Setup / Loop --------------------------------- //

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
        d[i] = 0;
        d_av[i] = 0;
        for(int j=0; j<N_ELEMENTS; j++){
            d_copies[i][j] = 0;
        }
    }
    int iterations = 50;
    //for(int i=0; i<iterations; i++){
        // compute duty-cycles vector, d
        feedforwardConsensus(K, Lref, o, d, c, Q, y, d_av, rho, elements);        // compute local duty cycle vector

        // compute average duty-cycles vector, d_av
        compute_av(d, d_av, d_copies);

        // update local lagrangian, y
        update_y(y, rho, d, d_av);

    //    }
        // broadcast / receive computed 'd' vectors

        // Serial.println("-----------------------------------");
        // Serial.println(" ");


    Serial.println("Done.");
    while(1){}
}
