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

struct arduino_info {
    int endereco;
    double ganho;
};
arduino_info elements[N_ELEMENTS] = {{0, 2}, {1, 1}};

// ---------------------------------------------------------------------- //

// System definition
float K[N_ELEMENTS][N_ELEMENTS] = {{2,1},{1,2}};
float Lref[N_ELEMENTS] = {150,80};
float o[N_ELEMENTS] = {30,0};

// Cost function definition
float c[N_ELEMENTS] = {1,1};
float Q[N_ELEMENTS] = {0,0};
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

void feedforwardConsensus(float (&d) [N_ELEMENTS],
                          float (&c) [N_ELEMENTS],
                          float (&Q) [N_ELEMENTS],
                          float (&y) [N_ELEMENTS],
                          float (&d_av) [N_ELEMENTS],
                          arduino_info (&elements) [N_ELEMENTS],
                          float &rho){

    // ###################################################################### //    <-- Code between these lines has been tested, adn is working
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

	// -----------------------------  DEBUG  -------------------------------- //
		// float f;
		// f = 0.5*Q[my_index]*d_unconstrained[my_index]*d_unconstrained[my_index]
		//   + c[my_index]*d_unconstrained[my_index];
		// for(int j=0; j<N_ELEMENTS; j+=2){
		// 	for(int i=0; i<N_ELEMENTS; i++){
		// 		f += y[i]*(d_unconstrained[i]-d_av[i])
		// 		   + 0.5*rho*(d_unconstrained[i]-d_av[i])*(d_unconstrained[i]-d_av[i]);
		// 		Serial.println(d_unconstrained[j]);
		// 		Serial.println(d_unconstrained[j+1]);
		// 		Serial.println(f);
		// 		Serial.println(" ");
		// 	}
	   // }
    // ---------------------------------------------------------------------- //

    // ------------------- Check solution feasibility ----------------------- //

    float L_unconstrained = o[my_index];    // unconstrained illuminance at local desk
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

    // ###################################################################### //    <-- Code between these lines has been tested, adn is working

    // ---------------------------------------------------------------------- //
    //                  2) Compute best solution on the boundary
    // ---------------------------------------------------------------------- //

    // Boolean flags for solution satisfiability
    int sol_dcmin;
    int sol_dcmax;
    int sol_linear;
    int sol_linear_dcmin;
    int sol_linear_dcmax;

    //Current best cost function / duty cycle values
	float f;
    float f_best = 100000;
    float d_best[N_ELEMENTS];

    //Local cost function computation
	f = Q[my_index]*d_best[my_index]*d_best[my_index]
	  + c[my_index]*d_best[my_index];
	for(int i=0; i<N_ELEMENTS; i++){
        f += y[i]*(d_best[i]-d_av[i])
           + 0.5*rho*(d_best[i]-d_av[i])*(d_best[i]-d_av[i]);
    }
    return;
}

void setup() {
    Serial.begin(9600);
 }

void loop() {
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
        feedforwardConsensus(d, c, Q, y, d_av, elements, rho);        // compute local duty cycle vector
        // compute average, d_av
        // update local lagrangian, y
        // broadcast / receive computed 'd' vectors
    }

    Serial.print("Done.");
    while(1){}
}
