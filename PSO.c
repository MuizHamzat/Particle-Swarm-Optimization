// CODE: include library(s)
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include "OF_lib.h"
#include "utility.h"

// Helper function to generate random numbers in a range
double random_double(double min, double max) {
    return min + (max - min) * ((double)rand() / RAND_MAX);
}


double pso(ObjectiveFunction objective_function, int NUM_VARIABLES, Bound *bounds, int NUM_PARTICLES, int MAX_ITERATIONS, double *best_position) {

    double w_max = 0.9;
    double w_min = 0.4;
    double c1 = 2.0;
    double c2 = 2.0;
    double initial_vmax = 0.1 * (bounds->upperBound - bounds->lowerBound);

    //Allocate memory for number of particles
    double** x = malloc(sizeof(double*) * (long unsigned int)NUM_PARTICLES);
    double** v = malloc(sizeof(double*) * (long unsigned int)NUM_PARTICLES);    
    double** p = malloc(sizeof(double*) * (long unsigned int)NUM_PARTICLES);
    double* fpBest = malloc(sizeof(double) * (long unsigned int)NUM_PARTICLES);
    double* g = malloc(sizeof(double) * (long unsigned int)NUM_VARIABLES);

    //Check if memory allocation failed
    if (!x || !v || !p || !fpBest || !g){
                printf("Memory allocation failed. Exiting program.");
                exit(1);
            }
    
    //Allocate memory for number of variables for each particle
    int i,j;
    for (i=0; i < NUM_PARTICLES; i++){
        x[i] = malloc(sizeof(double) * (long unsigned int)NUM_VARIABLES);        
        v[i] = malloc(sizeof(double) * (long unsigned int)NUM_VARIABLES);
        p[i] = malloc(sizeof(double) * (long unsigned int)NUM_VARIABLES);

        //Check if memory allocation failed
        if (!x[i] || !v[i] || !p[i]){
            printf("Memory allocation failed. Exiting program.");
            exit(1);
        }
    }

    //Set best global fitness value to a very large number initially. That way, whatever the first personal fitness value found is, it will become the global fitness value
    double fgBest = DBL_MAX;

    //INITIALIZATION
    //Give particles random initial positions, velocities, and personal fitness values
    for(i=0; i < NUM_PARTICLES; i++){
        for (j=0; j < NUM_VARIABLES; j++){
            x[i][j] = random_double(bounds[j].lowerBound, bounds[j].upperBound);
            v[i][j] = random_double(-1.0, 1.0);
            p[i][j] = x[i][j];
        }
        //Find the personal best fitness value of each particle. If it's better than the global best fitness value, replace it, and the best global position
        fpBest[i] = objective_function(NUM_VARIABLES, x[i]);
        if (fpBest[i] < fgBest){
            fgBest = fpBest[i];
            for (j=0; j < NUM_VARIABLES; j++){
                g[j] = p[i][j];
            }
        }
    }

    //PSO Loop
    int iter;
    int stagnationCounter[NUM_PARTICLES];
    for (iter=0; iter < MAX_ITERATIONS; iter++){
        // Dynamic inertia weight
        double w = w_max - ((w_max - w_min) * iter / MAX_ITERATIONS);

        if (iter % 100 == 0){
            printf("Iteration #%i: OF = %lf\nVariables: { ", iter, fgBest);
            for (int j = 0; j < NUM_VARIABLES; j++) {
                printf("%lf ", g[j]);
            }
            printf("}\n\n");
        }

        // Adaptive vmax adjustment
        double avg_fitness = 0.0;
        for (i = 0; i < NUM_PARTICLES; i++) {
            avg_fitness += fpBest[i];
        }
        avg_fitness /= NUM_PARTICLES;
        double vmax = initial_vmax * (1.0 - (fgBest / avg_fitness));

        for (i=0; i < NUM_PARTICLES; i++){
            for (j=0; j < NUM_VARIABLES; j++){
                double r1 = random_double(0,1);
                double r2 = random_double(0,1);
                //Update velocity
                v[i][j] = w *v[i][j]+c1*r1*(p[i][j]-x[i][j])+c2*r2*(g[j]-x[i][j]);

                // Adaptive velocity clamping
                if (v[i][j] > vmax) {
                    v[i][j] = vmax;
                }
                if (v[i][j] < -vmax) {
                    v[i][j] = -vmax;
                }

                //Update position
                x[i][j] += v[i][j];
                //Clamp x[i][j] within bounds
                if (x[i][j] < bounds[j].lowerBound) {x[i][j] = bounds[j].lowerBound;}
                if (x[i][j] > bounds[j].upperBound) {x[i][j] = bounds[j].upperBound;}
            }

            //Calculate fitness value and compare it with particle's best fitness value. If the current fitness value is better, replace it and the particle's personal best position
            double f = objective_function(NUM_VARIABLES, x[i]);
            if (f < fpBest[i]){
                fpBest[i] = f;
                for (j=0; j < NUM_VARIABLES; j++){
                    p[i][j] = x[i][j];
                }
                stagnationCounter[i] = 0;
            } else{
                stagnationCounter[i]++;
            }

            //The same thing but for the global best fitness value and position
            if (f < fgBest){
                fgBest = f;
                for (j=0; j < NUM_VARIABLES; j++){
                    g[j] = x[i][j];
                }
            }
        }

        // Adaptive parameter adjustment based on performance
        for (i = 0; i < NUM_PARTICLES; i++) {
            if (stagnationCounter[i] > 50) {
                // Increase cognitive component to encourage exploration
                c1 = 3.0;
                c2 = 1.0;
            } else {
                // Reset to default values
                c1 = 2.0;
                c2 = 2.0;
            }
        }

        // Check for convergence
        int convergence = 1;
        for (i = 0; i < NUM_PARTICLES; i++) {
            if (stagnationCounter[i] < 100) {
                convergence = 0;
                break;
            }
        }
        if (convergence) {
            printf("Converged at iteration #%i\n", iter + 1);
            break;
        }
    }

    //AFTER PSO LOOP
    //Save the global best position to the best_position paramter 
    for (j=0; j < NUM_VARIABLES; j++){
        best_position[j] = g[j];
    }

    //Deallocating
    for (i=0; i < NUM_PARTICLES; i++){
        free(x[i]);
        free(v[i]);
        free(p[i]);
    }

    free(x);
    free(v);
    free(p);
    free(fpBest);
    free(g);

    return fgBest;
}

//q: I can't seem to get the global minimum of 0 for the rastrigin function. I've tried dynamically adjusting the inertia weight and vmax, but it doesn't seem to help. Increasing the number of iterations doesn't seem to help either. What can I do to improve the performance of the PSO algorithm?
//a: The Rastrigin function is a multimodal function with many local minima. The PSO algorithm is not guaranteed to find the global minimum for such functions due to its stochastic nature. To improve the performance of the PSO algorithm for the Rastrigin function, you can try the following strategies:
//1. Increase the number of particles: Increasing the number of particles can help the algorithm explore the search space more effectively and increase the chances of finding the global minimum.
//2. Tune the algorithm parameters: Experiment with different values for the inertia weight, cognitive and social parameters, and velocity clamping to find the optimal settings for the Rastrigin function.
//3. Implement adaptive parameter adjustment: Implement adaptive mechanisms to adjust the algorithm parameters dynamically during the optimization process based on the performance of the algorithm.
//4. Use a different optimization algorithm: If the PSO algorithm is not performing well for the Rastrigin function, consider using other optimization algorithms such as Genetic Algorithms, Differential Evolution, or Simulated Annealing.
//5. Run multiple trials: Run the PSO algorithm multiple times with different random initializations to increase the chances of finding the global minimum of the Rastrigin function.
//6. Implement local search: Combine the PSO algorithm with a local search method to refine the solutions found by the algorithm and improve the convergence to the global minimum.
