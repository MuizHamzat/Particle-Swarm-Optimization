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
    //PSO Parameters
    double w = 0.7;
    double c1 = 1.5;
    double c2 = 1.5;

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
    int iter; //Iteration counter
    int stopper = 0; //Counter to stop the loop if the global best fitness value doesn't change for a certain number of iterations (stagnates)
    double prev_fgBest;
    int stagnationCounter[NUM_PARTICLES]; //Counter to keep track of how many iterations a particle's personal best fitness value hasn't changed
    for (iter=0; iter < MAX_ITERATIONS; iter++){
        prev_fgBest = fgBest;

        //Print out the global best fitness value and position every 1000 iterations
        if (iter % 1000 == 0){
            printf("Iteration #%i: OF = %lf\nVariables: { ", iter, fgBest);
            for (int j = 0; j < NUM_VARIABLES; j++) {
                printf("%lf ", g[j]);
            }
            printf("}\n\n");
        }

        //Update each particle's position and velocity
        for (i=0; i < NUM_PARTICLES; i++){
            for (j=0; j < NUM_VARIABLES; j++){
                double r1 = random_double(0,1);
                double r2 = random_double(0,1);
                //Update velocity
                v[i][j] = w *v[i][j]+c1*r1*(p[i][j]-x[i][j])+c2*r2*(g[j]-x[i][j]);
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
            if (stagnationCounter[i] > 100) {
                // Increase cognitive component to encourage exploration
                c1 = 2.0;
                c2 = 1.0;
            } else {
                // Reset to default values
                c1 = 1.5;
                c2 = 1.5;
            }
        }

        //Stopping condition: If the global best fitness value doesn't change for 500 iterations, stop the loop
        if (fabs(prev_fgBest-fgBest) < 1e-15) {
            stopper++;
        } else {
            stopper = 0;
        }

        if (stopper == 500) {
            printf("Stopped at iteration #%i\n", iter + 1);
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