// CODE: include library(s)
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "OF_lib.h"
#include "utility.h"

// Helper function to generate random numbers in a range
double random_double(double min, double max) {
    return min + (max - min) * ((double)rand() / RAND_MAX);
}

// CODE: implement other functions here if necessary

double pso(ObjectiveFunction objective_function, int NUM_VARIABLES, Bound *bounds, int NUM_PARTICLES, int MAX_ITERATIONS, double *best_position) {

    // CODE: implement pso function here
    float w = 0.7;
    float c1 = 1.5;
    float c2 = 1.5;

    double** x = malloc(sizeof(double*) * NUM_PARTICLES);
            if (x == NULL){
                printf("Memory allocation failed. Exiting program.");
                exit(1);
            }
    
    double** v = malloc(sizeof(double*) * NUM_PARTICLES);
            if (v == NULL){
                printf("Memory allocation failed. Exiting program.");
                exit(1);
            }
    
    double** p = malloc(sizeof(double*) * NUM_PARTICLES);
            if (p == NULL){
                printf("Memory allocation failed. Exiting program.");
                exit(1);
            }

    double* fpBest = malloc(sizeof(double) * NUM_PARTICLES);
            if (fpBest == NULL){
                printf("Memory allocation failed. Exiting program.");
                exit(1);
            }

    double* g = malloc(sizeof(double) * NUM_VARIABLES);
            if (fpBest == NULL){
                printf("Memory allocation failed. Exiting program.");
                exit(1);
            }
    
    int i,j;
    for (i=0; i < NUM_PARTICLES; i++){
        x[i] = malloc(sizeof(double) * NUM_VARIABLES);
        if (x[i] == NULL){
            printf("Memory allocation failed. Exiting program.");
            exit(1);
        }

        v[i] = malloc(sizeof(double) * NUM_VARIABLES);
        if (v[i] == NULL){
            printf("Memory allocation failed. Exiting program.");
            exit(1);
        }

        p[i] = malloc(sizeof(double) * NUM_VARIABLES);
        if (p[i] == NULL){
            printf("Memory allocation failed. Exiting program.");
            exit(1);
        }
    }

    double fgBest = INFINITY;

    for(i=0; i < NUM_PARTICLES; i++){
        for (j=0; j < NUM_VARIABLES; j++){
            x[i][j] = random_double(bounds->lowerBound, bounds->upperBound);
            v[i][j] = random_double(bounds->lowerBound, bounds->upperBound);
            p[i][j] = random_double(bounds->lowerBound, bounds->upperBound);
        }
        fpBest[i] = objective_function(NUM_VARIABLES, x[i]);
        if (fpBest[i] < fgBest){
            fgBest = fpBest[i];
            g = p[i];
        }
    }

    //PSO Loop
    int iter;
    for (iter=0; iter < MAX_ITERATIONS; iter++){
        for (i=0; i < NUM_PARTICLES; i++){
            for (j=0; j < NUM_VARIABLES; j++){
                double r1 = random_double(0,1);
                double r2 = random_double(0,1);
                v[i][j] = w *v[i][j]+c1*r1*(p[i][j]-x[i][j])+c2*r2*(g[j]-x[i][j]);
                x[i][j] = x[i][j]+v[i][j];
                //Clamp x[i][j] within bounds
                x[i][j] = fmax(bounds[j].lowerBound, fmin(bounds[j].upperBound, x[i][j]));
            }

            double f = objective_function(NUM_VARIABLES, x[i]);
            if (f < fpBest[i]){
                fpBest[i] = f;
                p[i] = x[i];
            }

            if (f < fgBest){
                fgBest = f;
                g = x[i];
            }
        }
        
    }

    for (j=0; j < NUM_VARIABLES; j++){
        best_position[j] = g[j];
    }

    for (i=0; i < NUM_PARTICLES; i++){
        free(x[i]);
        free(v[i]);
        //free(p[i]);
    }

    free(x);
    free(v);
    free(p);
    free(fpBest);
    free(g);

    return fgBest;
}