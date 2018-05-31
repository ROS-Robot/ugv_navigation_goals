#pragma once

#include <stack>
#include <time.h>
#include <stdlib.h>

/* useful definitions */

#define GENERATION_SIZE 4
#define NUM_OF_BEST_FIT 2
#define MAX_GENERATIONS 10      // maximum number of generations of individuals to examine
#define MAX_STAGNATED_GENS 4    // maximum number of acceptable generations with stagnated performance
#define STAGNATION_RATE 0.05     // if difference <= 5% then we have a generation with stagnated performance

/* Evolutionary algorithm core functions */

/* Check if termination criteria have been met */
bool terminationCriteriaMet(std::vector< std::vector<Waypoint> > & individuals, std::vector<double> & individuals_fitness, std::deque<double> & best_generations, double curr_generation);
/* Check if our goal has been achieved */
bool goalAchieved(std::vector< std::vector<Waypoint> > & individuals);