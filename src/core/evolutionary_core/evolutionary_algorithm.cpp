#include "../../../include/header.hpp"

/* Check if termination criteria have been met */
bool terminationCriteriaMet(std::vector< std::vector<Waypoint> > & individuals, std::vector<double> & individuals_fitness, std::deque<double> & best_generations, double curr_generation) {
    /* if maximum number of generations have been reached, then it is over */
    if (curr_generation >= MAX_GENERATIONS)
        return true;

    /* if performance is stagnating then it is not over */
    double diff = -1, prev_diff = -1;
    int stagnated_gens = 0;
    if (best_generations.size() == 1) {
        diff = 0;
        prev_diff = 0;
    }
    else {
        diff = std::abs(best_generations.at(0) - best_generations.at(1));
        if (best_generations.size() > 2) {
            for (int i = 0; i < best_generations.size()-1; i++) {
                prev_diff = diff;
                diff = std::abs(best_generations.at(i) - best_generations.at(i+1));
                if (std::abs(prev_diff - diff) <= diff*STAGNATION_RATE) {
                    stagnated_gens++;
                    if (stagnated_gens == MAX_STAGNATED_GENS)
                        return true;    // threshold reached                    
                }
            }
        }
    }
    
    /* if we reached this far,
        then termination is up to whether we achieved our goal of surviving the obstacles or not */
    return goalAchieved(individuals);
}

/* Check if our goal has been achieved */
bool goalAchieved(std::vector< std::vector<Waypoint> > & individuals) {
    /* for debugging */
    assert(individuals.at(0).size() > 1);
    /* iterate the best individual (path) */
    for (std::vector<Waypoint>::const_iterator it = individuals.at(0).begin(); it != std::prev(individuals.at(0).end(), 1); it++)
        if (throughLethalObstacle(*it, *(std::next(it, 1))))
            return true;
    
    return false;
}