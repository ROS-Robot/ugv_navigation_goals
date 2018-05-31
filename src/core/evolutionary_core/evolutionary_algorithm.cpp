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

/* Apply the (2-point) crossover operator between two individuals-paths */
void crossover(std::vector<Waypoint> & path_a, std::vector<Waypoint> & path_b, std::vector<Waypoint> & offspring_a, std::vector<Waypoint> & offspring_b) {
    /* for debugging */
    assert(path_a.size() == path_b.size());
    /* initialize pseudorandom numbers generator */
    srand(time(NULL));
    int first = rand() % path_a.size(), second = rand() % path_a.size();
    /* make sure that the crossover points are in a proper order */
    if (first > second) {
        int temp = second;
        second = first;
        first = temp;
    }
    /* do the crossover */
    for (int i = 0; i < path_a.size(); i++) {
        if (i < first || i > second) {
            offspring_a.push_back(path_a.at(i));
            offspring_b.push_back(path_b.at(i));
        }
        else {
            offspring_a.push_back(path_b.at(i));
            offspring_b.push_back(path_a.at(i));
        }
    }
}

/* Apply mutation to some individuals-paths */
void mutation(std::vector< std::vector<Waypoint> > & offsprings) {
    /* initialize pseudorandom number generator */
    srand(time(NULL));
    /* we don't want to mutate the same individual twice, as this may limit the bio-diversity of the next generation */
    std::set<int> visited;
    for (int i = 0; i < NUM_OF_MUTATIONS; i++) {
        /* select a random individual-path */
        int path;
        do {
            path = rand() % offsprings.size();
        } while (visited.find(path) != visited.end());
        visited.insert(path);
        /* select a random chromosome-waypoint of the selected individual-path */
        int chromosome = rand() % offsprings.at(path).size();
        /* do the mutation, essentially alter it's y coordinate randomly */
        /* we can't have a float-type pseudorandom, so we will first "slice" our y values boundary in a random position */
        int slice = rand() % (int) std::abs(terrain.goal_left.position.y - terrain.goal_right.position.y);
        int new_y = terrain.goal_right.position.y + slice;
        offsprings.at(path).at(chromosome).pose.pose.position.y = new_y;
    }
}