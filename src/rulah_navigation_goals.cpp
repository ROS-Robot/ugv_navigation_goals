#include "../include/header.hpp"

// #define TEST_BEZIER
// #define TEST_CALCULATIONS
// #define GENETIC_ALGORITHM_GENERATION
#define N_BEST_GENERATION

#ifdef TEST_BEZIER
/* Test Bezier curve's core functions */

int main(int argc, char *argv[]) {
    bezierTest(argc, argv);
    return 0;
}

#elif defined( TEST_CALCULATIONS )
/* Test calculations core functions */

int main(int argc, char *argv[]) {
    calculationsTest(argc, argv);
    return 0;
}

#elif defined( GENETIC_ALGORITHM_GENERATION )
/* A Genetic-algorithm based waypoint generation implementation */

int main(int argc, char *argv[]) {
    geneticAlgorithmGenerator(argc, argv);
    return 0;
}

#elif defined( N_BEST_GENERATION )
/* An N-best based waypoint generation implementation */

int main(int argc, char *argv[]) {
    nBestGenerator(argc, argv);
    return 0;
}

#else
/* A Hill-climbing based waypoint generation implementation */

int main(int argc, char *argv[]) {
    hillClimbingGenerator(argc, argv);
    return 0;
}

#endif
