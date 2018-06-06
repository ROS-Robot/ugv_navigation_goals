#include "../../include/header.hpp"

// #define TEST_EVOLUTIONARY_ALGORITHM

#ifndef TEST_EVOLUTIONARY_ALGORITHM
/* An Evolutionary-algorithm based waypoint generation implementation */

void evolutionaryAlgorithmGenerator(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on with EVOLUTIONARY ALGORITHM");

    /* GET SIMULATION'S CONFIGURATIONS */
    std::string move_base_goals_topic, initial_pose_topic, move_base_status_topic, odom_topic, header_frame_id;
    if (!nodeHandle.getParam("move_base_goals_topic", move_base_goals_topic))
        ROS_ERROR("Could not find move_base_goals_topic parameter!");
    if (!nodeHandle.getParam("initial_pose_topic", initial_pose_topic))
        ROS_ERROR("Could not find initial_pose_topic parameter!");
    if (!nodeHandle.getParam("move_base_status_topic", move_base_status_topic))
        ROS_ERROR("Could not find move_base_status_topic parameter!");
    if (!nodeHandle.getParam("odom_topic", odom_topic))
        ROS_ERROR("Could not find odom_topic parameter!");
    if (!nodeHandle.getParam("header_frame_id", header_frame_id))
        ROS_ERROR("Could not find header_frame_id parameter!");

    /* INITIALIZE PROBLEM'S ENVIRONMENT */
    /* 35 degrees */
    // terrain.goal.position.x = 6.0; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    // terrain.start.position.x = 0.4; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    // terrain.goal_left.position.x = 6.0; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.4; terrain.start_left.position.y = 3.0;
    // terrain.goal_right.position.x = 6.0; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.4; terrain.start_right.position.y = -3.0;
    // terrain.slope = 35.0;
    /* 45 degrees */
    terrain.goal.position.x = 6.2; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 0.49; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 6.2; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.49; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 6.2; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.49; terrain.start_right.position.y = -3.0;
    terrain.slope = 45.0;

    /* incorporate lethal obstacles */
    geometry_msgs::Point temp;
    temp.x = 1.16; temp.y = 1.0; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.6; temp.y = -0.67; terrain.lethal_obstacles.push_back(temp);
    temp.x = 4.75; temp.y = 0.99; terrain.lethal_obstacles.push_back(temp);
    /* make lethal obstacles more complex */
    // first lethal obstacles formation
    temp.x = 2.102800; temp.y = 0.312000; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.2; temp.y = 0.31; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.2; temp.y = 0.32; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.3; temp.y = 0.31; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.3; temp.y = 0.32; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.3; temp.y = 0.33; terrain.lethal_obstacles.push_back(temp);
    // second lethal obstacles formation

    /* create publishers and subscribers */
    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    ros::Subscriber odom_sub = nodeHandle.subscribe("/odometry/filtered", 1, &odometryTopicCallback);
    ros::Rate loop_rate(9.0);

    /* publish initial pose */
    geometry_msgs::PoseStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.position.x = terrain.start.position.x; init.pose.position.y = terrain.start.position.y; init.pose.position.z = terrain.start.position.z;
    init.pose.orientation.x = 0.0; init.pose.orientation.y = 0.0; init.pose.orientation.z = 0.0; init.pose.orientation.w = 1.0;
    init_pose_pub.publish(init);

    /* CREATE GRAPH'S GRID */
    // cols = distance of goal_left from goal_right and possibly some border
    int cols = (distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    // rows = distance of goal_left from goal_left from start_left and what remains distributed equally up and down
    int rows = (distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    double left_right_border = std::fmod((distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    double up_down_border = std::fmod((distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    ROS_INFO("cols = %d, rows = %d, left_right_border = %f, up_down_border = %f", cols, rows, left_right_border, up_down_border);

    /* Create an initial population of random paths from start to goal */
    std::vector< std::vector<Waypoint> > individuals;
    // initialize pseudorandom numbers generator
    srand(time(NULL));
    // take every two consecutive lines, with a fixed p0 from the previous line
    for (int i = 0; i < INIT_GENERATION_SIZE; i++) {
        /* create a random path from start to finish */
        std::vector<Waypoint> path;
        // initially p0 is start
        Waypoint p0; p0.pose.pose.orientation.w = 1.0; p0.pose.header.frame_id = "odom";
        p0.pose.pose.position.x = terrain.start.position.x; p0.pose.pose.position.y = terrain.start.position.y;
        Waypoint last_p2 = p0;
        // take a random cell of each grid row
        for (int r = rows-1; r >= 0; r -= 2) {
            // pick a random column
            int i = rand() % cols;  // row 1 (lower)
            int j = rand() % cols;  // row 2 (upper)
            // calculate waypoints' coordinates
            double  p1_x = terrain.goal.position.x - (up_down_border + (r+1) * SEARCH_STEP),
                        p1_y = terrain.goal_left.position.y - (left_right_border + i * SEARCH_STEP),
                        p2_x = terrain.goal.position.x - (up_down_border + (r) * SEARCH_STEP),
                        p2_y = terrain.goal_left.position.y - (left_right_border + j * SEARCH_STEP);
                        /* if we are in the last phase of our search make an effort to reach our goal instantly */
                        if (r == 0 || r == 1) {
                            p2_x = terrain.goal.position.x;
                            p2_y = terrain.goal.position.y;
                        }
            std::vector<Waypoint> temp_control_points;
            // for debugging
            assert(r >= 0);
            /* create temporary waypoint for i (p1) */
            Waypoint p1; p1.pose.pose.orientation.w = 1.0; p1.pose.header.frame_id = "odom";
            p1.pose.pose.position.x = p1_x;
            p1.pose.pose.position.y = p1_y;
            p1.deviation = distanceFromLine(p1.pose, terrain.start, terrain.goal);
            /* create temporary waypoint for j (p2) */
            Waypoint p2; p2.pose.pose.orientation.w = 1.0; p2.pose.header.frame_id = "odom";
            p2.pose.pose.position.x = p2_x;
            p2.pose.pose.position.y = p2_y;
            p2.deviation = distanceFromLine(p2.pose, terrain.start, terrain.goal);
            /* find the Bezier curve that p0, p1 and p2 create */
            temp_control_points.push_back(p0); temp_control_points.push_back(p1); temp_control_points.push_back(p2);
            std::vector<Waypoint> bezier_curve;
            createSuboptimalBezierPath(temp_control_points, bezier_curve, (r-2 < 0));
            /* calculate points metrics */
            // calculate angles, deviation and where the vehicle is looking at any waypoint
            for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                iterator->deviation = distanceFromLine(iterator->pose, terrain.start, terrain.goal);

                if (iterator == bezier_curve.begin() && (iterator->pose.pose.position.x != last_p2.pose.pose.position.x) && (iterator->pose.pose.position.y != last_p2.pose.pose.position.y)) {
                    iterator->arc = eulerAngleOf(iterator->pose, last_p2.pose, std::next(iterator,1)->pose);
                }
                else if (std::next(iterator,1) != bezier_curve.end() && (iterator->pose.pose.position.x != std::next(iterator,1)->pose.pose.position.x) && (iterator->pose.pose.position.y != std::next(iterator,1)->pose.pose.position.y)) {
                    iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, std::next(iterator,1)->pose);

                    if (std::next(iterator,1)->pose.pose.position.y > iterator->pose.pose.position.y)
                        iterator->looking_right = true;     // we haven't turned yet
                    else
                        iterator->looking_right = false;    // we haven't turned yet
                }
                else {
                    iterator->arc = 0.0;
                }
            }
            // calculate costs
            for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                if (iterator == bezier_curve.begin())
                    iterator->cost = iterator->deviation;
                else
                    iterator->cost = std::prev(iterator, 1)->cost + iterator->deviation;
            }
            // calculate roll, pitch, yaw and height
            for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                pitchAt(*iterator);
                rollAt(*iterator);
                yawAt(*iterator);
            }

            p0 = temp_control_points.at(2);
            last_p2 = p0;

            // insert to path, we don't want p0 since it is either the start position or the previous p2
            path.insert(path.end(), std::next(bezier_curve.begin(), 1), bezier_curve.end());
            // path.insert(path.end(), std::next(temp_control_points.begin(), 1), temp_control_points.end());                
        }

        individuals.push_back(path);
    }

    /* Print initial random generation -- for debugging */
    printGeneration(individuals);

    /* Evaluate fitness of the initial individuals */
    std::vector<double> individuals_fitness;
    evaluateFitness(individuals, individuals_fitness);
    
    /* Sort individuals based on fitness */
    for (int i = 0; i < individuals_fitness.size()-1; i++) {
        for (int j = 0; j < individuals_fitness.size()-i-1; j++) {
            if (individuals_fitness.at(j) > individuals_fitness.at(j+1)) {
                double temp = individuals_fitness.at(j);
                individuals_fitness.at(j) = individuals_fitness.at(j+1);
                individuals_fitness.at(j+1) = temp;

                std::vector<Waypoint> tempVec = individuals.at(j);
                individuals.at(j) = individuals.at(j+1);
                individuals.at(j+1) = tempVec;
            }
        }
    }

    /* Path generator's main loop */
    int curr_generation = 1;
    std::deque<double> best_generations;
    do {
        /* for debugging */
        ROS_WARN("Current generation: %d", curr_generation);
        /* Select the best-fit individuals for reproduction (reproduction loops) */
        std::vector< std::vector<Waypoint> > offsprings;
        /* TODO: is crossover operation counterfeit?  */
        for (int i = 0; i < NUM_OF_BEST_FIT; i++) {
            for (int j = 0; j < NUM_OF_BEST_FIT; j++) {
                if (i != j) {
                    /* Use crossover operator to create new individuals */
                    std::vector<Waypoint> offspring_a, offspring_b;
                    /* make sure to crossover each two best-fit individuals */
                    crossover(individuals.at(i), individuals.at(j), offspring_a, offspring_b);
                    offsprings.push_back(offspring_a);
                    offsprings.push_back(offspring_b);
                }
            }

            /* Use mutator to modify individuals */
            // mutation(offsprings);
        }

        /* Evaluate fitness of new individuals */
        std::vector<double> offsprings_fitness;
        evaluateFitness(offsprings, offsprings_fitness);

        /* Remove the less-fit individuals (those that weren't selected for reproduction in this loop) */
        for (int i = 1; i <= individuals.size()-NUM_OF_BEST_FIT; i++) {
            individuals.pop_back();
            individuals_fitness.pop_back();
        }

        /* Keep the best-fit new individuals for the next loop */
        for (int i = 0; i < NUM_OF_BEST_FIT; i++) {
            individuals.push_back(offsprings.at(i));
            individuals_fitness.push_back(offsprings_fitness.at(i));
        }

        /* Sort individuals based on fitness */
        for (int i = 0; i < individuals_fitness.size()-1; i++) {
            for (int j = 0; j < individuals_fitness.size()-i-1; j++) {
                if (individuals_fitness.at(j) > individuals_fitness.at(j+1)) {
                    double temp = individuals_fitness.at(j);
                    individuals_fitness.at(j) = individuals_fitness.at(j+1);
                    individuals_fitness.at(j+1) = temp;

                    std::vector<Waypoint> tempVec = individuals.at(j);
                    individuals.at(j) = individuals.at(j+1);
                    individuals.at(j+1) = tempVec;
                }
            }
        }

        /* do the necessary bookeeping */
        curr_generation++;
        if (best_generations.size() > MAX_STAGNATED_GENS)
            best_generations.pop_front();
        best_generations.push_back(individuals_fitness.at(0));
    } while (!terminationCriteriaMet(individuals, individuals_fitness, best_generations, curr_generation));

    /* proceed with the best solution */
    ROS_INFO("Keeping best path");
    // ROS_INFO("Keeping best control points");
    /* for debugging */
    assert(individuals.size() > 0);
    std::vector<Waypoint> bezier_path = individuals.at(0);
    // std::vector<Waypoint> control_points = individuals.at(0);
    
    /* STITCH AND POPULATE BEZIER CURVES DESCRIBED BY THE ABOVE CONTROL POINTS TO FORM BEZIER PATH */
    // ROS_INFO("Creating Bezier path");
    // std::vector<Waypoint> bezier_path;
    // createSuboptimalBezierPath(control_points, bezier_path);

    /* Print Bezier path -- for debugging */
    ROS_INFO("Bezier path (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* Clean up the Bezier path from irrational sequences of waypoints that may have occurred buring calculations */
    cleanUpBezierPath(bezier_path);

    /* Print Bezier path */
    ROS_INFO("Bezier path (clean) (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* SEND BEZIER PATH TO move_base */
    std::vector<Waypoint>::iterator iterator = bezier_path.begin();
    ros::spinOnce();
    ros::Rate(9.0).sleep();
    first_time = true;
    while (ros::ok() && iterator != bezier_path.end()) {
        if (iterator == bezier_path.begin()) iterator++;
        goals_pub.publish(iterator->pose);
        while (distance(iterator->pose.pose.position, curr_pose_msg.pose.position) > 0.25) {
            ros::spinOnce();
            ros::Rate(6.0).sleep();
        }
        iterator++;
        ROS_INFO("Moving on...");
    }
}

#else
/* Test Evolutionary Algorithm generator implementation core functions by using the standard
    Hill-Climbing generator implementation as a testing basis */

void evolutionaryAlgorithmGenerator(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on with HILL-CLIMBING");

    /* GET SIMULATION'S CONFIGURATIONS */
    std::string move_base_goals_topic, initial_pose_topic, move_base_status_topic, odom_topic, header_frame_id;
    if (!nodeHandle.getParam("move_base_goals_topic", move_base_goals_topic))
        ROS_ERROR("Could not find move_base_goals_topic parameter!");
    if (!nodeHandle.getParam("initial_pose_topic", initial_pose_topic))
        ROS_ERROR("Could not find initial_pose_topic parameter!");
    if (!nodeHandle.getParam("move_base_status_topic", move_base_status_topic))
        ROS_ERROR("Could not find move_base_status_topic parameter!");
    if (!nodeHandle.getParam("odom_topic", odom_topic))
        ROS_ERROR("Could not find odom_topic parameter!");
    if (!nodeHandle.getParam("header_frame_id", header_frame_id))
        ROS_ERROR("Could not find header_frame_id parameter!");

    /* INITIALIZE PROBLEM'S ENVIRONMENT */
    /* 35 degrees */
    terrain.goal.position.x = 7.2; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 0.4; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 7.2; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.4; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 7.2; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.4; terrain.start_right.position.y = -3.0;
    terrain.slope = 35.0;
    /* 45 degrees */
    terrain.goal.position.x = 6.2; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 0.49; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 6.2; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.49; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 6.2; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.49; terrain.start_right.position.y = -3.0;
    terrain.slope = 45.0;

    /* create publishers and subscribers */
    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    ros::Subscriber odom_sub = nodeHandle.subscribe("/odometry/filtered", 1, &odometryTopicCallback);
    ros::Rate loop_rate(9.0);

    /* publish initial pose */
    geometry_msgs::PoseStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.position.x = terrain.start.position.x; init.pose.position.y = terrain.start.position.y; init.pose.position.z = terrain.start.position.z;
    init.pose.orientation.x = 0.0; init.pose.orientation.y = 0.0; init.pose.orientation.z = 0.0; init.pose.orientation.w = 1.0;
    init_pose_pub.publish(init);

    /* CREATE GRAPH'S GRID */
    // cols = distance of goal_left from goal_right and possibly some border
    int cols = (distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    // rows = distance of goal_left from goal_left from start_left and what remains distributed equally up and down
    int rows = (distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX) / SEARCH_STEP;
    double left_right_border = std::fmod((distance(terrain.goal_left.position, terrain.goal_right.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    double up_down_border = std::fmod((distance(terrain.goal_left.position, terrain.start_left.position) - ROBOT_BODY_FIX), SEARCH_STEP) / 2;
    ROS_INFO("cols = %d, rows = %d, left_right_border = %f, up_down_border = %f", cols, rows, left_right_border, up_down_border);

    /* FIND THE CONTROL POINTS OF A "GOOD ENOUGH" BEZIER PATH */
    // TODO: consider admissibility for trimming search???
    std::vector<Waypoint> control_points;
    // initially p0 is start
    Waypoint p0; p0.pose.pose.orientation.w = 1.0; p0.pose.header.frame_id = "odom";
    p0.pose.pose.position.x = terrain.start.position.x; p0.pose.pose.position.y = terrain.start.position.y;
    Waypoint last_p2 = p0;
    // starting position will definitely be a control point
    control_points.push_back(p0);
    // take every two consecutive lines, with a fixed p0 from the previous line
    double path_cost = 0.0;
    for (int r = rows-1; r >= 0; r -= 2) {
        double local_cost = 0.0, best_local_cost = std::numeric_limits<double>::max();
        bool has_worst_local_cost = false;
        // take every possible combination of quadratic Bezier curve control points p1 and p2 from these two lines
        std::vector<Waypoint> best_local_waypoints;
        for (int i = 0; i < cols; i++) {        // row 1 (lower)
            for (int j = 0; j < cols; j++) {    // row 2 (upper)
                double  p1_x = terrain.goal.position.x - (up_down_border + (r+1) * SEARCH_STEP),
                        p1_y = terrain.goal_left.position.y - (left_right_border + i * SEARCH_STEP),
                        p2_x = terrain.goal.position.x - (up_down_border + (r) * SEARCH_STEP),
                        p2_y = terrain.goal_left.position.y - (left_right_border + j * SEARCH_STEP);
                        /* if we are in the last phase of our search make an effort to reach our goal instantly */
                        if (r == 0 || r == 1) {
                            p2_x = terrain.goal.position.x;
                            p2_y = terrain.goal.position.y;
                        }
                /* we don't want to go straight ahead, also take a chance to deal with equalities caused by grid resolution */
                if ( i == j ||
                    (p2_x == p0.pose.pose.position.x && p2_y == p0.pose.pose.position.y) ||
                    (p1_x == p0.pose.pose.position.x && p1_y == p0.pose.pose.position.y) ||
                    (p2_x == p1_x && p2_y == p1_y) )
                    continue;
                std::vector<Waypoint> temp_control_points;
                // for debugging
                assert(r >= 0);
                /* create temporary waypoint for i (p1) */
                Waypoint p1; p1.pose.pose.orientation.w = 1.0; p1.pose.header.frame_id = "odom";
                p1.pose.pose.position.x = p1_x;
                p1.pose.pose.position.y = p1_y;
                p1.deviation = distanceFromLine(p1.pose, terrain.start, terrain.goal);
                /* create temporary waypoint for j (p2) */
                Waypoint p2; p2.pose.pose.orientation.w = 1.0; p2.pose.header.frame_id = "odom";
                p2.pose.pose.position.x = p2_x;
                p2.pose.pose.position.y = p2_y;
                p2.deviation = distanceFromLine(p2.pose, terrain.start, terrain.goal);
                /* find the Bezier curve that p0, p1 and p2 create */
                temp_control_points.push_back(p0); temp_control_points.push_back(p1); temp_control_points.push_back(p2);
                std::vector<Waypoint> bezier_curve;
                createSuboptimalBezierPath(temp_control_points, bezier_curve);
                /* calculate points metrics */
                // calculate angles, deviation and where the vehicle is looking at any waypoint
                for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                    iterator->deviation = distanceFromLine(iterator->pose, terrain.start, terrain.goal);

                    if (iterator == bezier_curve.begin() && (iterator->pose.pose.position.x != last_p2.pose.pose.position.x) && (iterator->pose.pose.position.y != last_p2.pose.pose.position.y)) {
                        iterator->arc = eulerAngleOf(iterator->pose, last_p2.pose, std::next(iterator,1)->pose);
                    }
                    else if (std::next(iterator,1) != bezier_curve.end() && (iterator->pose.pose.position.x != std::next(iterator,1)->pose.pose.position.x) && (iterator->pose.pose.position.y != std::next(iterator,1)->pose.pose.position.y)) {
                        iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, std::next(iterator,1)->pose);

                        if (std::next(iterator,1)->pose.pose.position.y > iterator->pose.pose.position.y)
                            iterator->looking_right = true;     // we haven't turned yet
                        else
                            iterator->looking_right = false;    // we haven't turned yet
                    }
                    else {
                        iterator->arc = 0.0;
                    }
                }
                // calculate costs
                for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                    if (iterator == bezier_curve.begin())
                        iterator->cost = iterator->deviation;
                    else
                        iterator->cost = std::prev(iterator, 1)->cost + iterator->deviation;
                }
                // calculate roll, pitch, yaw and height
                for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                    pitchAt(*iterator);
                    rollAt(*iterator);
                    yawAt(*iterator);
                }
                /* evaluate the Bezier curve */
                local_cost = evaluateBezierCurve(bezier_curve, has_worst_local_cost);
                // TODO: is the following OK here??? Is is necessary???
                // if the path that is currently may have the worst local cost, punish it with extra cost
                // if (local_cost == terrain.worst_local_cost)
                //     local_cost *= 2;
                /* if curve may be locally optimal */
                if (local_cost < best_local_cost) {
                    // temporarily save local curve's control points
                    while(best_local_waypoints.size()) best_local_waypoints.pop_back(); // pop the previous best waypoints
                    best_local_waypoints.push_back(p0); best_local_waypoints.push_back(p1); best_local_waypoints.push_back(p2);
                    best_local_cost = local_cost;
                }

                p0 = temp_control_points.at(2);
                last_p2 = p0;
            }
        }
        // add local curve's control points to the path
        if (best_local_waypoints.size()) {
            control_points.push_back(best_local_waypoints.at(1));
            control_points.push_back(best_local_waypoints.at(2));
        }
    }

    /* STITCH AND POPULATE BEZIER CURVES DESCRIBED BY THE ABOVE CONTROL POINTS TO FORM BEZIER PATH */
    ROS_INFO("Creating Bezier path");
    std::vector<Waypoint> bezier_path;
    createSuboptimalBezierPath(control_points, bezier_path);

    /* Print Bezier path -- for debugging */
    ROS_INFO("Bezier path (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* Clean up the Bezier path from irrational sequences of waypoints that may have occurred buring calculations */
    cleanUpBezierPath(bezier_path);

    /* Print Bezier path */
    ROS_INFO("Bezier path (clean) (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* TESTING EVOLUTIONARY ALGORITHM -- START */

    /* 
    *   TODO
    *
    *   TODO
    * 
    *   TODO
    */

    /* TESTING EVOLUTIONARY ALGORITHM -- START */

    /* SEND BEZIER PATH TO move_base */
    std::vector<Waypoint>::iterator iterator = bezier_path.begin();
    ros::spinOnce();
    ros::Rate(9.0).sleep();
    first_time = true;
    while (ros::ok() && iterator != bezier_path.end()) {
        if (iterator == bezier_path.begin()) iterator++;
        goals_pub.publish(iterator->pose);
        while (distance(iterator->pose.pose.position, curr_pose_msg.pose.position) > 0.25) {
            ros::spinOnce();
            ros::Rate(6.0).sleep();
        }
        iterator++;
        ROS_INFO("Moving on...");
    }
}

#endif