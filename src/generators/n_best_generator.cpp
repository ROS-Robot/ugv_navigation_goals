#include "../../include/header.hpp"

/* An N-best based waypoint generation implementation */
/*
    Main idea:
    In each search stage we keep a deque with the N-best choices that don't gp through lethal obstacles.
    Then, we move on with the best viable local choice.
    If in the next stage of the search we always run into lethal obstacles then we backtrack to the
    previous step and we choose the next best viables choices.
    If we run into lethal obstacles as many times as our N-best viable alternatives, then the problem
    is rendered as "UNSOLVABLE" by the current implementation.
*/
int nBestGenerator(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on N-BEST");

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
    // terrain.goal.position.x = 6.2; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    // terrain.start.position.x = 0.4; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    // terrain.goal_left.position.x = 6.2; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.4; terrain.start_left.position.y = 3.0;
    // terrain.goal_right.position.x = 6.2; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.4; terrain.start_right.position.y = -3.0;
    // terrain.slope = 35.0;
    /* 45 degrees */
    // terrain.goal.position.x = 6.2; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    // terrain.start.position.x = 0.49; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    // terrain.goal_left.position.x = 6.2; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 0.49; terrain.start_left.position.y = 3.0;
    // terrain.goal_right.position.x = 6.2; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 0.49; terrain.start_right.position.y = -3.0;
    // terrain.slope = 45.0;
    /* 43 degrees - 30 meters (<pose frame=''>87.25 0.0 -8 0 0.125 0</pose>) */
    terrain.goal.position.x = 45.0; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 1.0; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 45.0; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 1.0; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 45.0; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 1.0; terrain.start_right.position.y = -3.0;
    terrain.slope = 43.0;

    // incorporate lethal obstacles
    geometry_msgs::Point temp;
    temp.x = 1.6; temp.y = -0.027; terrain.lethal_obstacles.push_back(temp);
    temp.x = 2.6; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 4.3; temp.y = 1.6; terrain.lethal_obstacles.push_back(temp);
    temp.x = 20.8; temp.y = 0.8; terrain.lethal_obstacles.push_back(temp);
    temp.x = 29.825; temp.y = -0.675; terrain.lethal_obstacles.push_back(temp);
    temp.x = 38.82; temp.y = 0.82; terrain.lethal_obstacles.push_back(temp);

    /* Print lethal obstacles -- for documentation */
    for (std::vector<geometry_msgs::Point>::const_iterator it = terrain.lethal_obstacles.begin(); it != terrain.lethal_obstacles.end(); it++)
        ROS_INFO("Lethal obstacle at (x, y) = (%f, %f)", it->x, it->y);

    /* create publishers and subscribers */
    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/initialpose", 1);
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
    std::deque< std::deque< std::pair< std::pair<Waypoint, Waypoint>, int > > > all_n_best;
    int loops = 0;  // count loops -- for debugging
    // initially p0 is start
    Waypoint p0; p0.pose.pose.orientation.w = 1.0; p0.pose.header.frame_id = "odom";
    p0.pose.pose.position.x = terrain.start.position.x; p0.pose.pose.position.y = terrain.start.position.y;
    Waypoint last_p2 = p0;
    // starting position will definitely be a control point
    control_points.push_back(p0);
    // take every two consecutive lines, with a fixed p0 from the previous line
    double path_cost = 0.0;
    /* Count visited states -- for documentation */
    int visited_states = 0;
    for (int r = rows-1; r >= 0; r -= 2) {
        loops++;    // count loops -- for debugging
        double local_cost = 0.0, best_local_cost = std::numeric_limits<double>::max();
        bool has_worst_local_cost = false;
        // take every possible combination of quadratic Bezier curve control points p1 and p2 from these two lines
        std::vector<Waypoint> best_local_waypoints;
        std::deque< std::pair< std::pair<Waypoint, Waypoint>, int > > n_best_control_points;
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
                /* we have one new visited state, count it -- for documentation */
                visited_states++;
                // local control points
                std::vector<Waypoint> temp_control_points;
                // for debugging
                assert(r >= 0);
                ROS_INFO("p0 = (%f, %f)", p0.pose.pose.position.x, p0.pose.pose.position.y);
                /* create temporary waypoint for i (p1) */
                Waypoint p1; p1.pose.pose.orientation.w = 1.0; p1.pose.header.frame_id = "odom";
                p1.pose.pose.position.x = p1_x;
                p1.pose.pose.position.y = p1_y;
                p1.deviation = distanceFromLine(p1.pose, terrain.start, terrain.goal);
                ROS_INFO("p1 = (%f, %f)", p1.pose.pose.position.x, p1.pose.pose.position.y);
                /* create temporary waypoint for j (p2) */
                Waypoint p2; p2.pose.pose.orientation.w = 1.0; p2.pose.header.frame_id = "odom";
                p2.pose.pose.position.x = p2_x;
                p2.pose.pose.position.y = p2_y;
                p2.deviation = distanceFromLine(p2.pose, terrain.start, terrain.goal);
                ROS_INFO("p2 = (%f, %f)", p2.pose.pose.position.x, p2.pose.pose.position.y);
                /* find the Bezier curve that p0, p1 and p2 create */
                temp_control_points.push_back(p0); temp_control_points.push_back(p1); temp_control_points.push_back(p2);
                std::vector<Waypoint> bezier_curve;
                createSuboptimalBezierPath(temp_control_points, bezier_curve, (r-2 < 0));   // if r-2 < 0 then we are in the last loop

                /* detect contact with lethal obstacle */
                bool danger = false;
                for (std::vector<Waypoint>::iterator it = bezier_curve.begin(); it != bezier_curve.end(); it++) {
                    // ROS_INFO("(x, y) = (%f, %f)", it->pose.pose.position.x, it->pose.pose.position.y);
                    if (proximityToLethalObstacle(*it)) {
                        // deal with the contact
                        danger = true;
                        ROS_WARN("Danger at loop %d for (x, y) = (%f, %f) !!!", loops, it->pose.pose.position.x, it->pose.pose.position.y);
                    }
                }

                /* if projected Bezier path is safe then proceed */
                if (!danger) {
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
                    /* if curve may be locally optimal */
                    if (local_cost < best_local_cost) {
                        // temporarily save local curve's control points
                        while(best_local_waypoints.size()) best_local_waypoints.pop_back(); // pop the previous best waypoints
                        best_local_waypoints.push_back(p0); best_local_waypoints.push_back(p1); best_local_waypoints.push_back(p2);
                        best_local_cost = local_cost;
                        /* local N-best bookeeping */
                        // if we can't fit in any other local best, then trow the worst (last) one away
                        if (n_best_control_points.size() == N)
                            n_best_control_points.pop_back();
                        std::pair<Waypoint, Waypoint> p = std::make_pair(p1, p2);
                        std::pair< std::pair<Waypoint, Waypoint>, int > best = std::make_pair(p, best_local_cost);
                        n_best_control_points.push_front(best);
                    }
                    /* else check if curve can be one of the N-best */
                    else {
                        // local N-best bookeeping
                        bool kept = false;
                        for (std::deque< std::pair< std::pair<Waypoint, Waypoint>, int > >::iterator it = n_best_control_points.begin(); it != n_best_control_points.end(); it++) {
                            // if we have an equally best fallback option add it right after it if you can
                            if (local_cost == it->second && it != std::prev(n_best_control_points.end(), 1))
                                it++;
                            // if we have a better fallback option
                            if (local_cost < it->second) {
                                std::pair<Waypoint, Waypoint> p = std::make_pair(p1, p2);
                                std::pair< std::pair<Waypoint, Waypoint>, int > good_enough = std::make_pair(p, best_local_cost);
                                std::swap(*it, good_enough);
                                // shift the remaining N-best elements
                                for (std::deque< std::pair< std::pair<Waypoint, Waypoint>, int > >::iterator j = it; j != n_best_control_points.end(); j++)
                                    std::swap(*j, good_enough);
                                // we are done
                                kept = true;
                                break;
                            }
                        }
                        // if there is room in the end, keep it to the end
                        if (!kept && n_best_control_points.size() < N) {
                            std::pair<Waypoint, Waypoint> p = std::make_pair(p1, p2);
                            std::pair< std::pair<Waypoint, Waypoint>, int > good_enough = std::make_pair(p, best_local_cost);
                            n_best_control_points.push_back(good_enough);
                        }
                    }

                    p0 = temp_control_points.at(2);
                    last_p2 = p0;
                }
            }
        }

        /* if we reached the end of our local search and we haven't found anything both viable and locally optimal */
        if (best_local_waypoints.size() == 0) {
            ROS_INFO("Backtrack");
            // "backtrack"
            loops--;
            r += 4; // += 2 stays at the same place, += 4 takes one step back
            // fallback to the next best choice from the previous local search
            // you can't fallback from the first step
            if (loops != 1) {
                all_n_best.at(all_n_best.size()-1).pop_front(); // pop the previously best option
                if (!all_n_best.at(all_n_best.size()-1).empty()) {
                    control_points.pop_back(); control_points.pop_back();   // pop latest viable p1, p2
                    p0 = control_points.at(control_points.size()-1);    // from where we last started
                    last_p2 = p0;
                    ROS_WARN("p0 is again (%f, %f)", p0.pose.pose.position.x, p0.pose.pose.position.y);
                }
                // as far as this implementation is concerned we have reached a dead-end
                else {
                    ROS_ERROR("DEAD END");
                    return -1;
                }
            }
            else {
                ROS_ERROR("DEAD END");
                return -1;
            }
        }
        /* else proceed normally */
        else {
            // add local curve's control points to the path
            if (best_local_waypoints.size()) {
                control_points.push_back(best_local_waypoints.at(1));
                control_points.push_back(best_local_waypoints.at(2));
            }

            // add local n-best to all n-best
            all_n_best.push_back(n_best_control_points);
        }
    }

    /* Print all N-best -- for debugging */
    ROS_INFO("loops = %d", loops); // count loops -- for debugging
    for (std::deque< std::deque< std::pair< std::pair<Waypoint, Waypoint>, int > > >::iterator i = all_n_best.begin(); i != all_n_best.end(); i++) {
        for (std::deque< std::pair< std::pair<Waypoint, Waypoint>, int > >::iterator j = i->begin(); j != i->end(); j++)
            ROS_INFO("(%f, %f)", j->first.first.pose.pose.position.x, j->first.second.pose.pose.position.y);
        ROS_INFO("--------------------------------------");
    }

    /* STITCH AND POPULATE BEZIER CURVES DESCRIBED BY THE ABOVE CONTROL POINTS TO FORM BEZIER PATH */
    ROS_INFO("Creating Bezier path");
    std::vector<Waypoint> bezier_path;
    createSuboptimalBezierPath(control_points, bezier_path);

    /* Print Bezier path -- for debugging */
    ROS_INFO("Bezier path (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* Print visited states -- for documentation */
    ROS_INFO("Path generator visited %d states during search", visited_states);

    /* Print Bezier path's cost and length -- for documentation */
    calculateBezierCurveMetrics(bezier_path);   // we need metrics for cost calculation
    // print path's details -- for debugging
    printBezierPathDetails(bezier_path);    
    bool has_worst_local_cost = false;
    ROS_INFO("Bezier path cost = %f, length = %f meters", evaluateBezierCurve(bezier_path, has_worst_local_cost), bezierPathLength(bezier_path));

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

    return 0;
}