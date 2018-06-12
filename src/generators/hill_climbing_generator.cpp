#include "../../include/header.hpp"

/* A Hill-climbing based waypoint generation implementation */
void hillClimbingGenerator(int argc, char *argv[]) {
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
    /* 43 degrees - 30 meters (84.15 * -8 0 0.09 0) */
    terrain.goal.position.x = 40.0; terrain.goal.position.y = 0.0; terrain.goal.position.z = 0.0;
    terrain.start.position.x = 1.0; terrain.start.position.y = 0.0; terrain.start.position.z = 0.0;
    terrain.goal_left.position.x = 40.0; terrain.goal_left.position.y = 3.0; terrain.start_left.position.x = 1.0; terrain.start_left.position.y = 3.0;
    terrain.goal_right.position.x = 40.0; terrain.goal_right.position.y = -3.0; terrain.start_right.position.x = 1.0; terrain.start_right.position.y = -3.0;
    terrain.slope = 45.0;

    // incorporate no obstacles

    /* Print lethal obstacles -- for documentation */
    for (std::vector<geometry_msgs::Point>::const_iterator it = terrain.lethal_obstacles.begin(); it != terrain.lethal_obstacles.end(); it++)
        ROS_INFO("Lethal obstacle at (x, y) = (%f, %f)", it->x, it->y);

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
    /* Count visited states -- for documentation */
    int visited_states = 0;
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
                createBezierPath(temp_control_points, bezier_curve);
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
                ROS_WARN("local_cost = %f", local_cost);
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

    /* Print visited states -- for documentation */
    ROS_INFO("Path generator visited %d states during search", visited_states);

    /* STITCH AND POPULATE BEZIER CURVES DESCRIBED BY THE ABOVE CONTROL POINTS TO FORM BEZIER PATH */
    ROS_INFO("Creating Bezier path");
    std::vector<Waypoint> bezier_path;
    createSuboptimalBezierPath(control_points, bezier_path);

    /* Print Bezier path -- for debugging */
    ROS_INFO("Bezier path (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* Print Bezier path's cost and length -- for documentation */
    calculateBezierCurveMetrics(bezier_path);   // we need metrics for cost calculation
    // print path's details -- for debugging
    // printBezierPathDetails(bezier_path);    
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
}