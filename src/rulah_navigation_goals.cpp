#include "../include/header.hpp"

// #define TEST_BEZIER
// #define TEST_CALCULATIONS

#ifdef TEST_BEZIER

/* node's main function */
int main(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on");

    /* INITIALIZE TERRAIN */
    terrain.goal.position.x = 1.5; terrain.goal.position.y = 6.0; terrain.start.position.x = -4.85; terrain.start.position.y = 6.0;
    terrain.goal_left.position.x = 1.5; terrain.goal_left.position.y = 7.0; terrain.start_left.position.x = -4.85; terrain.start_left.position.y = 7.0;
    terrain.goal_right.position.x = 1.5; terrain.goal_right.position.y = 5.0; terrain.start_right.position.x = -4.85; terrain.start_right.position.y = 5.0;
    terrain.slope = 40.0;

    /* TEST BEZIER CURVE FUNCTIONS, BY CREATING A DUMB PLAN */
    std::vector<Waypoint> control_points, bezier_path;
    geometry_msgs::Point p0, p1, p2;

    /* Find the control_points of a series of Bezier curves */
    /* 1 */
    p0.x = terrain.start.position.x; p0.y = terrain.start.position.y; p1.x = -4.4; p1.y = 6.4; p2.x = -4.0; p2.y = 6.2;
    Waypoint temp;
    temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
    temp.pose.pose.position.x = p0.x; temp.pose.pose.position.y = p0.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);
    /* 2 */
    p0.x = p2.x; p0.y = p2.y; p1.x = -3.5; p1.y = 5.8; p2.x = -3; p2.y = 6.0;
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);
    /* 3 */
    p0.x = p2.x; p0.y = p2.y; p1.x = -2.5; p1.y = 6.2; p2.x = -2; p2.y = 6.4;
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);
    /* 4 */
    p0.x = p2.x; p0.y = p2.y; p1.x = -1.7; p1.y = 5.6; p2.x = -1; p2.y = 5.8;
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);
    /* 5 */
    p0.x = p2.x; p0.y = p2.y; p1.x = -0.8; p1.y = 6.0; p2.x = 0.0; p2.y = 6.2;
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);
    /* 6 */
    p0.x = p2.x; p0.y = p2.y; p1.x = 0.0; p1.y = 6.2; p2.x = terrain.goal.position.x; p2.y = terrain.goal.position.y;
    temp.pose.pose.position.x = p1.x; temp.pose.pose.position.y = p1.y; control_points.push_back(temp);
    temp.pose.pose.position.x = p2.x; temp.pose.pose.position.y = p2.y; control_points.push_back(temp);

    /* Print the above Bezier control points */
    ROS_INFO("Control Points (size = %ld):", control_points.size());
    for (int i = 0; i < control_points.size(); i++)
        ROS_INFO("(%f, %f)", control_points.at(i).pose.pose.position.x, control_points.at(i).pose.pose.position.y);

    /* Create a Bezier path by stiching the above Bezier curves together */
    createBezierPath(control_points, bezier_path);

    /* Print Bezier path */
    ROS_INFO("Bezier path (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* Interpolate the above Bezier path */
    interpolateBezierPath(bezier_path, INTERPOLATION_SCALE);

    /* Print interpolated Bezier path */
    ROS_INFO("Interpolated Bezier path (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* Evaluate Bezier path */
    // TODO: add metrics for the path's waypoints before uncommenting the following
    // bool has_worst_local_cost = false;
    // double local_cost = evaluateBezierCurve(control_points, has_worst_local_cost);
    // ROS_INFO("Bezier path cost = %f", local_cost);

    /* Attempt to send Bezier path to move_base */
    // Publishers and Subscribers stuff
    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    ros::Rate loop_rate(9.0);

    // Publish initial pose
    geometry_msgs::PoseStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.position.x = terrain.start.position.x; init.pose.position.y = terrain.start.position.y; init.pose.position.z = 4.0;
    init.pose.orientation.x = 0.0; init.pose.orientation.y = 0.0; init.pose.orientation.z = 0.0; init.pose.orientation.w = 1.0;
    init_pose_pub.publish(init);

    // send path
    std::vector<Waypoint>::iterator iterator = bezier_path.begin();
    while (ros::ok() && iterator != bezier_path.end()) {
        goals_pub.publish(iterator->pose);
        if (first_time || ( move_base_status_msg.status_list.size() > 0 && move_base_status_msg.status_list[0].status != PENDING && move_base_status_msg.status_list[0].status != ACTIVE && move_base_status_msg.status_list[0].status != PREEMPTING) ) {
            goals_pub.publish(iterator->pose);
            first_time == false;
            if (!first_time) {
                iterator++;
                first_time = true;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

#elif defined( TEST_CALCULATIONS )

/* node's main function */
int main(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on");

    first_time = true;
    num_of_waypoints = 0;

    /* INITIALIZE TERRAIN */
    terrain.goal.position.x = 1.5; terrain.goal.position.y = 6.0; terrain.start.position.x = -4.85; terrain.start.position.y = 6.0;
    terrain.goal_left.position.x = 1.5; terrain.goal_left.position.y = 7.0; terrain.start_left.position.x = -4.85; terrain.start_left.position.y = 7.0;
    terrain.goal_right.position.x = 1.5; terrain.goal_right.position.y = 5.0; terrain.start_right.position.x = -4.85; terrain.start_right.position.y = 5.0;
    terrain.slope = 40.0;
    geometry_msgs::PoseStamped temp;
    temp.pose.position.x = -3.0; temp.pose.position.y = 5.5; terrain.lethal_obstacles.push_back(temp);
    temp.pose.position.x = -1.0; temp.pose.position.y = 5.5; terrain.lethal_obstacles.push_back(temp);
    temp.pose.position.x = -1.5; temp.pose.position.y = 6.2; terrain.lethal_obstacles.push_back(temp);

    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    // ros::Subscriber pose_sub = nodeHandle.subscribe("/pose", 1, &poseTopicCallback);
    // ros::Subscriber move_base_feedback_sub = nodeHandle.subscribe("/move_base/feedback", 1, &moveBaseFeedbackCallback);
    ros::Rate loop_rate(9.0);

    /* PUBLISH INITIAL POSE */
    geometry_msgs::PoseStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.position.x = terrain.start.position.x; init.pose.position.y = terrain.start.position.y; init.pose.position.z = 4.0;
    init.pose.orientation.x = 0.0; init.pose.orientation.y = 0.0; init.pose.orientation.z = 0.0; init.pose.orientation.w = 1.0;
    init_pose_pub.publish(init);

    /* CREATE INITIAL PLAN */
    double x = -4.85, y = 6.5, angle = 45.0;
    ROS_INFO("Creating initial plan");
    while (x < GRANT_X) {
        num_of_waypoints++;
        x += 0.95;
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = x; goal.pose.position.y = y; goal.pose.position.z = 0.0;
        // goal.pose.orientation.w = 1.0;
        goal.pose.orientation = turnEulerAngleToQuaternion(angle);

        Waypoint tempw(goal);
        tempw.id = num_of_waypoints;
        tempw.cost = 0.0;    // assignment at the iterator below
        tempw.arc = 0;       // assignment at the iterator below
        tempw.looking_right = true; // assignment at the iterator below
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        // tempw.traversability = 0;        // TODO LATER
        // tempw.traversability_slope = 0;  // TODO LATER

        /* debugging */
        // ROS_WARN("%f", angle);

        if (angle == 45.0) {
            angle = 135.0;
            y = 5.5;
        }
        else {
            angle = 45.0;
            y = 6.5;
        }
        waypoints_list.push_back(tempw);
    }

    // calculate angles and where the vehicle is looking at any waypoint
    for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator) {
        if (iterator == waypoints_list.begin())
            iterator->arc = eulerAngleOf(iterator->pose, init, std::next(iterator,1)->pose);
        else if (std::next(iterator,1) != waypoints_list.end()) {
            iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, std::next(iterator,1)->pose);

            if (std::next(iterator,1)->pose.pose.position.y > iterator->pose.pose.position.y)
                iterator->looking_right = true; // we haven't turned yet
            else
                iterator->looking_right = false; // we haven't turned yet
        }
        else
            iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, terrain.goal);
    }
    // calculate costs
    for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator) {
        if (iterator == waypoints_list.begin())
            iterator->cost = iterator->deviation;
        else
            iterator->cost = std::prev(iterator, 1)->cost + iterator->deviation;
    }
    // calculate roll, pitch, yaw
    for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator) {
        pitchAt(*iterator);
        rollAt(*iterator);
        yawAt(*iterator);
        if (iterator != waypoints_list.begin()) {
            ROS_WARN("waypoint %d admissibility = %d", iterator->id, isAdmissible(*iterator, *(std::prev(iterator,1))));
        }
    }

    ROS_INFO("We have the initial goals:");
    for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator)
        ROS_INFO("(p.x = %f, p.y = %f, p.z = %f), (o.x = %f, o.y = %f, o.z = %f, o.w = %f)",
                    iterator->pose.pose.position.x, iterator->pose.pose.position.y, iterator->pose.pose.position.z,
                    iterator->pose.pose.orientation.x, iterator->pose.pose.orientation.y, iterator->pose.pose.orientation.z, iterator->pose.pose.orientation.w);

    /* EVALUATE INITIAL PLAN (for debugging) */
    bool has_worst_local_cost = false;
    double eval = evaluate(waypoints_list, has_worst_local_cost);
    ROS_WARN("Evaluation = %f\t(has_worst_local_cost = %d)", eval, has_worst_local_cost);

    // /* FORM OPTIMAL PLAN */
    // generateOptimalPlan();
    //
    // // recalculate angles
    // for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator) {
    //     if (iterator == waypoints_list.begin()) iterator->arc = eulerAngleOf(iterator->pose, init, std::next(iterator,1)->pose);
    //     else if (std::next(iterator,1) != waypoints_list.end()) iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, std::next(iterator,1)->pose);
    //     else iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, terrain.goal);
    // }
    // // recalculate costs
    // for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator) {
    //     if (iterator == waypoints_list.begin())
    //         iterator->cost = iterator->deviation;
    //     else
    //         iterator->cost = std::prev(iterator, 1)->cost + iterator->deviation;
    // }
    //
    // ROS_INFO("We have the optimal goals:");
    // for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator)
    //     ROS_INFO("(p.x = %f, p.y = %f, p.z = %f), (o.x = %f, o.y = %f, o.z = %f, o.w = %f)",
    //                 iterator->pose.pose.position.x, iterator->pose.pose.position.y, iterator->pose.pose.position.z,
    //                 iterator->pose.pose.orientation.x, iterator->pose.pose.orientation.y, iterator->pose.pose.orientation.z, iterator->pose.pose.orientation.w);

    /* GRADUALLY SEND PLAN TO move_base */
    x = -3.0, y = 6.5, angle = 45.0;
    std::list<Waypoint>::iterator iterator = waypoints_list.begin();
    // ROS_INFO("Sending goals to move_base");
    // first_time = true;
    while (ros::ok() && iterator != waypoints_list.end()) {
        goals_pub.publish(iterator->pose);
        if (first_time || ( move_base_status_msg.status_list.size() > 0 && move_base_status_msg.status_list[0].status != PENDING && move_base_status_msg.status_list[0].status != ACTIVE && move_base_status_msg.status_list[0].status != PREEMPTING) ) {
            // ROS_INFO("Publishing goal");
            goals_pub.publish(iterator->pose);
            first_time == false;
            if (!first_time) {
                // ROS_INFO("iterator++");
                iterator++;
                first_time = true;
            }
            // first_time = false;
        }
        // ROS_INFO("after if");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

#else

int main(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on");

    /* INITIALIZE PROBLEM'S ENVIRONMENT */
    terrain.goal.position.x = 1.5; terrain.goal.position.y = 6.0; terrain.start.position.x = -4.85; terrain.start.position.y = 6.0;
    terrain.goal_left.position.x = 1.5; terrain.goal_left.position.y = 7.0; terrain.start_left.position.x = -4.85; terrain.start_left.position.y = 7.0;
    terrain.goal_right.position.x = 1.5; terrain.goal_right.position.y = 5.0; terrain.start_right.position.x = -4.85; terrain.start_right.position.y = 5.0;
    terrain.slope = 40.0;
    // TODO: incorporate lethal obstacles
    geometry_msgs::PoseStamped temp;
    temp.pose.position.x = -3.0; temp.pose.position.y = 5.5; terrain.lethal_obstacles.push_back(temp);
    temp.pose.position.x = -1.0; temp.pose.position.y = 5.5; terrain.lethal_obstacles.push_back(temp);
    temp.pose.position.x = -1.5; temp.pose.position.y = 6.2; terrain.lethal_obstacles.push_back(temp);

    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    ros::Rate loop_rate(9.0);

    /* publish initial pose */
    geometry_msgs::PoseStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.position.x = terrain.start.position.x; init.pose.position.y = terrain.start.position.y; init.pose.position.z = 4.0;
    init.pose.orientation.x = 0.0; init.pose.orientation.y = 0.0; init.pose.orientation.z = 0.0; init.pose.orientation.w = 1.0;
    init_pose_pub.publish(init);

    /* CREATE GRAPH'S GRID */
    // cols = distance of goal_left from goal_right and possibly some border
    int cols = distance(terrain.goal_left.position, terrain.goal_right.position) / SEARCH_STEP;
    // rows = distance of goal_left from goal_left from start_left and what remains distributed equally up and down
    int rows = distance(terrain.goal_left.position, terrain.start_left.position) / SEARCH_STEP;
    double left_right_border = std::fmod(distance(terrain.goal_left.position, terrain.goal_right.position), SEARCH_STEP) / 2;
    double up_down_border = std::fmod(distance(terrain.goal_left.position, terrain.start_left.position), SEARCH_STEP) / 2;
    // ROS_INFO("cols = %d, rows = %d, left_right_border = %f, up_down_border = %f", cols, rows, left_right_border, up_down_border);

    /* FIND THE CONTROL POINTS OF A "GOOD ENOUGH" BEZIER PATH */
    // TODO: consider admissibility for trimming search???
    std::vector<Waypoint> control_points;
    // initially p0 is start
    Waypoint p0; p0.pose.pose.orientation.w = 1.0; p0.pose.header.frame_id = "odom";
    p0.pose.pose.position.x = terrain.start.position.x; p0.pose.pose.position.y = terrain.start.position.y;
    Waypoint last_p2 = p0;
    // starting position will definitelly be a control point
    control_points.push_back(p0);
    // take every two consecutive lines, with a fixed p0 from the previous line
    double path_cost = 0.0;
    // for (int r = 0; r < rows-1; r += 2) {
    for (int r = rows-1; r >= 0; r -= 2) {
        double local_cost = 0.0, best_local_cost = std::numeric_limits<double>::max();
        bool has_worst_local_cost = false;
        // take every possible combination of quadratic Bezier curve control points p1 and p2 from these two lines
        std::vector<Waypoint> best_local_waypoints;
        for (int i = 0; i < cols; i++) {        // row 1 (lower)
            for (int j = 0; j < cols; j++) {    // row 2 (upper)
                double  p1_x = terrain.goal.position.x - (up_down_border + r * SEARCH_STEP),
                        p1_y = left_right_border + i * SEARCH_STEP,
                        p2_x = terrain.goal.position.x - (up_down_border + (r+1) * SEARCH_STEP),
                        p2_y = left_right_border + j * SEARCH_STEP;
                /* we don't want to go straight ahead, also take a chance to deal with equalities caused by grid resolution */
                if (i == j || (p2_x == p0.pose.pose.position.x && p2_y == p0.pose.pose.position.y) || (p1_x == p0.pose.pose.position.x && p1_y == p0.pose.pose.position.y) || (p2_x == p1_x && p2_y == p1_y))
                    continue;
                std::vector<Waypoint> temp_control_points;
                /* create temporary waypoint for i (p1) */
                Waypoint p1; p1.pose.pose.orientation.w = 1.0; p1.pose.header.frame_id = "odom";
                // for debugging
                // assert(r < rows && r+1 < rows);
                assert(r >= 0);
                // ROS_INFO("p0 = %f %f %f", p0.pose.pose.position.x, p0.pose.pose.position.y, p0.deviation);
                // TODO: p1 coords
                p1.pose.pose.position.x = p1_x;
                p1.pose.pose.position.y = p1_y;
                p1.deviation = distanceFromLine(p1.pose, terrain.start, terrain.goal);
                // ROS_INFO("p1 = %f %f %f", p1.pose.pose.position.x, p1.pose.pose.position.y, p1.deviation);
                /* create temporary waypoint for j (p2) */
                Waypoint p2; p2.pose.pose.orientation.w = 1.0; p2.pose.header.frame_id = "odom";
                // TODO: p2 coords
                p2.pose.pose.position.x = p2_x;
                p2.pose.pose.position.y = p2_y;
                p2.deviation = distanceFromLine(p2.pose, terrain.start, terrain.goal);
                // ROS_INFO("p2 = %f %f %f", p2.pose.pose.position.x, p2.pose.pose.position.y, p2.deviation);
                /* find the Bezier curve that p0, p1 and p2 create */
                temp_control_points.push_back(p0); temp_control_points.push_back(p1); temp_control_points.push_back(p2);
                std::vector<Waypoint> bezier_curve;
                createBezierPath(temp_control_points, bezier_curve);
                /* calculate points metrics */
                // calculate angles, deviation and where the vehicle is looking at any waypoint
                // ROS_INFO("angles, deviation and looking");
                for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                    iterator->deviation = distanceFromLine(iterator->pose, terrain.start, terrain.goal);

                    if (iterator == bezier_curve.begin() && (iterator->pose.pose.position.x != last_p2.pose.pose.position.x) && (iterator->pose.pose.position.y != last_p2.pose.pose.position.y)) {
                        // ROS_INFO("arc A (%f, %f), (%f, %f), (%f, %f)", iterator->pose.pose.position.x, iterator->pose.pose.position.y, last_p2.pose.pose.position.x, last_p2.pose.pose.position.y, std::next(iterator, 1)->pose.pose.position.x, std::next(iterator, 1)->pose.pose.position.y);
                        iterator->arc = eulerAngleOf(iterator->pose, last_p2.pose, std::next(iterator,1)->pose);
                    }
                    else if (std::next(iterator,1) != bezier_curve.end() && (iterator->pose.pose.position.x != std::next(iterator,1)->pose.pose.position.x) && (iterator->pose.pose.position.y != std::next(iterator,1)->pose.pose.position.y)) {
                        // ROS_INFO("arc B (%f, %f), (%f, %f), (%f, %f)", iterator->pose.pose.position.x, iterator->pose.pose.position.y, std::prev(iterator,1)->pose.pose.position.x, std::prev(iterator,1)->pose.pose.position.y, std::next(iterator, 1)->pose.pose.position.x, std::next(iterator, 1)->pose.pose.position.y);
                        iterator->arc = eulerAngleOf(iterator->pose, std::prev(iterator,1)->pose, std::next(iterator,1)->pose);

                        if (std::next(iterator,1)->pose.pose.position.y > iterator->pose.pose.position.y)
                            iterator->looking_right = true; // we haven't turned yet
                        else
                            iterator->looking_right = false; // we haven't turned yet
                    }
                    else {
                        // ROS_INFO("arc C");
                        iterator->arc = 0.0;
                    }
                    // ROS_INFO("%f", iterator->arc);
                }
                // calculate costs
                // ROS_INFO("costs");
                for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                    if (iterator == bezier_curve.begin())
                        iterator->cost = iterator->deviation;
                    else
                        iterator->cost = std::prev(iterator, 1)->cost + iterator->deviation;
                }
                // calculate roll, pitch, yaw
                // ROS_INFO("roll, pitch, yaw");
                for (std::vector<Waypoint>::iterator iterator = bezier_curve.begin(); iterator != bezier_curve.end(); ++iterator) {
                    pitchAt(*iterator);
                    rollAt(*iterator);
                    yawAt(*iterator);
                }
                /* evaluate the Bezier curve */
                // ROS_INFO("p0 = (%f, %f), p1 = (%f, %f), p2 = (%f, %f)",
                //     p0.pose.pose.position.x, p0.pose.pose.position.y, p1.pose.pose.position.x, p1.pose.pose.position.y, p2.pose.pose.position.x, p2.pose.pose.position.y);
                // ROS_INFO("evaluating");
                local_cost = evaluateBezierCurve(bezier_curve, has_worst_local_cost);
                ROS_INFO("local_cost = %f", local_cost);
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
                // ROS_INFO("cycled");
            }
        }
        // add local curve's control points to the path
        if (best_local_waypoints.size()) {
            // ROS_INFO("A");
            ROS_INFO("p1 = (%f, %f), p2 = (%f, %f)", best_local_waypoints.at(1).pose.pose.position.x, best_local_waypoints.at(1).pose.pose.position.y, best_local_waypoints.at(2).pose.pose.position.x, best_local_waypoints.at(2).pose.pose.position.y);
            control_points.push_back(best_local_waypoints.at(1));
            // ROS_INFO("B");
            control_points.push_back(best_local_waypoints.at(2));
            // ROS_INFO("C");
        }
    }

    /* STITCH AND POPULATE BEZIER CURVES DESCRIBED BY THE ABOVE CONTROL POINTS TO FORM BEZIER PATH */
    ROS_INFO("Creating Bezier path");
    std::vector<Waypoint> bezier_path;
    createBezierPath(control_points, bezier_path);

    /* Print Bezier path -- for debugging */
    ROS_INFO("Bezier path (size = %ld):", bezier_path.size());
    for (int i = 0; i < bezier_path.size(); i++)
        ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* INTERPOLATE BEZIER PATH */
    // TODO: why does it not work? is it really necessary?
    // ROS_INFO("Interpolating Bezier path");
    // interpolateBezierPath(bezier_path, INTERPOLATION_SCALE);
    //
    // /* Print Bezier path -- for debugging */
    // ROS_INFO("Bezier path (size = %ld):", bezier_path.size());
    // for (int i = 0; i < bezier_path.size(); i++)
    //     ROS_INFO("(%f, %f)", bezier_path.at(i).pose.pose.position.x, bezier_path.at(i).pose.pose.position.y);

    /* SEND BEZIER PATH TO move_base */
    std::vector<Waypoint>::iterator iterator = bezier_path.begin();
    while (ros::ok() && iterator != bezier_path.end()) {
        goals_pub.publish(iterator->pose);
        if (first_time || ( move_base_status_msg.status_list.size() > 0 && move_base_status_msg.status_list[0].status != PENDING && move_base_status_msg.status_list[0].status != ACTIVE && move_base_status_msg.status_list[0].status != PREEMPTING) ) {
            goals_pub.publish(iterator->pose);
            first_time == false;
            if (!first_time) {
                iterator++;
                first_time = true;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

#endif
