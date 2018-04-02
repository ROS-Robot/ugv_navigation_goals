#include "../include/header.hpp"

/* node's main function */
int main(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on");

    first_time = true;
    num_of_waypoints = 0;

    // initialize terrain
    terrain.goal.position.x = 1.5; terrain.goal.position.y = 6.0; terrain.start.position.x = -5.0; terrain.start.position.y = 6.0;
    terrain.goal_left.position.x = 1.5; terrain.goal_left.position.y = 7.0; terrain.start_left.position.x = -5.0; terrain.start_left.position.y = 7.0;
    terrain.goal_right.position.x = 1.5; terrain.goal_right.position.y = 5.0; terrain.start_right.position.x = -5.0; terrain.start_right.position.y = 5.0;
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
    geometry_msgs::PoseWithCovarianceStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.pose.position.x = terrain.start.position.x; init.pose.pose.position.y = terrain.start.position.y; init.pose.pose.position.z = 4.0;
    init.pose.pose.orientation.x = 0.0; init.pose.pose.orientation.y = 0.0; init.pose.pose.orientation.z = 0.0; init.pose.pose.orientation.w = 1.0;

    /* CREATE INITIAL PLAN */
    double x = -3.0, y = 6.5, angle = 45.0;
    ROS_INFO("Creating initial plan");
    while (x < GRANT_X) {
        num_of_waypoints++;
        x += 0.5;
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = x; goal.pose.position.y = y; goal.pose.position.z = 0.0;
        goal.pose.orientation = turnEulerAngleToQuaternion(angle);

        Waypoint temp(goal);
        temp.id = num_of_waypoints;
        temp.cost = 0.0;    // assignment at the iterator below
        temp.arc = 0;       // assignment at the iterator below
        temp.divertion = std::abs(y-init.pose.pose.position.y);
        temp.traversability = 0;        // TODO LATER
        temp.traversability_slope = 0;  // TODO LATER

        /* debugging */
        ROS_WARN("%f", angle);

        if (angle == 45) {
            angle = 135;
            y = 5.5;
        }
        else {
            angle = 45;
            y = 6.5;
        }
        waypoints_list.push_back(temp);
    }

    for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator) {
        if (std::next(iterator,2) != waypoints_list.end()) {
            iterator->arc = eulerAngleOf(std::next(iterator,1)->pose, iterator->pose, std::next(iterator,2)->pose);
        }
        if (iterator == waypoints_list.begin())
            iterator->cost = iterator->divertion;
        else
            iterator->cost = std::prev(iterator, 1)->cost + iterator->divertion;
    }

    ROS_INFO("We have the goals:");
    for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator)
        ROS_INFO("(p.x = %f, p.y = %f, p.z = %f), (o.x = %f, o.y = %f, o.z = %f, o.w = %f)",
                    iterator->pose.pose.position.x, iterator->pose.pose.position.y, iterator->pose.pose.position.z,
                    iterator->pose.pose.orientation.x, iterator->pose.pose.orientation.y, iterator->pose.pose.orientation.z, iterator->pose.pose.orientation.w);

    /* FORM OPTIMAL PLAN */
    generateOptimalPlan();

    /* GRADUALLY SEND PLAN TO move_base */
    x = -3.0, y = 6.5, angle = 45.0;
    std::list<Waypoint>::iterator iterator = waypoints_list.begin();
    ROS_INFO("To loop");
    // first_time = true;
    while (ros::ok() && iterator != waypoints_list.end()) {
        goals_pub.publish(iterator->pose);
        if (first_time || ( move_base_status_msg.status_list.size() > 0 && move_base_status_msg.status_list[0].status != PENDING && move_base_status_msg.status_list[0].status != ACTIVE && move_base_status_msg.status_list[0].status != PREEMPTING) ) {
            ROS_INFO("Publishing goal");
            goals_pub.publish(iterator->pose);
            first_time == false;
            if (!first_time) {
                ROS_INFO("iterator++");
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
