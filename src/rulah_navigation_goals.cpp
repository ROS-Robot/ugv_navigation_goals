#include "../include/header.hpp"

/* node's main function */
int main(int argc, char *argv[]) {
    /* SET-UP */
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on");

    first_time = true;
    num_of_waypoints = 0;

    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    // ros::Subscriber pose_sub = nodeHandle.subscribe("/pose", 1, &poseTopicCallback);
    // ros::Subscriber move_base_feedback_sub = nodeHandle.subscribe("/move_base/feedback", 1, &moveBaseFeedbackCallback);
    ros::Rate loop_rate(9.0);

    /* PUBLISH INITIAL POSE */
    geometry_msgs::PoseWithCovarianceStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.pose.position.x = -5.0; init.pose.pose.position.y = 6.0; init.pose.pose.position.z = 4.0;
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
        temp.angle_with_next = 0;   // TODO
        temp.divertion = 0;         // TODO
        temp.traversability = 0;    // TODO LATER
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

    ROS_INFO("We have the goals:");
    for (std::list<Waypoint>::const_iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator)
        ROS_INFO("(p.x = %f, p.y = %f, p.z = %f), (o.x = %f, o.y = %f, o.z = %f, o.w = %f)",
                    iterator->pose.pose.position.x, iterator->pose.pose.position.y, iterator->pose.pose.position.z,
                    iterator->pose.pose.orientation.x, iterator->pose.pose.orientation.y, iterator->pose.pose.orientation.z, iterator->pose.pose.orientation.w);

    /* FORM OPTIMAL PLAN */
    generateOptimalPlan();

    /* GRADUALLY SEND PLAN TO move_base */
    x = -3.0, y = 6.5, angle = 45.0;
    std::list<Waypoint>::const_iterator iterator = waypoints_list.begin();
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
    // while (ros::ok() && x < GRANT_X) {
    //     // ROS_INFO("IN loop");
    //     // wait for the previous goal to finish
    //     // if (move_base_feedback_msg.status.status == SUCCEEDED) {
    //     // if (first_time || ( move_base_status_msg.status_list.size() > 0 && move_base_status_msg.status_list[0].status == SUCCEEDED) ) {
    //     if (first_time || ( move_base_status_msg.status_list.size() > 0 && move_base_status_msg.status_list[0].status != PENDING && move_base_status_msg.status_list[0].status != ACTIVE && move_base_status_msg.status_list[0].status != PREEMPTING) ) {
    //         // ROS_INFO("in IF");
    //         if (!first_time)
    //             x += 0.5;
    //         geometry_msgs::PoseStamped goal;
    //         goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
    //         goal.pose.position.x = x; goal.pose.position.y = y; goal.pose.position.z = 0.0;
    //         goal.pose.orientation = turnEulerAngleToQuaternion(angle);
    //
    //         /* debugging */
    //         ROS_WARN("%f", angle);
    //
    //         if (!first_time) {
    //             if (angle == 45) {
    //                 angle = 135;
    //                 y = 5.5;
    //             }
    //             else {
    //                 angle = 45;
    //                 y = 6.5;
    //             }
    //         }
    //         goals_pub.publish(goal);
    //     }
    //     // ROS_INFO("after if");
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    // ros::spin();
    return 0;
}
