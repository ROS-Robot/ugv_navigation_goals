#include "../include/header.hpp"

/* node's main function */
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on");

    first_time = true;
    // initialize feedback message's status
    // move_base_feedback_msg.status.status = LOST;

    // rulah_navigation_goals::RulahNavigationGoals RulahNavigationGoals(nodeHandle);
    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    // ros::Subscriber pose_sub = nodeHandle.subscribe("/pose", 1, &poseTopicCallback);
    // ros::Subscriber move_base_feedback_sub = nodeHandle.subscribe("/move_base/feedback", 1, &moveBaseFeedbackCallback);

    ros::Rate loop_rate(9.0);

    // publish initial pose
    geometry_msgs::PoseWithCovarianceStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.pose.position.x = -5.0; init.pose.pose.position.y = 6.0; init.pose.pose.position.z = 4.0;
    init.pose.pose.orientation.x = 0.0; init.pose.pose.orientation.y = 0.0; init.pose.pose.orientation.z = 0.0; init.pose.pose.orientation.w = 1.0;

    double x = -3.0, y = 6.5, angle = 45.0;
    ROS_INFO("Creating initial plan");
    while (x < GRANT_X) {
        x += 0.5;
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = x; goal.pose.position.y = y; goal.pose.position.z = 0.0;
        goal.pose.orientation = turnEulerAngleToQuaternion(angle);

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
        goals_list.push_back(goal);
    }

    ROS_INFO("We have the goals:");
    for (std::list<geometry_msgs::PoseStamped>::const_iterator iterator = goals_list.begin(); iterator != goals_list.end(); ++iterator)
        ROS_INFO("(p.x = %f, p.y = %f, p.z = %f), (o.x = %f, o.y = %f, o.z = %f, o.w = %f)",
                    iterator->pose.position.x, iterator->pose.position.y, iterator->pose.position.z, iterator->pose.orientation.x, iterator->pose.orientation.y, iterator->pose.orientation.z, iterator->pose.orientation.w);

    x = -3.0, y = 6.5, angle = 45.0;
    ROS_INFO("To loop");
    while (ros::ok() && x < GRANT_X) {
        // ROS_INFO("IN loop");
        // wait for the previous goal to finish
        // if (move_base_feedback_msg.status.status == SUCCEEDED) {
        // if (first_time || ( move_base_status_msg.status_list.size() > 0 && move_base_status_msg.status_list[0].status == SUCCEEDED) ) {
        if (first_time || ( move_base_status_msg.status_list.size() > 0 && move_base_status_msg.status_list[0].status != PENDING && move_base_status_msg.status_list[0].status != ACTIVE && move_base_status_msg.status_list[0].status != PREEMPTING) ) {
            // ROS_INFO("in IF");
            if (!first_time)
                x += 0.5;
            geometry_msgs::PoseStamped goal;
            goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
            goal.pose.position.x = x; goal.pose.position.y = y; goal.pose.position.z = 0.0;
            goal.pose.orientation = turnEulerAngleToQuaternion(angle);

            /* debugging */
            ROS_WARN("%f", angle);

            if (!first_time) {
                if (angle == 45) {
                    angle = 135;
                    y = 5.5;
                }
                else {
                    angle = 45;
                    y = 6.5;
                }
            }
            goals_pub.publish(goal);
        }
        // ROS_INFO("after if");
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ros::spin();
    return 0;
}
