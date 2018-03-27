/* C++ ROS libraries */
#include <ros/ros.h>
#include <ros/time.h>       // to calculate time between two messages at any platform
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include "actionlib_msgs/GoalStatusArray.h"
/* C++ utility libraries */

/* useful definitions */
// actionlib_msgs/GoalStatus Message : uint8 status
#define PENDING 0
#define ACTIVE 1
#define PREEMPTED 2
#define SUCCEEDED 3
#define ABORTED 4
#define REJECTED 5
#define PREEMPTING 6
#define RECALLING 7
#define RECALLED 8
#define LOST 9
// the position of the "grant" goal in the field
#define GRANT_X 1.5
#define GRANT_Y 10.5    // 10m ahead of the robot's initial position
#define GRANT_Z 4.0     // same level with the robot in it's initial position

/* global variables */
move_base_msgs::MoveBaseActionFeedback move_base_feedback_msg;
actionlib_msgs::GoalStatusArray move_base_status_msg;
bool first_time = true;

/* callback functions declarations */
void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg);
// void moveBaseFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedback_msg);

/* utility functions declarations */
geometry_msgs::Quaternion turnEulerAngleToQuaternion(double theta);

/* node's main function */
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "rulah_navigation_goals");
    ros::NodeHandle nodeHandle("~");
    ROS_INFO("It's on");

    // initialize feedback message's status
    // move_base_feedback_msg.status.status = LOST;

    // rulah_navigation_goals::RulahNavigationGoals RulahNavigationGoals(nodeHandle);
    ros::Publisher goals_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Publisher init_pose_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("initialpose", 1);
    ros::Subscriber move_base_status_sub = nodeHandle.subscribe("/move_base/status", 1, &moveBaseStatusCallback);
    // ros::Subscriber move_base_feedback_sub = nodeHandle.subscribe("/move_base/feedback", 1, &moveBaseFeedbackCallback);

    ros::Rate loop_rate(9.0);

    // publish initial pose
    geometry_msgs::PoseWithCovarianceStamped init;
    init.header.stamp = ros::Time::now(); init.header.frame_id = "odom";
    init.pose.pose.position.x = -5.0; init.pose.pose.position.y = 6.0; init.pose.pose.position.z = 4.0;
    init.pose.pose.orientation.x = 0.0; init.pose.pose.orientation.y = 0.0; init.pose.pose.orientation.z = 0.0; init.pose.pose.orientation.w = 1.0;

    double x = -3.0, y = 6.5, angle = 45.0;
    ROS_INFO("To loop");
    while (ros::ok() && x < GRANT_X) {
        ROS_INFO("IN loop");
        // wait for the previous goal to finish
        // if (move_base_feedback_msg.status.status == SUCCEEDED) {
        if (first_time || ( move_base_status_msg.status_list.size() > 0 && move_base_status_msg.status_list[0].status == SUCCEEDED) ) {
            ROS_INFO("in IF");
            // if (move_base_feedback_msg.status.status == LOST) x--;    // send previous goal if it was lost
            // if (x > 100)  break;
            if (!first_time)
                x += 2.0;
            geometry_msgs::PoseStamped goal;
            goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
            goal.pose.position.x = x; goal.pose.position.y = y; goal.pose.position.z = 0.0;
            // goal.pose.orientation.x = 0.0; goal.pose.orientation.y = 0.0; goal.pose.orientation.z = 0.0; goal.pose.orientation.w = 1.0;
            goal.pose.orientation = turnEulerAngleToQuaternion(angle);

            if (!first_time){
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
        ROS_INFO("after if");
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ros::spin();
    return 0;
}

/* callback functions definitions */
void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg) {
    if (status_msg->status_list.size() > 0) {
        first_time = false;
        move_base_status_msg = *status_msg;
    }
}
// void moveBaseFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedback_msg) {
//     move_base_feedback_msg = *feedback_msg;
// }

/* utility functions definitions */
geometry_msgs::Quaternion turnEulerAngleToQuaternion(double theta) {
    // double theta = 90.0;
    double radians = theta * (M_PI/180);

    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);
    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);
    qMsg.w = 1.0;

    return qMsg;
}
