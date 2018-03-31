#include "../include/header.hpp"

/* global variables */
std::list<geometry_msgs::PoseStamped> goals_list;
/* ROS messages */
move_base_msgs::MoveBaseActionFeedback move_base_feedback_msg;
actionlib_msgs::GoalStatusArray move_base_status_msg;
geometry_msgs::PoseWithCovarianceStamped pose_msg;
/* utility variables */
bool first_time;

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
void poseTopicCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ps_msg) {
    pose_msg = *ps_msg;
}

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

double turnQuaternionToEulerAngle(geometry_msgs::PoseStamped ps) {
    tf::Pose pose;
    tf::poseMsgToTF(ps.pose, pose);
    double yaw_angle = tf::getYaw(pose.getRotation());

    return yaw_angle;
}
