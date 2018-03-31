#pragma once

/* C++ ROS libraries */
#include <ros/ros.h>
#include <ros/time.h>       // to calculate time between two messages at any platform
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include "actionlib_msgs/GoalStatusArray.h"
/* C++ utility libraries */
#include <list>

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

/* our waypoint's structure */
typedef struct waypoint {
    double cost;
    geometry_msgs::PoseStamped pose;
    double angle_with_next;
    double divertion;
    double traversability;          // TODO
    double traversability_slope;    // TODO
} waypoint;

/* global variables */
extern std::list<geometry_msgs::PoseStamped> goals_list;
/* ROS messages */
extern move_base_msgs::MoveBaseActionFeedback move_base_feedback_msg;
extern actionlib_msgs::GoalStatusArray move_base_status_msg;
extern geometry_msgs::PoseWithCovarianceStamped pose_msg;
/* utility variables */
extern bool first_time;

/* callback functions declarations */
void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg);
// void moveBaseFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedback_msg);
void poseTopicCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ps_msg);

/* utility functions declarations */
geometry_msgs::Quaternion turnEulerAngleToQuaternion(double theta);
double turnQuaternionToEulerAngle(geometry_msgs::PoseStamped pose);
