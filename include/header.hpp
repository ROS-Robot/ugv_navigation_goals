#pragma once

/* C++ ROS libraries */
#include <ros/ros.h>
#include <ros/time.h>       // to calculate time between two messages at any platform
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include "actionlib_msgs/GoalStatusArray.h"
/* Grid Map */
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMapInfo.h>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>
/* C++ utility libraries */
#include <list>
#include <math.h>       /* acos */

/* useful definitions */
#define PI 3.14159265
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
#define MAX_VIABLE_ALT 4    // find up to 4 viable alternative waypoints when needed
#define MIN_JUMP 0.2    // the minimum jump in space when looking for an alternative waypoint
#define MAX_REPS_FOR_OPT 20 // maximum number of optimization repetitions

/* our waypoint's structure */
class Waypoint {
public:
    unsigned id;
    double cost;
    geometry_msgs::PoseStamped pose;
    double arc;                     // the arc which has as an edge the current waypoint and is formed with it, it's previous and it's next waypoints
    double deviation;
    double traversability;          // TODO
    double traversability_slope;    // TODO

    Waypoint() {};
    Waypoint(geometry_msgs::PoseStamped goal) {
        this->pose.header = goal.header;
        this->pose.pose = goal.pose;
    };
    ~Waypoint() {};
};

/* our problem's terrain structure */
class Terrain {
public:
    geometry_msgs::Pose start;
    geometry_msgs::Pose goal;
    geometry_msgs::Pose goal_left;
    geometry_msgs::Pose goal_right;
    geometry_msgs::Pose start_left;
    geometry_msgs::Pose start_right;
    double goal_altitude;                   // TODO
    double slope;
    std::vector<geometry_msgs::PoseStamped>  lethal_obstacles;
    grid_map::GridMap travers_map;          // TODO
    grid_map::GridMap travers_slope_map;    // TODO

    Terrain() {};
    Terrain(std::vector<geometry_msgs::PoseStamped> lethal_obstacles) : lethal_obstacles(lethal_obstacles) {};
    ~Terrain() {};
};

/* global variables */
extern std::list<Waypoint> waypoints_list;
extern Terrain terrain;
/* ROS messages */
extern move_base_msgs::MoveBaseActionFeedback move_base_feedback_msg;
extern actionlib_msgs::GoalStatusArray move_base_status_msg;
extern geometry_msgs::PoseWithCovarianceStamped pose_msg;
/* utility variables */
extern bool first_time;
extern unsigned num_of_waypoints;

/* callback functions declarations */

void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg);
// void moveBaseFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedback_msg);
void poseTopicCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ps_msg);

/* utility functions declarations */

geometry_msgs::Quaternion turnEulerAngleToQuaternion(double theta);
double turnQuaternionToEulerAngle(geometry_msgs::PoseStamped pose);
bool areCoLinear(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::PoseStamped & pose_c);
/* returns the euler angle where pose_a is the vertex */
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::PoseStamped & pose_c);
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseWithCovarianceStamped& pose_b, const geometry_msgs::PoseStamped & pose_c);
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::Pose & pose_c);
/* returns the outer product of AP and AB */
/* source: https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located */
double outerProduct(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b);
double outerProduct(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::Pose & pose_a, const geometry_msgs::Pose & pose_b);
/* returns the distance of P from the line defined by A and B */
/* source: wikipedia, Distance_from_a_point_to_a_line */
double distanceFromLine(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b);
double distanceFromLine(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::Pose & pose_a, const geometry_msgs::Pose & pose_b);

/* problem's core functions declarations */

/* generate optimal plan based on an initial set of waypoints */
void generateOptimalPlan();
/* is waypoint_a-->waypoint_b route going through a lethal obstacle? */
bool throughLethalObstacle(const Waypoint & waypoint_a, const Waypoint & waypoint_b);
/* is waypoint_a-->waypoint_b not a good route? */
bool notGoodRoute(const Waypoint & waypoint_a);
/* what is the closest better (relative to it's cost) alternative to waypoint_a? */
Waypoint closestBetterAlternative(const Waypoint & waypoint_a, const Waypoint & waypoint_b);
