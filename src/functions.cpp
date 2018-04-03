#include "../include/header.hpp"

/* global variables */
std::list<Waypoint> waypoints_list;
Terrain terrain;
/* ROS messages */
move_base_msgs::MoveBaseActionFeedback move_base_feedback_msg;
actionlib_msgs::GoalStatusArray move_base_status_msg;
geometry_msgs::PoseWithCovarianceStamped pose_msg;
/* utility variables */
bool first_time;
unsigned num_of_waypoints;

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

bool areCoLinear(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::PoseStamped & pose_c) {
    double x1 = pose_a.pose.position.x, y1 = pose_a.pose.position.y,
        x2 = pose_b.pose.position.x, y2 = pose_b.pose.position.y,
        x3 = pose_c.pose.position.x, y3 = pose_c.pose.position.y;

    return ((y2-y1)/(x2-x1)) == ((y3-y1)/(x3-x1));
}

/* returns the euler angle where pose_a is the vertex */
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::PoseStamped & pose_c) {
    double p_ab = std::sqrt((pose_a.pose.position.x-pose_b.pose.position.x)*(pose_a.pose.position.x-pose_b.pose.position.x) + (pose_a.pose.position.y-pose_b.pose.position.y)*(pose_a.pose.position.y-pose_b.pose.position.y)),
        p_bc = std::sqrt((pose_b.pose.position.x-pose_c.pose.position.x)*(pose_b.pose.position.x-pose_c.pose.position.x) + (pose_b.pose.position.y-pose_c.pose.position.y)*(pose_b.pose.position.y-pose_c.pose.position.y)),
        p_ac = std::sqrt((pose_a.pose.position.x-pose_c.pose.position.x)*(pose_a.pose.position.x-pose_c.pose.position.x) + (pose_a.pose.position.y-pose_c.pose.position.y)*(pose_a.pose.position.y-pose_c.pose.position.y));
    double res = std::acos((p_ab*p_ab+p_ac*p_ac-p_bc*p_bc)/(2*p_ab*p_ac));
    return res * 180.0 / PI;
}
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseWithCovarianceStamped& pose_b, const geometry_msgs::PoseStamped & pose_c) {
    double p_ab = std::sqrt((pose_a.pose.position.x-pose_b.pose.pose.position.x)*(pose_a.pose.position.x-pose_b.pose.pose.position.x) + (pose_a.pose.position.y-pose_b.pose.pose.position.y)*(pose_a.pose.position.y-pose_b.pose.pose.position.y)),
        p_bc = std::sqrt((pose_b.pose.pose.position.x-pose_c.pose.position.x)*(pose_b.pose.pose.position.x-pose_c.pose.position.x) + (pose_b.pose.pose.position.y-pose_c.pose.position.y)*(pose_b.pose.pose.position.y-pose_c.pose.position.y)),
        p_ac = std::sqrt((pose_a.pose.position.x-pose_c.pose.position.x)*(pose_a.pose.position.x-pose_c.pose.position.x) + (pose_a.pose.position.y-pose_c.pose.position.y)*(pose_a.pose.position.y-pose_c.pose.position.y));
    double res = std::acos((p_ab*p_ab+p_ac*p_ac-p_bc*p_bc)/(2*p_ab*p_ac));
    return res * 180.0 / PI;
}
double eulerAngleOf(const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b, const geometry_msgs::Pose & pose_c) {
    double p_ab = std::sqrt((pose_a.pose.position.x-pose_b.pose.position.x)*(pose_a.pose.position.x-pose_b.pose.position.x) + (pose_a.pose.position.y-pose_b.pose.position.y)*(pose_a.pose.position.y-pose_b.pose.position.y)),
        p_bc = std::sqrt((pose_b.pose.position.x-pose_c.position.x)*(pose_b.pose.position.x-pose_c.position.x) + (pose_b.pose.position.y-pose_c.position.y)*(pose_b.pose.position.y-pose_c.position.y)),
        p_ac = std::sqrt((pose_a.pose.position.x-pose_c.position.x)*(pose_a.pose.position.x-pose_c.position.x) + (pose_a.pose.position.y-pose_c.position.y)*(pose_a.pose.position.y-pose_c.position.y));
    double res = std::acos((p_ab*p_ab+p_ac*p_ac-p_bc*p_bc)/(2*p_ab*p_ac));
    return res * 180.0 / PI;
}

/* returns the outer product of AP and AB */
/* source: https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located */
double outerProduct(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b) {
    double d = (pose_p.pose.position.x-pose_a.pose.position.x)*(pose_b.pose.position.y-pose_a.pose.position.y)-(pose_p.pose.position.y-pose_a.pose.position.y)*(pose_b.pose.position.x-pose_a.pose.position.x);

    return d;
}

/* problem's core functions definitions */

/* generate optimal plan based on an initial set of waypoints */
void generateOptimalPlan() {
    // ROS_INFO("generateOptimalPlan in");
    if (!MAX_REPS_FOR_OPT) return;  // no optimization
    unsigned changes = 0, counter = 0;
    do {
        // eliminate routes that go through lethal obstacles
        for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator) {
            if (std::next(iterator,1) != waypoints_list.end() && throughLethalObstacle(*iterator, *(std::next(iterator,1)))) {
                closestBetterAlternative(*iterator);
                changes++;
            }
        }
        // eliminate routes that are inappropriate for the given problem
        for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); ++iterator) {
            if (std::next(iterator,1) != waypoints_list.end() && notGoodRoute(*iterator)) {
                closestBetterAlternative(*iterator);
                changes++;
            }
        }
        counter++;
    } while (changes && counter < MAX_REPS_FOR_OPT);
    if (counter == MAX_REPS_FOR_OPT) ROS_WARN("Max reps for optimization reached!");
    // ROS_INFO("generateOptimalPlan out");
}

/* is waypoint_a-->waypoint_b route going through a lethal obstacle? */
bool throughLethalObstacle(const Waypoint & waypoint_a, const Waypoint & waypoint_b) {
    // ROS_INFO("throughLethalObstacle in");
    for (std::vector<geometry_msgs::PoseStamped>::iterator iterator = terrain.lethal_obstacles.begin(); iterator != terrain.lethal_obstacles.end(); ++iterator)
        if (areCoLinear(waypoint_a.pose, waypoint_b.pose, *iterator))
            return true;
    // ROS_INFO("throughLethalObstacle out");
    return false;
}

/* is waypoint_a-->waypoint_b not a good route? */
bool notGoodRoute(const Waypoint & waypoint_a) {
    // ROS_INFO("notGoodRoute in");
    if (terrain.slope >= 45.0) {
        // TODO
        if (waypoint_a.arc >= 90.0 ||
            waypoint_a.deviation > 0.8*terrain.goal_left.position.y || waypoint_a.deviation > 0.8*terrain.goal_right.position.y ||
            waypoint_a.deviation > 0.8*terrain.start_left.position.y || waypoint_a.deviation > 0.8*terrain.start_right.position.y) {
                ROS_WARN("Waypoint %d is NOT GOOD", waypoint_a.id);
                return true;   // meaning that it IS a BAD route
            }
    }
    else {
        // TODO
        if (waypoint_a.arc < 90.0 ||
            waypoint_a.deviation > 0.8*terrain.goal_left.position.y || waypoint_a.deviation > 0.8*terrain.goal_right.position.y ||
            waypoint_a.deviation > 0.8*terrain.start_left.position.y || waypoint_a.deviation > 0.8*terrain.start_right.position.y) {
                ROS_WARN("Waypoint %d is NOT GOOD", waypoint_a.id);
                return true;   // meaning that it IS a BAD route
            }
    }
    // ROS_INFO("notGoodRoute out");
    return false;   // meaning that it IS a GOOD route
}

/* what is the closest better (relative to it's cost) alternative to waypoint_a? */
void closestBetterAlternative(const Waypoint & waypoint_a) {
    // generate a list of possible waypoint_a's alternatives

}
