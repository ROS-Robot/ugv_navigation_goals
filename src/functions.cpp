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
double outerProduct(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::Pose & pose_a, const geometry_msgs::Pose & pose_b) {
    double d = (pose_p.pose.position.x-pose_a.position.x)*(pose_b.position.y-pose_a.position.y)-(pose_p.pose.position.y-pose_a.position.y)*(pose_b.position.x-pose_a.position.x);

    return d;
}
/* returns the distance of P from the line defined by A and B */
/* source: wikipedia, Distance_from_a_point_to_a_line */
double distanceFromLine(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::PoseStamped & pose_a, const geometry_msgs::PoseStamped & pose_b) {
    double d = std::abs((pose_b.pose.position.y-pose_a.pose.position.y)*pose_p.pose.position.x-(pose_b.pose.position.x-pose_a.pose.position.x)*pose_p.pose.position.y+pose_b.pose.position.x*pose_a.pose.position.y-pose_b.pose.position.y*pose_a.pose.position.x)
                /
                std::sqrt((pose_b.pose.position.y-pose_a.pose.position.y)*(pose_b.pose.position.y-pose_a.pose.position.y)+(pose_b.pose.position.x-pose_a.pose.position.x)*(pose_b.pose.position.x-pose_a.pose.position.x));

    return d;
}
double distanceFromLine(const geometry_msgs::PoseStamped & pose_p, const geometry_msgs::Pose & pose_a, const geometry_msgs::Pose & pose_b) {
    double d = std::abs((pose_b.position.y-pose_a.position.y)*pose_p.pose.position.x-(pose_b.position.x-pose_a.position.x)*pose_p.pose.position.y+pose_b.position.x*pose_a.position.y-pose_b.position.y*pose_a.position.x)
                /
                std::sqrt((pose_b.position.y-pose_a.position.y)*(pose_b.position.y-pose_a.position.y)+(pose_b.position.x-pose_a.position.x)*(pose_b.position.x-pose_a.position.x));

    return d;
}

/* returns the distance between two points */
double distance(const geometry_msgs::Point & p_a, const geometry_msgs::Point & p_b) {
    double d = std::sqrt((p_a.x-p_b.x)*(p_a.x-p_b.x)+(p_a.y-p_b.y)*(p_a.y-p_b.y));
    return d;
}

/* problem's core functions definitions */

/* generate optimal plan based on an initial set of waypoints */
void generateOptimalPlan() {
    // ROS_INFO("generateOptimalPlan in");
    if (!MAX_REPS_FOR_OPT) return;  // no optimization
    unsigned changes = 0, counter = 0;
    do {
        // ROS_INFO("A");
        // eliminate routes that go through lethal obstacles
        for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); iterator++) {
            if (std::next(iterator,1) != waypoints_list.end() && throughLethalObstacle(*iterator, *(std::next(iterator,1)))) {
                Waypoint temp = closestBetterAlternative(*iterator, *(std::next(iterator,1)));
                changes++;
            }
        }
        // ROS_INFO("B");
        // eliminate routes that are inappropriate for the given problem
        for (std::list<Waypoint>::iterator iterator = waypoints_list.begin(); iterator != waypoints_list.end(); iterator++) {
            if (std::next(iterator,1) != waypoints_list.end() && notGoodRoute(*iterator)) {
                Waypoint temp = closestBetterAlternative(*iterator, *(std::next(iterator,1)));
                changes++;
            }
        }
        // ROS_INFO("C");
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
Waypoint closestBetterAlternative(const Waypoint & waypoint_a, const Waypoint & waypoint_b) {
    // ROS_INFO("closestBetterAlternative in");
    // generate a list of all viable waypoint_a's alternatives
    std::vector<Waypoint> alternatives;
    Waypoint tempw;
    int viable = 0;
    // north
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x+1; goal.pose.position.y = waypoint_a.pose.pose.position.y;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;
        tempw.traversability = 0;        // TODO LATER
        tempw.traversability_slope = 0;  // TODO LATER

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line and TODO*/
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // east
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x; goal.pose.position.y = waypoint_a.pose.pose.position.y-1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;
        tempw.traversability = 0;        // TODO LATER
        tempw.traversability_slope = 0;  // TODO LATER

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line and TODO*/
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // south
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x-1; goal.pose.position.y = waypoint_a.pose.pose.position.y;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;
        tempw.traversability = 0;        // TODO LATER
        tempw.traversability_slope = 0;  // TODO LATER

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line and TODO*/
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // west
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x; goal.pose.position.y = waypoint_a.pose.pose.position.y+1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;
        tempw.traversability = 0;        // TODO LATER
        tempw.traversability_slope = 0;  // TODO LATER

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line and TODO*/
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // north-east
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x+1; goal.pose.position.y = waypoint_a.pose.pose.position.y-1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;
        tempw.traversability = 0;        // TODO LATER
        tempw.traversability_slope = 0;  // TODO LATER

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line and TODO*/
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // north-west
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x+1; goal.pose.position.y = waypoint_a.pose.pose.position.y+1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;
        tempw.traversability = 0;        // TODO LATER
        tempw.traversability_slope = 0;  // TODO LATER

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line and TODO*/
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // south-east
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x-1; goal.pose.position.y = waypoint_a.pose.pose.position.y-1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;
        tempw.traversability = 0;        // TODO LATER
        tempw.traversability_slope = 0;  // TODO LATER

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line and TODO*/
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }
    // south-west
    if (viable < MAX_VIABLE_ALT) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now(); goal.header.frame_id = "odom";
        goal.pose.position.x = waypoint_a.pose.pose.position.x-1; goal.pose.position.y = waypoint_a.pose.pose.position.y+1;
        goal.pose.orientation.w = 1.0;

        tempw.pose = goal;
        tempw.id = num_of_waypoints;
        tempw.arc = 0;
        // distance from the straight line that connects the start with the goal (finish)
        tempw.deviation = distanceFromLine(goal, terrain.start, terrain.goal);
        tempw.cost = waypoint_a.cost-waypoint_a.deviation+tempw.deviation;
        tempw.traversability = 0;        // TODO LATER
        tempw.traversability_slope = 0;  // TODO LATER

        /* in order of appearance, we want our new waypoint:
            not to be leading us at a lethal obstacle, to be on the right of the left border of the field, to be on the left of the right border of the field
            to be below the finish line, to be above the start line and TODO*/
        if (!throughLethalObstacle(tempw, waypoint_b) && outerProduct(goal, terrain.goal_left, terrain.start_left) > 0 &&
            outerProduct(goal, terrain.goal_right, terrain.start_right) < 0 && outerProduct(goal, terrain.goal_left, terrain.goal_right) > 0 &&
            outerProduct(goal, terrain.start_left, terrain.start_right) < 0) {
            alternatives.push_back(tempw);
            viable++;
        }
    }

    // find the alternative with the lowest cost
    std::vector<Waypoint>::iterator best_it = alternatives.end();
    double best_cost = waypoint_a.cost;
    for (std::vector<Waypoint>::iterator i = alternatives.begin(); i != alternatives.end(); i++) {
        if (i->cost < best_cost) {
            best_cost = i->cost;
            best_it = i;
        }
    }
    // ROS_INFO("closestBetterAlternative out");
    if (best_it == alternatives.end()) return waypoint_a;
    return *best_it;
}


/* is a waypoint admissible for path planning?
 * w_c: the candidate-waypoint, w_f: it's previous, currently fixed, waypoint */
bool isAdmissible(Waypoint & w_c, const Waypoint & w_f){
    /* in order of appearance, we want w_c:
        not to be leading us at a lethal obstacle in order to reach w_f, to be on the right of the left border of the field,
        to be on the left of the right border of the field to be below the finish line, to be above the start line */
    if ( throughLethalObstacle(w_c, w_f) || !outerProduct(w_c.pose, terrain.goal_left, terrain.start_left) > 0 ||
        !outerProduct(w_c.pose, terrain.goal_right, terrain.start_right) < 0 || !outerProduct(w_c.pose, terrain.goal_left, terrain.goal_right) > 0 ||
        !outerProduct(w_c.pose, terrain.start_left, terrain.start_right) < 0 ) {
        return false;
    }

    /* we also don't want the pitch at w_c to be greater than the pitch in terrain.start */
    if (pitchAt(w_c) > pitchAt(terrain.start.position))
        return false;

    return true;    // we reached so far, we have an admissible waypoint
}

/* evaluate a given plan (a vector of waypoints) as a possible solution */
double evaluate(const std::vector<Waypoint> plan) {
    double score = 0.0, s_dev = 0.0, s_pitch = 0.0, s_yaw = 0.0, s_roll_neg = 0.0,
            s_roll_pos = 0.0, s_arc = 0.0;

    for (std::vector<Waypoint>::const_iterator it = plan.begin(); it != plan.end(); ++it) {
            s_dev += it->deviation;
            s_pitch += it->pitch;
            s_yaw += it->yaw;
            /* TODO: the roll thing... */
            if (1)
                s_roll_pos += it->roll;
            else
                s_roll_neg += it->roll;
            s_arc += it->arc;
    }

    /* TODO: perhaps add weights for normalization? */
    score = s_dev + s_pitch - s_yaw + s_roll_neg - s_roll_pos + s_arc;

    return score;
}

/* calculate the pitch of the platform at a certain position */
double pitchAt(Waypoint & w) {
    double pitch = 0.0, x_3 = 0.0, x_2 = 0.0, sinTheta = 0.0, sinThetaP = 0.0;
    geometry_msgs::Point p = w.pose.pose.position;

    /* if the platform is looking towards goal_right */
    if (w.arc <= 90) {
        x_3 = std::abs(p.x-terrain.goal_right.position.x);
        x_2 = distance(p, terrain.goal_right.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        sinThetaP = sinTheta * x_3 / x_2;
    }
    /* if the platform is looking towards goal_left */
    else {
        x_3 = std::abs(p.x-terrain.goal_left.position.x);
        x_2 = distance(p, terrain.goal_left.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        sinThetaP = sinTheta * x_3 / x_2;
    }

    pitch = std::asin(sinThetaP);
    w.pitch = pitch;

    return pitch;
}

double pitchAt(const geometry_msgs::Point & p) {
    double pitch = 0.0, x_3 = 0.0, x_2 = 0.0, sinTheta = 0.0, sinThetaP = 0.0;

    x_3 = std::abs(p.x-terrain.goal_right.position.x);
    x_2 = distance(p, terrain.goal_right.position);
    sinTheta = std::sin(terrain.slope*PI/180.0);
    sinThetaP = sinTheta * x_3 / x_2;

    pitch = std::asin(sinThetaP);

    return pitch;
}

/* calculate the roll of the platform at a certain position */
double rollAt(Waypoint & w) {
    double  roll = 0.0, pitch = 0.0, x_3 = 0.0, x_2 = 0.0,
            sinTheta = 0.0, sinThetaP = 0.0, cosTheta0 = 0.0,
            theta0 = 0.0, sinThetaR = 0.0;
    geometry_msgs::Point p = w.pose.pose.position;

    /* if the platform is looking towards goal_right */
    if (w.arc <= 90) {
        x_3 = std::abs(p.x-terrain.goal_right.position.x);
        x_2 = distance(p, terrain.goal_right.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        sinThetaP = sinTheta * x_3 / x_2;
    }
    /* if the platform is looking towards goal_left */
    else {
        x_3 = std::abs(p.x-terrain.goal_left.position.x);
        x_2 = distance(p, terrain.goal_left.position);
        sinTheta = std::sin(terrain.slope*PI/180.0);
        sinThetaP = sinTheta * x_3 / x_2;
    }

    pitch = std::asin(sinThetaP);
    w.pitch = pitch;

    cosTheta0 = sinThetaP / sinTheta;
    theta0 = std::acos(cosTheta0);
    sinThetaR = sinThetaP / std::tan(90 - theta0*PI/180.0);
    roll = std::asin(sinThetaR);
    w.roll = roll;

    return roll;
}

double rollAt(const geometry_msgs::Point & p) {
    double  roll = 0.0, pitch = 0.0, x_3 = 0.0, x_2 = 0.0,
            sinTheta = 0.0, sinThetaP = 0.0, cosTheta0 = 0.0,
            theta0 = 0.0, sinThetaR = 0.0;

    x_3 = std::abs(p.x-terrain.goal_right.position.x);
    x_2 = distance(p, terrain.goal_right.position);
    sinTheta = std::sin(terrain.slope*PI/180.0);
    sinThetaP = sinTheta * x_3 / x_2;

    pitch = std::asin(sinThetaP);

    cosTheta0 = sinThetaP / sinTheta;
    theta0 = std::acos(cosTheta0);
    sinThetaR = sinThetaP / std::tan(90 - theta0*PI/180.0);
    roll = std::asin(sinThetaR);

    return roll;
}
