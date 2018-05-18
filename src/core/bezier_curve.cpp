#include "../../include/header.hpp"

/* Bezier curve functions declarations */

/* calculate Bezier point of a quadratic Bezier curve */
void calculateBezierPoint(const float & t, const geometry_msgs::Point & p0, const geometry_msgs::Point & p1, const geometry_msgs::Point & p2, geometry_msgs::Point & p) {
    double one_minus_t = 1-t;
    p.x = one_minus_t*one_minus_t*p0.x + 2*one_minus_t*t*p1.x + t*t*p2.x;
    p.y = one_minus_t*one_minus_t*p0.y + 2*one_minus_t*t*p1.y + t*t*p2.y;
    // p.z = heightAt(p) + 4.22;
}

/* calculate segmentation points of a Bezier curve, in order to "form" it */
void formBezierCurve(const geometry_msgs::Point & p0, const geometry_msgs::Point & p1, const geometry_msgs::Point & p2, std::vector<Waypoint> & bezier_curve) {
    Waypoint first;
    first.pose.pose.orientation.w = 1.0; first.pose.header.frame_id = "odom";
    first.pose.pose.position.x = p0.x;
    first.pose.pose.position.y = p0.y;
    bezier_curve.push_back(first);
    float t;
    for (int i = 1; i <= SEGMENTS_PER_CURVE; i++) {
        t = i / (float) SEGMENTS_PER_CURVE;
        Waypoint temp;
        temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
        calculateBezierPoint(t, p0, p1, p2, temp.pose.pose.position);
        bezier_curve.push_back(temp);
    }
    Waypoint last;
    last.pose.pose.orientation.w = 1.0; last.pose.header.frame_id = "odom";
    last.pose.pose.position.x = p2.x;
    last.pose.pose.position.y = p2.y;
    bezier_curve.push_back(last);
}

/* create a Bezier path, by stiching many Bezier curves together */
void createBezierPath(const std::vector<Waypoint> & control_points, std::vector<Waypoint> & bezier_path) {
    if (control_points.size() < 3)
        return;
    // ROS_WARN("createBezierPath in");

    /* Tweak for the final approach to goal. If the distance between the two control points is
        greater than the average distance between any control points plus the length of the vehicle, 
        then we will increase the segments per curve just for this one curve */
    double avg_distance = 0.0;

    for (int i = 0; i < control_points.size()-2; i += 2) {
        geometry_msgs::Point p0 = control_points.at(i).pose.pose.position;
        geometry_msgs::Point p1 = control_points.at(i+1).pose.pose.position;
        geometry_msgs::Point p2 = control_points.at(i+2).pose.pose.position;

        /* Tweak for the final approach to goal */
        double dist = distance(p0, p1);
        if (!avg_distance)
            avg_distance = dist;
        else
            avg_distance = (avg_distance + dist) / 2;

        /* Only do this for the first endpoint. When i != 0, this coincides
            with the end point of the previous segment */
        if (i == 0) {
            Waypoint temp;
            temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
            calculateBezierPoint(0, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }

        int segments = SEGMENTS_PER_CURVE;
        /* "override" for the last one, so we have an increase in the dynamically generated paths per request */
        if (dist > avg_distance + ROBOT_BODY_LENGTH)
            segments += segments * 2*ROBOT_BODY_LENGTH;     // 60% increase (determined experimentally)

        Waypoint temp;
        temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
        for (int j = 1; j <= segments; j++) {
            float t = j / (float) segments;
            calculateBezierPoint(t, p0, p1, p2, temp.pose.pose.position);

            /* Tweak to ensure that the is no way that the robot will try to move straight upwards.
                Note, though, that this tweak takes us a bit off the "mathematical" Bezier path. */
            if (bezier_path.size() > 1 && std::abs(temp.pose.pose.position.y - bezier_path.at(bezier_path.size()-1).pose.pose.position.y) < ROBOT_BODY_FIX) {
                // try to fix the problem
                bezier_path.at(bezier_path.size()-1).pose.pose.position.x -= ROBOT_BODY_LENGTH;
                bezier_path.at(bezier_path.size()-1).pose.pose.position.y -= ROBOT_BODY_LENGTH;
                // if this doesn't work, make another effort
                if (!isSafe(bezier_path.at(bezier_path.size()-1), temp)) {
                    // undo the previous fix
                    bezier_path.at(bezier_path.size()-1).pose.pose.position.x += ROBOT_BODY_LENGTH;
                    bezier_path.at(bezier_path.size()-1).pose.pose.position.y += ROBOT_BODY_LENGTH;
                    // retry to fix the problem
                    if (!isSafe(bezier_path.at(bezier_path.size()-1), temp)) {
                        // undo the previous fix
                        temp.pose.pose.position.x += ROBOT_BODY_LENGTH;
                        temp.pose.pose.position.y += ROBOT_BODY_LENGTH;
                        ROS_WARN("Dangerous passage between (%f, %f) and (%f, %f)", bezier_path.at(bezier_path.size()-1).pose.pose.position.x, bezier_path.at(bezier_path.size()-1).pose.pose.position.y, temp.pose.pose.position.x, temp.pose.pose.position.y);
                    }
                }
            }

            bezier_path.push_back(temp);
        }
    }
    // ROS_WARN("createBezierPath out");
}
void createBezierPath(const std::vector<Waypoint> & control_points, std::vector<Waypoint> & bezier_path, bool last_one) {
    if (control_points.size() < 3)
        return;
    // ROS_WARN("createBezierPath in");

    /* Tweak for the final approach to goal. If the distance between the two control points is
        greater than the average distance between any control points plus the length of the vehicle, 
        then we will increase the segments per curve just for this one curve */
    double avg_distance = 0.0;

    for (int i = 0; i < control_points.size()-2; i += 2) {
        geometry_msgs::Point p0 = control_points.at(i).pose.pose.position;
        geometry_msgs::Point p1 = control_points.at(i+1).pose.pose.position;
        geometry_msgs::Point p2 = control_points.at(i+2).pose.pose.position;

        /* Tweak for the final approach to goal */
        double dist = distance(p0, p1);
        if (!avg_distance)
            avg_distance = dist;
        else
            avg_distance = (avg_distance + dist) / 2;

        /* Only do this for the first endpoint. When i != 0, this coincides
            with the end point of the previous segment */
        if (i == 0) {
            Waypoint temp;
            temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
            calculateBezierPoint(0, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }

        int segments = SEGMENTS_PER_CURVE;
        /* "override" for the last one, so we have an increase in the dynamically generated paths per request */
        if (dist > avg_distance + ROBOT_BODY_LENGTH || last_one)
            segments += segments * 2*ROBOT_BODY_LENGTH;     // 60% increase (determined experimentally)

        Waypoint temp;
        temp.pose.pose.orientation.w = 1.0; temp.pose.header.frame_id = "odom";
        for (int j = 1; j <= segments; j++) {
            float t = j / (float) segments;
            calculateBezierPoint(t, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }
    }
    // ROS_WARN("createBezierPath out");
}

/* clean up a Bezier path from irrational sequences of waypoints that may have occured buring calculations */
void cleanUpBezierPath(std::vector<Waypoint> & bezier_path) {
    for (std::vector<Waypoint>::iterator it = bezier_path.begin(); it != bezier_path.end(); it++) {
        if ( it != bezier_path.begin() &&
                (std::prev(it,1)->pose.pose.position.x >= it->pose.pose.position.x ||
                 std::prev(it,1)->pose.pose.position.y == it->pose.pose.position.y)) {
            bezier_path.erase(it);
            it = std::prev(it,1);
        }
    }
}

/* interpolate a Bezier path */
void interpolateBezierPath(std::vector<Waypoint> & segments, float scale) {
    if (segments.size() <= 2)
        return;

    // ROS_WARN("interpolateBezierPath in");

    for (int i = 0; i < segments.size()-1; i++) {
        /* is first */
        if (i == 0) {
            geometry_msgs::Point p1 = segments.at(i).pose.pose.position;
            geometry_msgs::Point p2 = segments.at(i+1).pose.pose.position;

            geometry_msgs::Point tangent;
            tangent.x = p2.x - p1.x; tangent.y = p2.y - p1.y;
            geometry_msgs::Point q1;
            q1.x = p1.x + scale * tangent.x; q1.y = p1.y + scale * tangent.y;

            segments.at(i).pose.pose.position.x = p1.x; segments.at(i).pose.pose.position.y = p1.y;
            segments.at(i+1).pose.pose.position.x = q1.x; segments.at(i+1).pose.pose.position.y = q1.y;
        }
        /* is last */
        else if (i == segments.size()-1) {
            geometry_msgs::Point p0 = segments.at(i-1).pose.pose.position;
            geometry_msgs::Point p1 = segments.at(i).pose.pose.position;

            geometry_msgs::Point tangent;
            tangent.x = p1.x - p0.x; tangent.y = p1.y - p0.y;
            geometry_msgs::Point q0;
            q0.x = p1.x - scale * tangent.x; q0.y = p1.y - scale * tangent.y;

            segments.at(i-1).pose.pose.position.x = q0.x; segments.at(i-1).pose.pose.position.y = q0.y;
            segments.at(i).pose.pose.position.x = p1.x; segments.at(i).pose.pose.position.y = p1.y;
        }
        /* is anything else */
        else {
            geometry_msgs::Point p0 = segments.at(i-1).pose.pose.position;
            geometry_msgs::Point p1 = segments.at(i).pose.pose.position;
            geometry_msgs::Point p2 = segments.at(i+1).pose.pose.position;

            geometry_msgs::Point tangent;
            tangent.x = p2.x - p0.x; tangent.y = p2.y - p0.y;
            double p1_m_p0_magn = std::sqrt((p1.x-p0.x)*(p1.x-p0.x)+(p1.y-p0.y)*(p1.y-p0.y));
            double p2_m_p1_magn = std::sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y));
            geometry_msgs::Point q0;
            q0.x = p1.x - scale * tangent.x * p1_m_p0_magn; q0.y = p1.y - scale * tangent.y * p1_m_p0_magn;
            geometry_msgs::Point q1;
            q1.x = p1.x + scale * tangent.x * p2_m_p1_magn; q1.y = p1.y + scale * tangent.y * p2_m_p1_magn;

            segments.at(i-1).pose.pose.position.x = q0.x; segments.at(i-1).pose.pose.position.y = q0.y;
            segments.at(i).pose.pose.position.x = p1.x; segments.at(i).pose.pose.position.y = p1.y;
            segments.at(i+1).pose.pose.position.x = q1.x; segments.at(i+1).pose.pose.position.y = q1.y;
        }
    }
    // ROS_WARN("interpolateBezierPath out");
}

/* evaluate a Bezier curve */
double evaluateBezierCurve(std::vector<Waypoint> & control_points, bool & has_worst_local_cost) {
    // ROS_WARN("evaluateBezierCurve in");
    double cost = 0.0, s_norm_dev = 0.0, s_pitch = 0.0, s_yaw = 0.0, s_roll_neg = 0.0,
            s_roll_pos = 0.0, s_arc = 0.0;
            /* TODO: trade-offs discussion at final text*/
    for (std::vector<Waypoint>::iterator it = control_points.begin(); it != control_points.end(); ++it) {
        /* for debugging */
        // ROS_INFO("(p.x = %f, p.y = %f, p.z = %f), (o.x = %f, o.y = %f, o.z = %f, o.w = %f)",
        //             it->pose.pose.position.x, it->pose.pose.position.y, it->pose.pose.position.z,
        //             it->pose.pose.orientation.x, it->pose.pose.orientation.y, it->pose.pose.orientation.z, it->pose.pose.orientation.w);
        // ROS_INFO("deviation = %f, roll = %f, pitch = %f, yaw = %f, arc = %f, looking_right = %d", it->deviation, it->roll, it->pitch, it->yaw, it->arc, it->looking_right);

        it->cost = 0;
        // normalize deviation (between 0 and 1) and multiply by 100 to be like the angle values
        s_norm_dev += it->deviation / distance(terrain.start_left.position, terrain.start.position); it->cost += 100*it->deviation / distance(terrain.start_left.position, terrain.start.position);
        s_pitch += it->pitch; it->cost += 2.7*it->pitch;
        s_yaw += it->yaw; it->cost -= 1.3*it->yaw;
        /* TODO: fix roll, pitch, yaw signs */
        if ((it->looking_right && it->roll < 0) || (it->looking_right && it->roll > 0)) { // ((it->roll < 0 && it->yaw > 0) || (it->roll > 0 && it->yaw < 0))
            s_roll_pos += it->roll;     // roll that positively impacts the movement of the vehicle
            it->cost -= 2.7*it->roll;
        }
        else {
            s_roll_neg += it->roll;     // roll that negatively impacts the movement of the vehicle
            it->cost += 2.7*it->roll;
        }
        s_arc += it->arc;
        it->cost += 0.4*it->arc;

        if (it->cost > terrain.worst_local_cost) {
            terrain.worst_local_cost = it->cost;
            has_worst_local_cost = true;
        }

        // ROS_WARN("waypoint %d cost = %f", it->id, it->cost);
    }

    cost = (90/terrain.slope)*s_norm_dev + 10.0*s_pitch - 10.0*(s_roll_neg+s_roll_pos) /*2.7*s_yaw + 1.3*s_roll_neg - 1.3*s_roll_pos*/ + (terrain.slope/10)*s_arc;

    if (cost > terrain.worst_global_cost) terrain.worst_global_cost = cost;

    // ROS_WARN("evaluateBezierCurve out");

    return cost;
}
