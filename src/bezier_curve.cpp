#include "../include/header.hpp"

/* Bezier curves functions declarations */

/* calculate Bezier point of a quadratic Bezier curve */
void calculateBezierPoint(const float t, const geometry_msgs::Point p0, const geometry_msgs::Point p1, const geometry_msgs::Point p2, geometry_msgs::Point & p) {
    double one_minus_t = 1-t;
    p.x = one_minus_t*one_minus_t*p0.x + 2*one_minus_t*t*p1.x + t*t*p2.x;
    p.y = one_minus_t*one_minus_t*p0.y + 2*one_minus_t*t*p1.y + t*t*p2.y;
}

/* calculate segmentation points of a Bezier curve, in order to "form" it */
void formBezierCurve(const geometry_msgs::Point p0, const geometry_msgs::Point p1, const geometry_msgs::Point p2, std::vector<Waypoint> & waypoints) {
    Waypoint first;
    first.pose.pose.position.x = p0.x;
    first.pose.pose.position.y = p0.y;
    waypoints.push_back(first);
    float t;
    for (int i = 0; i <= SEGMENTS_PER_CURVE; i++) {
        t = i / (float) SEGMENTS_PER_CURVE;
        Waypoint temp;
        calculateBezierPoint(t, p0, p1, p2, temp.pose.pose.position);
        waypoints.push_back(temp);
    }
    Waypoint last;
    last.pose.pose.position.x = p0.x;
    last.pose.pose.position.y = p0.y;
    waypoints.push_back(last);
}

/* create a Bezier path, by stiching many Bezier curves together */
void createBezierPath(const std::vector<Waypoint> & control_points, std::vector<Waypoint> & bezier_path) {
    for (int i = 0; i < control_points.size(); i += 2) {
        geometry_msgs::Point p0 = control_points.at(i).pose.pose.position;
        geometry_msgs::Point p1 = control_points.at(i+1).pose.pose.position;
        geometry_msgs::Point p2 = control_points.at(i+2).pose.pose.position;

        /* Only do this for the first endpoint. When i != 0, this coincides
            with the end point of the previous segment */
        if (i == 0) {
            Waypoint temp;
            calculateBezierPoint(0, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }

        for (int j = 1; j <= SEGMENTS_PER_CURVE; j++) {
            float t = j / (float) SEGMENTS_PER_CURVE;
            Waypoint temp;
            calculateBezierPoint(t, p0, p1, p2, temp.pose.pose.position);
            bezier_path.push_back(temp);
        }
    }
}

/* interpolate a Bezier path */
void interpolateBezierPath(std::vector<Waypoint> & segments, float scale) {
    if (segments.size() <= 2)
        return;

    for (int i = 0; i < segments.size(); i++) {
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
}

/* evaluate a Bezier curve */
double evaluateBezierCurve(std::vector<Waypoint> & control_points) {
    return 0.0;
}

/* find some "good enough" Bezier control points greedily */
void greedyBezierControlPoints(std::vector<Waypoint> & control_points) {

}
