#include "../include/header.hpp"

/* Bezier curves functions declarations */

/* calculate Bezier point of a quadratic Bezier curve */
void calculateBezierPoint(const float t, const geometry_msgs::Point p0, const geometry_msgs::Point p1, const geometry_msgs::Point p2, geometry_msgs::Point & p) {
    double one_minus_t = 1-t;
    p.x = one_minus_t*one_minus_t*p0.x + 2*one_minus_t*t*p1.x + t*t*p2.x;
    p.y = one_minus_t*one_minus_t*p0.y + 2*one_minus_t*t*p1.y + t*t*p2.y;
}

/* calculate segmentation points of a Bezier curve, in order to "form" it */
void formBezierCurve(const geometry_msgs::Point p0, const geometry_msgs::Point p1, const geometry_msgs::Point p2, std::list<Waypoint> & waypoints) {
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
