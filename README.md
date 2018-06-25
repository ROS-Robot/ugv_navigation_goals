# ugv_navigation_goals

## About

This is a ROS package comprising of the rulah_navigation_goals ROS node software.
It was developed at the Spring of 2018 by **Georgios Kamaras** as a complementary software 
to his undergraduate thesis in **Embodied Artificial Intelligence**. More specifically, 
ugv_navigation_goals consists of an _offline path planner_ for incline terrain navigation. 
This planner takes as input facts about a specific incline terrain region, like degree of 
slope, the robot's start position and its goal position and lethal obstacles that exist in 
this terrain region and outputs a vector of waypoints that have been deduced as the _recommended_ 
for the robot to traverse the incline terrain region with safety. This software package acts 
as a _proof of concept_ for its authors proposal regarding the utilization of Bezier curves 
to construct a smooth curved path from the robot's start position, on the start of the slope, 
to its goal position, significantly higher in the slope.

## Usage

### Prerequisites

* To use this software to its full extend, your robot should run on the ROS middleware.
* Prior to launching the rulah_navigation_goals node you should have already launched your robot's specific packages and especially its **move_base** node.

### Launch

Type:
> roslaunch rulah_navigation_goals rulah_navigation_goals.launch

## Contact & Feedback
* Georgios Kamaras