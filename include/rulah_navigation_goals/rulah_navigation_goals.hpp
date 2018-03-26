#pragma once

/* Our controller's main namespace */
namespace rulah_navigation_goals {
  /* the class of the Husky's high-level controller */
  class RulahNavigationGoals {
    /* class private variable-members */
    /* ROS communications */
    ros::NodeHandle nodeHandle_;
    // ros::Subscriber TODO;
    ros::Publisher goals_pub_;
    /* ROS messages */
    /* various variables */
    std::string goals_topic_;
    int queue_size_;

    /* class callback functions */

    /* class utility functions */

  public:
      /* class constructor */
      RulahNavigationGoals(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
          /* get parameters */
          if (!nodeHandle_.getParam("move_base_goals_topic", goals_topic_)) ROS_ERROR("Could not find move_base_goals_topic parameter!");
          if (!nodeHandle_.getParam("queue_size", queue_size_)) ROS_ERROR("Could not find queue_size parameter!");

          /* initialize member variables */

          /* advertise publishers to topics */
          goals_pub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>(goals_topic_, queue_size_);
          /* subscribe to topics */
      }

      /* class destructor */
      virtual ~RulahNavigationGoals() {}
  };
}
