#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class control {
  public:
    control(ros::NodeHandle& nh, const float& radius, const float& length);

    void callback(const geometry_msgs::Twist::ConstPtr& msg);

    ~control() = default;

  private:

    ros::NodeHandle nh_;

    ros::Publisher pub_left_;
    ros::Publisher pub_right_;

    ros::Subscriber sub_;
};
