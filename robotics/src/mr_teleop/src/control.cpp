#include <std_msgs/Float64.h>

#include "mr_teleop/control.hpp"

control::control(ros::NodeHandle& nh, const float& radius, const float& length)
  : nh_(nh){

  sub_ = nh_.subscribe("cmd_vel", 1000, &control::callback, this);
  pub_left_ = nh_.advertise<std_msgs::Float64>("/gazebo/rwheel_traction_controller/command", 1);
  pub_right_ = nh_.advertise<std_msgs::Float64>("/gazebo/lwheel_traction_controller/command", 1);
}

void control::callback(const geometry_msgs::Twist::ConstPtr& msg){
  // TODO compute the speeds
  
  std_msgs::Float64 data_left;
  data_left.data = 0; // TODO fill with your data
  std_msgs::Float64 data_right;
  data_right.data = 0; // TODO fill with your data

  pub_left_.publish(data_left);
  pub_right_.publish(data_right);
}
