#pragma once

#include <memory>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <gazebo_msgs/ContactsState.h>

using coords = std::tuple<float, float, float>;

class Roomba
{
public:
  Roomba(ros::NodeHandle &nh, const ros::NodeHandle &nh_p);
  ~Roomba();

private:
  void controlLoop(const ros::TimerEvent &event);
  void updateControlPose();
  void cbBumper(const gazebo_msgs::ContactsState::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;

  ros::ServiceServer srv_;
  ros::ServiceServer stop_srv_;

  ros::Timer timer_;

  ros::Publisher pub_left_;
  ros::Publisher pub_right_;
  ros::Subscriber sub_bumper_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener odom_to_map_listener_;
  coords pose_;

  bool contact_;
};
