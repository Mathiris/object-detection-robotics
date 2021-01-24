#pragma once

#include <memory>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>

#include "mr_control/Goal.h"

using coords = std::tuple<float, float, float>;

class PathsFollower
{
public:
  PathsFollower(ros::NodeHandle &nh, const ros::NodeHandle &nh_p);

private:
  void loadPath();
  void controlLoop(const ros::TimerEvent &event);
  bool goalCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool stopCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  void updateControlPose();
  void cb_speed_ratio(const geometry_msgs::Twist::ConstPtr &msg);
  void updateCurrPoint(const coords roomba, const coords point, const float speed);
  float lateralError(const coords roomba, const coords point);
  float angularError(const coords roomba, const coords point);
  float computeAngularSpeed(const coords roomba, const coords point);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;

  float max_tolerance_goal_norm_;
  float max_tolerance_goal_angle_;

  float speed_ratio_;

  // full filename and starting point
  std::string path_;
  std::vector<coords> path_to_follow_;

  float roomba_length_;
  float roomba_radius_;

  int current_point_index_;
  float init_time_;
  float prev_time_;

  ros::ServiceServer srv_;
  ros::ServiceServer stop_srv_;

  ros::Timer timer_;

  ros::Publisher pub_left_;
  ros::Publisher pub_right_;
  ros::Publisher pub_point_;
  ros::Publisher pub_path_;
  ros::Publisher pub_path_poses_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener odom_to_map_listener_;
  coords pose_;
};
