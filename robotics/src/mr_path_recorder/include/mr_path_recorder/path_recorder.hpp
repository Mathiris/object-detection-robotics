#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

#include "csv_writer.hpp"

class PathRecorder
{
public:
  PathRecorder(ros::NodeHandle& nh, const ros::NodeHandle& nh_p);

private:
  bool recordCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

  bool startRecord();
  bool stopRecord();

  void poseCB(const nav_msgs::Odometry::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;

  ros::ServiceServer srv_;
  std::string filename_;
  bool recording_;
  std::unique_ptr<ros::Subscriber> pose_sub_;

  std::unique_ptr<CSVWriter> csv_writter_;

  ros::Time last_record_;
};
