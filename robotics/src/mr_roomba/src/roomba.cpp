#include <dirent.h>
#include <sys/types.h>
#include <ostream>
#include <chrono>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <ros/package.h>

#include "mr_roomba/roomba.hpp"

template <class T1>
double quatRosToYaw(const T1& q) {
  // yaw (z-axis rotation)
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return atan2(siny_cosp, cosy_cosp);
}



Roomba::Roomba(ros::NodeHandle &nh, const ros::NodeHandle &nh_p)
  : nh_(nh)
  , nh_p_(nh_p)
  , odom_to_map_listener_(tfBuffer_)
  , contact_(false)
{ 
  timer_ = nh.createTimer(ros::Duration(0.1), &Roomba::controlLoop, this, false, false);
  pub_left_ = nh_.advertise<std_msgs::Float64>("/gazebo/lwheel_traction_controller/command", 1);
  pub_right_ = nh_.advertise<std_msgs::Float64>("/gazebo/rwheel_traction_controller/command", 1);
  sub_bumper_ = nh_.subscribe("/bumper_vals", 1, &Roomba::cbBumper, this);
  timer_.start();
}

Roomba::~Roomba(){
  timer_.stop();
  std_msgs::Float64 stop;
  stop.data = 0.0;
  pub_left_.publish(stop);
  pub_right_.publish(stop);
}

void Roomba::cbBumper(const gazebo_msgs::ContactsState::ConstPtr& msg){
  contact_ = (msg->states.size() != 0);
}

void Roomba::updateControlPose()
{
  try
  {
    auto &&ts = tfBuffer_.lookupTransform("map", "base_link", ros::Time(0));
    pose_= {ts.transform.translation.x, ts.transform.translation.y, quatRosToYaw(ts.transform.rotation)};
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
}


void Roomba::controlLoop(const ros::TimerEvent &event)
{
  updateControlPose(); // get localization in pose_

  //TODO behavior



  std_msgs::Float64 data_left;
  data_left.data = 0;
  std_msgs::Float64 data_right;
  data_right.data = 0;
  // pub_left_.publish(data_left);
  // pub_right_.publish(data_right);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "roomba");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");
  Roomba Roomba(nh, nh_p);

  ros::spin();

  return 0;
}
