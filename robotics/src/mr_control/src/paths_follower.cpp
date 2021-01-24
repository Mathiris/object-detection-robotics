#include <dirent.h>
#include <sys/types.h>
#include <ostream>
#include <chrono>

#include <angles/angles.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <ros/package.h>

#include "mr_control/paths_follower.hpp"
#include "mr_control/csv_reader.hpp"

template <typename T1>
void yawToQuatRos(const double& yaw, T1& q) {
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(yaw * 0.5);
  q.w = cos(yaw * 0.5);
}

template <typename T1>
double quatRosToYaw(const T1& q) {
  // yaw (z-axis rotation)
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return atan2(siny_cosp, cosy_cosp);
}


std::pair<nav_msgs::Path, geometry_msgs::PoseArray> rosPathConverter(const std::vector<coords>& data)
{
  nav_msgs::Path path_to_publish;
  geometry_msgs::PoseArray pose_array_to_publish;
  path_to_publish.header.frame_id = "map";
  pose_array_to_publish.header.frame_id = "map";
  for (auto &&d : data)
  {
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = std::get<0>(d);
    ps.pose.position.y = std::get<1>(d);
    ps.pose.position.z = 0.0;
    yawToQuatRos(std::get<2>(d), ps.pose.orientation);
    path_to_publish.poses.push_back(ps);
    pose_array_to_publish.poses.push_back(ps.pose);
  }
  return {path_to_publish, pose_array_to_publish};
}

std::vector<coords> pathConverter(const std::vector<std::vector<std::string>> &raw_data)
{
  std::vector<coords> res(raw_data.size());
  std::size_t i =0;
  for (auto&& line : raw_data)
    res[i++] = {stod(line[0]), stod(line[1]), stod(line[2])};
  return res;
}



PathsFollower::PathsFollower(ros::NodeHandle &nh, const ros::NodeHandle &nh_p)
  : nh_(nh)
  , nh_p_(nh_p)
  , path_("")
  , odom_to_map_listener_(tfBuffer_)
  , roomba_length_(0.46)
  , roomba_radius_(0.1)
{ 
  timer_ = nh.createTimer(ros::Duration(0.1), &PathsFollower::controlLoop, this, false, false);
  srv_ = nh_p_.advertiseService("Goal", &PathsFollower::goalCB, this);
  stop_srv_ = nh_p_.advertiseService("Stop", &PathsFollower::stopCB, this);
  pub_left_ = nh_.advertise<std_msgs::Float64>("/gazebo/lwheel_traction_controller/command", 1);
  pub_right_ = nh_.advertise<std_msgs::Float64>("/gazebo/rwheel_traction_controller/command", 1);
  pub_point_ = nh_.advertise<geometry_msgs::PoseStamped>("/paths_follower/tracked_point", 1);
  pub_path_ = nh_.advertise<nav_msgs::Path>("/path", 1, true);
  pub_path_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/path_poses", 1, true);
  loadPath();
}

void PathsFollower::updateControlPose()
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


void PathsFollower::loadPath()
{
  auto path = ros::package::getPath("mr_control");
  std::ostringstream ss;
  ss << path << "/path/path_to_follow.path";
  ROS_INFO("Loading path from: %s", ss.str().c_str());
  path_ = ss.str();
  CSVReader reader{path_};
  path_to_follow_ = pathConverter(reader.getData());
  auto to_publish = rosPathConverter(path_to_follow_);
  pub_path_.publish(to_publish.first);
  pub_path_poses_.publish(to_publish.second);
}

bool PathsFollower::goalCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  updateControlPose();

  current_point_index_ = 1;
  init_time_ = ros::Time::now().toSec();
  prev_time_ = ros::Time::now().toSec();
  timer_.start();

  return true;
}

bool PathsFollower::stopCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  path_ = "";
  timer_.stop();
  std_msgs::Float64 stop;
  stop.data = 0.0;
  pub_left_.publish(stop);
  pub_right_.publish(stop);
  ROS_ERROR("aborting current path");
  return true;
}

void PathsFollower::updateCurrPoint(const coords roomba, const coords point, const float speed)
{
  float dist = sqrt(
    pow(std::get<0>(roomba) - std::get<0>(point), 2) +
    pow(std::get<1>(roomba) - std::get<1>(point), 2)
  );
  if (dist < roomba_length_ * (2 + speed / 4) && current_point_index_ < size(path_to_follow_)) {
    current_point_index_ += 1;
    // Publishing the new point to improve monitoring
    geometry_msgs::PoseStamped newPoint;
    coords np = path_to_follow_[current_point_index_];
    newPoint.pose.position.x = std::get<0>(np);
    newPoint.pose.position.y = std::get<1>(np);
    newPoint.pose.position.z = 0.0;
    yawToQuatRos(std::get<2>(np), newPoint.pose.orientation);
    newPoint.header.frame_id = "map";
    pub_point_.publish(newPoint);
  }
}

float PathsFollower::lateralError(const coords roomba, const coords point)
{
  const double c_yaw = std::cos(std::get<2>(point));
  const double s_yaw = std::sin(std::get<2>(point));
  const double dx = std::get<0>(roomba) - std::get<0>(point);
  const double dy = std::get<1>(roomba) - std::get<1>(point);
  return dx * s_yaw - dy * c_yaw;
}

float PathsFollower::angularError(const coords roomba, const coords point)
{
  return angles::normalize_angle(std::get<2>(roomba) - std::get<2>(point));
}

float PathsFollower::computeAngularSpeed(const coords roomba, const coords point)
{
  // PID Params
  const float Kp_lat = -1.0;
  const float Ki_lat = -2.0;
  const float Kp_ang = 3.0;
  const float Ki_ang = 2.0;

  // Lateral and angular errors
  float error_lat = lateralError(roomba, point);
  float error_ang = angularError(roomba, point);

  // Time delta
  float current_time = ros::Time::now().toSec();
  float delta_time = current_time - prev_time_;
  prev_time_ = current_time;

  // PID computation
  float PID_lat = Kp_lat * error_lat + Ki_lat * error_lat * delta_time;
  float PID_ang = Kp_ang * error_ang + Ki_ang * error_ang * delta_time;

  return PID_lat + PID_ang;
}

void PathsFollower::controlLoop(const ros::TimerEvent &event)
{
  updateControlPose(); // get localization in pose_

  // Init linear and angular speeds
  const float v = 2;
  float w = 0.0;

  // Fetch current point to reach
  coords curr_point = path_to_follow_[current_point_index_];
  updateCurrPoint(pose_, curr_point, v);

  // Angular speed computation
  w = computeAngularSpeed(pose_, curr_point);

  // Transform to roomba wheel speeds
  float phi1 = (2 * v + roomba_length_ * w) / (2 * roomba_radius_);
  float phi2 = (2 * v - roomba_length_ * w) / (2 * roomba_radius_);

  // Stop roomba on last point
  if (current_point_index_ == size(path_to_follow_)) {
    phi1 = 0;
    phi2 = 0;
    ROS_INFO_STREAM("Total time (in s): " << ros::Time::now().toSec() - init_time_);
  }

  std_msgs::Float64 data_left;
  data_left.data = phi1;
  std_msgs::Float64 data_right;
  data_right.data = phi2;
  pub_left_.publish(data_left);
  pub_right_.publish(data_right);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_follower");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");
  PathsFollower PathsFollower(nh, nh_p);

  ros::spin();

  return 0;
}
