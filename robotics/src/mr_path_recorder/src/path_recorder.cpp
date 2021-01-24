#include "mr_path_recorder/path_recorder.hpp"

#include <tf/transform_datatypes.h>
#include <time.h>

PathRecorder::PathRecorder(ros::NodeHandle &nh, const ros::NodeHandle &nh_p)
    : nh_(nh), nh_p_(nh_p), recording_(false), filename_(""), pose_sub_()
{
  srv_ = nh_.advertiseService("record", &PathRecorder::recordCB, this);
  last_record_ = ros::Time::now();
}

bool PathRecorder::recordCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
  if (recording_)
  {
    if (stopRecord())
    {
      resp.success = true;
      resp.message = "Successfully recorded " + filename_;
    }
    else
    {
      resp.success = false;
      resp.message =
          "Failed to stop the record in file " + filename_ + " . Please check that nothing else is editing this file";
    }
  }
  else
  {
    if (startRecord())
    {
      resp.success = true;
      resp.message = "Start recording: " + filename_;
    }
    else
    {
      resp.success = false;
      resp.message = "failed to start record in file " + filename_ + " .Please check that the path exist";
    }
  }
  return true;
}

bool PathRecorder::startRecord()
{
  recording_ = true;
  std::string date_time = currentDateTime();
  filename_ = date_time.insert(0, "/tmp/gps_record_");
  csv_writter_ = std::make_unique<CSVWriter>(filename_);
  try
  {
    pose_sub_ = std::make_unique<ros::Subscriber>(nh_.subscribe("/odom", 1, &PathRecorder::poseCB, this));
  }
  catch (const std::exception &ex)
  {
    std::cout << "Exception was thrown: " << ex.what() << std::endl;
    return false;
  }
  return true;
}

bool PathRecorder::stopRecord()
{
  recording_ = false;
  pose_sub_.reset();
  try
  {
    csv_writter_.reset();
  }
  catch (const std::exception &ex)
  {
    std::cout << "Exception was thrown: " << ex.what() << std::endl;
    return false;
  }
  return true;
}

void PathRecorder::poseCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  auto now = ros::Time::now();
  if((now - last_record_).toSec() > 1.0){
    *csv_writter_ << msg->pose.pose.position.x << msg->pose.pose.position.y << tf::getYaw(msg->pose.pose.orientation) << endrow;
    last_record_ = now;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_recorder");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");
  PathRecorder path_recorder(nh, nh_p);

  ros::spin();

  return 0;
}
