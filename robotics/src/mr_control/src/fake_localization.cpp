#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class OdomTf {
  public:
    OdomTf();
    void getInfo(const nav_msgs::Odometry::ConstPtr& msg);
    ros::Time          current_time;
    void               publishOdom();
    nav_msgs::Odometry odom;

  private:
    ros::NodeHandle                 nh_;
    ros::Subscriber                 data_sub_;
    ros::Publisher                  data_pub_;
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster        odom_broadcaster;
};

OdomTf::OdomTf() {
    data_sub_ = nh_.subscribe("odom", 1, &OdomTf::getInfo, this);
}
void OdomTf::getInfo(const nav_msgs::Odometry::ConstPtr& msg) {
    current_time = ros::Time::now();

    odom_trans.header.stamp            = current_time;
    odom_trans.header.frame_id         = "map";
    odom_trans.child_frame_id          = "base_link";
    odom_trans.transform.translation.x = msg->pose.pose.position.x;
    odom_trans.transform.translation.y = msg->pose.pose.position.y;
    odom_trans.transform.translation.z = msg->pose.pose.position.z;

    odom_trans.transform.rotation.x = msg->pose.pose.orientation.x;
    odom_trans.transform.rotation.y = msg->pose.pose.orientation.y;
    odom_trans.transform.rotation.z = msg->pose.pose.orientation.z;
    odom_trans.transform.rotation.w = msg->pose.pose.orientation.w;

    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    odom.header.stamp    = current_time;
    odom.header.frame_id = "map";
    odom.child_frame_id  = "base_link";

    odom.pose.pose.position.x    = msg->pose.pose.position.x;
    odom.pose.pose.position.y    = msg->pose.pose.position.y;
    odom.pose.pose.position.z    = msg->pose.pose.position.z;
    odom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    odom.pose.pose.orientation.w = msg->pose.pose.orientation.w;

    odom.twist.twist.linear.x  = msg->twist.twist.linear.x;
    odom.twist.twist.linear.y  = msg->twist.twist.linear.y;
    odom.twist.twist.angular.z = msg->twist.twist.linear.z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_localization");
    OdomTf odomtf;
    ros::Rate  loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
