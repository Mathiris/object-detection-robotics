#include "mr_teleop/control.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mr_teleop_cpp");
    ros::NodeHandle nh;

    control c(nh, 0.1, 0.46);

    ros::spin();

    return 0;
}
