#include <ros/ros.h>
#include <mag_tracking_control/tracking_controller.hpp>

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "tracking_controller");
    ros::NodeHandle nh("~");

    mag_tracking_control::TrackingController controller(nh);
    controller.run();

    return 0;
}