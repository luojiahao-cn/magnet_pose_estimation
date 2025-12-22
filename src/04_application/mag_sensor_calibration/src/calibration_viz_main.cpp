#include <mag_sensor_calibration/calibration_viz_node.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration_viz_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    try
    {
        mag_sensor_calibration::CalibrationVizNode node(nh, pnh);
        node.start();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("[calibration_viz] 节点异常: %s", e.what());
        return 1;
    }
    
    return 0;
}

