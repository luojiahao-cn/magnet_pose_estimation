#include <ros/ros.h>
#include <mag_sensor_node/MagSensorData.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <string>

class RawDataRecorder{
public:
  RawDataRecorder(ros::NodeHandle& nh): nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_) {
    if(!nh_.getParam("topic", topic_)) throw std::runtime_error("缺少参数: ~topic");
    if(!nh_.getParam("frame", frame_)) throw std::runtime_error("缺少参数: ~frame");
    if(!nh_.getParam("output_csv", output_csv_)) throw std::runtime_error("缺少参数: ~output_csv");
    // 覆盖/新建文件并写表头
    {
      std::ofstream f(output_csv_, std::ios::trunc);
      f << "timestamp,mag_x,mag_y,mag_z,pos_x,pos_y,pos_z" << std::endl;
    }
    sub_ = nh_.subscribe(topic_, 200, &RawDataRecorder::onMsg, this);
    ROS_INFO("[raw_data_recorder] writing to %s (frame=%s)", output_csv_.c_str(), frame_.c_str());
  }
private:
  void onMsg(const mag_sensor_node::MagSensorData::ConstPtr& msg){
    geometry_msgs::Pose pose_in = msg->sensor_pose; geometry_msgs::Pose pose_w = pose_in; bool did_tf=false;
    try{
      if(!frame_.empty() && frame_!=msg->header.frame_id){
        geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.05));
        tf2::doTransform(pose_in, pose_w, T);
        did_tf=true;
      }
    }catch(const std::exception& e){
      pose_w = pose_in; ROS_WARN_THROTTLE(5.0, "TF to '%s' failed: %s; using source frame.", frame_.c_str(), e.what());
    }
    double bx=msg->mag_x, by=msg->mag_y, bz=msg->mag_z;
    if(did_tf){
      try{ geometry_msgs::Vector3Stamped vin,vout; vin.header=msg->header; vin.vector.x=bx; vin.vector.y=by; vin.vector.z=bz; 
        geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.05));
        tf2::doTransform(vin, vout, T); bx=vout.vector.x; by=vout.vector.y; bz=vout.vector.z; }catch(...){/*ignore*/}
    }
    std::ofstream f(output_csv_, std::ios::app);
    if(f.is_open()){
      f << msg->header.stamp.toSec() << "," << bx << "," << by << "," << bz << ","
        << pose_w.position.x << "," << pose_w.position.y << "," << pose_w.position.z << "\n";
    }
  }
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string topic_, frame_, output_csv_;
};

int main(int argc,char** argv){
  setlocale(LC_ALL, "zh_CN.UTF-8");
  ros::init(argc, argv, "raw_data_recorder");
  ros::NodeHandle nh("~");
  RawDataRecorder node(nh);
  ros::spin();
  return 0;
}
