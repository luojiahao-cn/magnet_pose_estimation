#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <mag_sensor_node/MagSensorData.h>

#include <vector>
#include <string>
#include <unordered_map>
#include <fstream>
#include <cmath>
#include <cstdint>
#include <sys/stat.h>

struct Vec3 {
  double x{0}, y{0}, z{0};
  Vec3() = default;
  Vec3(double X,double Y,double Z):x(X),y(Y),z(Z){}
  Vec3& operator+=(const Vec3& o){x+=o.x;y+=o.y;z+=o.z;return *this;}
  Vec3 operator/(double d) const {return d>0?Vec3(x/d,y/d,z/d):*this;}
  double norm() const {return std::sqrt(x*x+y*y+z*z);}  
};

struct CellAgg {
  Vec3 b_sum;   // 累积磁场
  Vec3 pos_sum; // 累积位置
  size_t n{0};
};

struct CellKey {
  int ix{0};
  int iy{0};
  int iz{0};
  uint32_t sensor_id{0};
  bool operator==(const CellKey& other) const {
    return ix == other.ix && iy == other.iy && iz == other.iz && sensor_id == other.sensor_id;
  }
};

struct CellKeyHash {
  static inline void hashCombine(std::size_t& seed, std::size_t value) {
    seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  std::size_t operator()(const CellKey& key) const {
    std::size_t seed = std::hash<int>()(key.ix);
    hashCombine(seed, std::hash<int>()(key.iy));
    hashCombine(seed, std::hash<int>()(key.iz));
    hashCombine(seed, std::hash<uint32_t>()(key.sensor_id));
    return seed;
  }
};

class FieldMapAggregator {
public:
  FieldMapAggregator(ros::NodeHandle& nh): nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_) {
    // 读取参数（全部在私有命名空间 ~ 下）
    if (!nh_.getParam("topic", topic_)) throw std::runtime_error("缺少参数: ~topic");
    if (!nh_.getParam("frame", frame_)) throw std::runtime_error("缺少参数: ~frame");
    if (!nh_.getParam("marker_topic", marker_topic_)) throw std::runtime_error("缺少参数: ~marker_topic");
    if (!nh_.getParam("volume_min", volume_min_) || volume_min_.size()!=3) throw std::runtime_error("缺少或非法参数: ~volume_min[3]");
    if (!nh_.getParam("volume_max", volume_max_) || volume_max_.size()!=3) throw std::runtime_error("缺少或非法参数: ~volume_max[3]");
    if (!nh_.getParam("step", step_) || step_.size()!=3) throw std::runtime_error("缺少或非法参数: ~step[3]");
    if (!nh_.getParam("field_scale", field_scale_)) throw std::runtime_error("缺少参数: ~field_scale");
    if (!nh_.getParam("marker_lifetime", marker_lifetime_)) throw std::runtime_error("缺少参数: ~marker_lifetime");
    if (!nh_.getParam("color_max", color_max_)) throw std::runtime_error("缺少参数: ~color_max");
    nh_.param<std::string>("export_csv", export_csv_, std::string(""));
    nh_.param<std::string>("output_base_dir", output_base_dir_, std::string(""));
    
    // 如果设置了export_csv，创建时间戳目录
    if (!export_csv_.empty()) {
        createTimestampDirectory();
    }

    nh_.param<int>("publish_stride", publish_stride_, 1);

    initGrid();

    sub_ = nh_.subscribe(topic_, 200, &FieldMapAggregator::onMag, this);
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 5);
    clear_srv_ = nh_.advertiseService("clear_map", &FieldMapAggregator::clearMap, this);
    at_position_sub_ = nh_.subscribe("/scan_at_position", 1, &FieldMapAggregator::atPositionCallback, this);

    pub_timer_ = nh_.createTimer(ros::Duration(0.5), &FieldMapAggregator::publishMarkersTimer, this);

    ROS_INFO_STREAM("[field_map_aggregator] subscribe='"<<topic_<<"' frame='"<<frame_
      <<"' marker='"<<marker_topic_<<"' grid="<<nx_<<"x"<<ny_<<"x"<<nz_);
  }

private:
  // 将连续坐标落至网格索引
  bool worldToIdx(const geometry_msgs::Point& p, int& ix,int& iy,int& iz) const {
    ix = int(std::floor((p.x - volume_min_[0]) / step_[0] + 1e-9));
    iy = int(std::floor((p.y - volume_min_[1]) / step_[1] + 1e-9));
    iz = int(std::floor((p.z - volume_min_[2]) / step_[2] + 1e-9));
    return ix>=0 && iy>=0 && iz>=0 && ix<nx_ && iy<ny_ && iz<nz_;
  }
  void initGrid(){
    nx_ = std::max(1, int(std::floor((volume_max_[0]-volume_min_[0]) / step_[0] + 0.5)) + 1);
    ny_ = std::max(1, int(std::floor((volume_max_[1]-volume_min_[1]) / step_[1] + 0.5)) + 1);
    nz_ = std::max(1, int(std::floor((volume_max_[2]-volume_min_[2]) / step_[2] + 0.5)) + 1);
    cell_map_.clear();
    cell_map_.reserve(static_cast<std::size_t>(nx_) * ny_ * nz_);
  }

  struct Color{double r,g,b,a;};
  static Color lerp(const Color&a,const Color&b,double t){return {a.r+(b.r-a.r)*t,a.g+(b.g-a.g)*t,a.b+(b.b-a.b)*t,a.a+(b.a-a.a)*t};}
  Color colormap(double mag) const{
    double m = std::max(0.0, std::min(mag, color_max_));
    double t = color_max_>1e-9 ? m/color_max_ : 0.0;
    return t<=0.5 ? lerp({0,0,1,1},{0,1,0,1}, t*2.0) : lerp({0,1,0,1},{1,0,0,1}, (t-0.5)*2.0);
  }

  bool clearMap(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    cell_map_.clear();
    ROS_INFO("[field_map_aggregator] cleared accumulated data");
    res.success = true;
    res.message = "Map cleared";
    return true;
  }

  void atPositionCallback(const std_msgs::Bool::ConstPtr& msg) {
    at_position_ = msg->data;
  }

  void onMag(const mag_sensor_node::MagSensorData::ConstPtr& msg){
    if (!at_position_) return;  // Only accumulate when at scan position

    // 将传感器姿态与向量转换到 target frame
    geometry_msgs::Pose pose_in = msg->sensor_pose;
    geometry_msgs::Pose pose_w = pose_in; // in frame_
    bool did_tf=false;
    try{
      if (!frame_.empty() && frame_ != msg->header.frame_id){
        geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
        tf2::doTransform(pose_in, pose_w, T);
        geometry_msgs::Vector3Stamped vin,vout; vin.header=msg->header; vin.vector.x=msg->mag_x; vin.vector.y=msg->mag_y; vin.vector.z=msg->mag_z; 
        tf2::doTransform(vin, vout, T);
        bx_=vout.vector.x; by_=vout.vector.y; bz_=vout.vector.z;
        did_tf=true;
      }
    }catch(const std::exception& e){
      pose_w = pose_in; bx_=msg->mag_x; by_=msg->mag_y; bz_=msg->mag_z;
      ROS_WARN_THROTTLE(5.0,"TF to '%s' failed: %s; using source frame.", frame_.c_str(), e.what());
    }
    if(!did_tf){ bx_=msg->mag_x; by_=msg->mag_y; bz_=msg->mag_z; }

  int ix,iy,iz; if(!worldToIdx(pose_w.position, ix,iy,iz)) return; // out of volume
  CellKey key{ix, iy, iz, msg->sensor_id};
  auto& cell = cell_map_[key];
  cell.b_sum += Vec3(bx_,by_,bz_);
  cell.pos_sum += Vec3(pose_w.position.x, pose_w.position.y, pose_w.position.z);
  cell.n += 1;

    if (!export_csv_.empty()) {
      // 追加原始样本：timestamp,Bx,By,Bz,x,y,z
      std::ofstream f(export_csv_, std::ios::app);
      if (f.is_open()){
        f << msg->header.stamp.toSec() << "," << bx_ << "," << by_ << "," << bz_ << ","
          << pose_w.position.x << "," << pose_w.position.y << "," << pose_w.position.z << "\n";
      }
    }
  }

  void publishMarkersTimer(const ros::TimerEvent&){
    visualization_msgs::MarkerArray arr; arr.markers.reserve(cell_map_.size());
    int id=0;
    for (const auto& kv : cell_map_){
      const CellKey& key = kv.first;
      const CellAgg& cell = kv.second;
      if (cell.n == 0) continue;
      if (publish_stride_>1 && ((key.ix + key.iy + key.iz) % publish_stride_ != 0)) continue;

      Vec3 b = cell.b_sum / double(cell.n);
      Vec3 pos = cell.pos_sum / double(cell.n);
      double norm = b.norm();
      double dx=b.x,dy=b.y,dz=b.z; if(norm>1e-9){dx/=norm;dy/=norm;dz/=norm;}

      visualization_msgs::Marker m; m.header.frame_id=frame_; m.header.stamp=ros::Time::now();
      m.ns="sensor_field"; m.id=id++; m.type=visualization_msgs::Marker::ARROW; m.action=visualization_msgs::Marker::ADD;
      m.scale.x = 0.0005; m.scale.y = 0.002; m.scale.z = 0.002; m.lifetime = ros::Duration(marker_lifetime_);
      geometry_msgs::Pose p; p.position.x = pos.x; p.position.y = pos.y; p.position.z = pos.z; p.orientation.w = 1.0; m.pose = p;
      m.points.resize(2); m.points[0].x=0; m.points[0].y=0; m.points[0].z=0; m.points[1].x=dx*field_scale_; m.points[1].y=dy*field_scale_; m.points[1].z=dz*field_scale_;
      auto c = colormap(norm); m.color.r=c.r; m.color.g=c.g; m.color.b=c.b; m.color.a=c.a;
      arr.markers.push_back(m);
    }
    pub_.publish(arr);
    if (!arr.markers.empty()) {
      ROS_INFO("[field_map_aggregator] published %zu markers", arr.markers.size());
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::ServiceServer clear_srv_;
  ros::Subscriber at_position_sub_;
  ros::Timer pub_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void createTimestampDirectory()
  {
    // 获取当前时间
    std::time_t now = std::time(nullptr);
    std::tm* tm = std::localtime(&now);
    
    // 格式化为 YYYYMMDD_HHMM
    char timestamp[20];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M", tm);
    
    // 使用配置的基目录创建时间戳文件夹
    std::string timestamp_dir = output_base_dir_ + "/scan_" + timestamp;
    
    // 创建目录
    if (mkdir(timestamp_dir.c_str(), 0755) == 0) {
      ROS_INFO("[field_map_aggregator] created timestamp directory: %s", timestamp_dir.c_str());
    } else {
      ROS_WARN("[field_map_aggregator] failed to create directory: %s", timestamp_dir.c_str());
    }
    
    // 设置输出文件路径为文件夹中的online_samples.csv
    export_csv_ = timestamp_dir + "/online_samples.csv";
    ROS_INFO("[field_map_aggregator] export file: %s", export_csv_.c_str());
  }

  std::string topic_;
  std::string frame_;
  std::string marker_topic_;
  std::vector<double> volume_min_, volume_max_, step_;
  double field_scale_{}; double marker_lifetime_{}; double color_max_{};
  std::string export_csv_;
  std::string output_base_dir_;
  int publish_stride_{1};

  int nx_{0}, ny_{0}, nz_{0};
  std::unordered_map<CellKey, CellAgg, CellKeyHash> cell_map_;

  bool at_position_{false};
  double bx_{0}, by_{0}, bz_{0};
};

int main(int argc,char** argv){
  setlocale(LC_ALL, "zh_CN.UTF-8");
  ros::init(argc, argv, "field_map_aggregator");
  ros::NodeHandle nh("~");
  FieldMapAggregator node(nh);
  ros::spin();
  return 0;
}
