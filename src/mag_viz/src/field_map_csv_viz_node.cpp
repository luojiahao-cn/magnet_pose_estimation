#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <dirent.h>
#include <algorithm>

struct Sample{ double t; double bx,by,bz; double x,y,z; };

struct Color{ double r,g,b,a; };
static Color lerp(const Color&a,const Color&b,double t){return {a.r+(b.r-a.r)*t,a.g+(b.g-a.g)*t,a.b+(b.b-a.b)*t,a.a+(b.a-a.a)*t};}

std::string findLatestScanFolder(const std::string& data_dir) {
    DIR* dir = opendir(data_dir.c_str());
    if (!dir) {
        ROS_WARN("[field_map_csv_viz] cannot open data directory: %s", data_dir.c_str());
        return "";
    }

    std::vector<std::string> scan_folders;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if (entry->d_type == DT_DIR && name.find("scan_") == 0) {
            scan_folders.push_back(name);
        }
    }
    closedir(dir);

    if (scan_folders.empty()) {
        ROS_WARN("[field_map_csv_viz] no scan folders found in: %s", data_dir.c_str());
        return "";
    }

    // 按时间戳排序，找到最新的
    std::sort(scan_folders.begin(), scan_folders.end(), std::greater<std::string>());
    return scan_folders[0];
}

class FieldMapCsvViz{
public:
  FieldMapCsvViz(ros::NodeHandle& nh): nh_(nh){
    if(!nh_.getParam("csv_path", csv_)) {
      ROS_ERROR("[field_map_csv_viz] missing parameter: ~csv_path");
      throw std::runtime_error("缺少参数: ~csv_path");
    }
    if(!nh_.getParam("frame", frame_)) throw std::runtime_error("缺少参数: ~frame");
    if(!nh_.getParam("marker_topic", marker_topic_)) throw std::runtime_error("缺少参数: ~marker_topic");
    if(!nh_.getParam("field_scale", field_scale_)) throw std::runtime_error("缺少参数: ~field_scale");
    if(!nh_.getParam("marker_lifetime", marker_lifetime_)) throw std::runtime_error("缺少参数: ~marker_lifetime");
    if(!nh_.getParam("color_max", color_max_)) throw std::runtime_error("缺少参数: ~color_max");
    nh_.param<int>("stride", stride_, 1);

    std::string subfolder;
    nh_.param<std::string>("subfolder", subfolder, "");

    // 解析相对路径（相对于工作空间根目录）
    if(!csv_.empty() && csv_[0] != '/'){
      try{
        std::string pkg = ros::package::getPath("mag_viz");
        size_t src_pos = pkg.find("/src/");
        if(src_pos != std::string::npos){
          std::string workspace_root = pkg.substr(0, src_pos);
          csv_ = workspace_root + "/" + csv_;
        } else {
          ROS_WARN("[field_map_csv_viz] could not determine workspace root from package path, using given path");
        }
      } catch(const std::exception& e){
        ROS_WARN("[field_map_csv_viz] failed to resolve relative csv path: %s", e.what());
      }
    }

    // 处理子文件夹逻辑
    std::string data_dir = csv_.substr(0, csv_.find_last_of('/')); // 获取data目录路径
    std::string filename = csv_.substr(csv_.find_last_of('/') + 1); // 获取文件名

    if (!subfolder.empty()) {
        // 使用指定的子文件夹
        csv_ = data_dir + "/" + subfolder + "/" + filename;
        ROS_INFO("[field_map_csv_viz] using specified subfolder: %s", subfolder.c_str());
    } else {
        // 自动找到最新的扫描文件夹
        std::string latest_folder = findLatestScanFolder(data_dir);
        if (!latest_folder.empty()) {
            csv_ = data_dir + "/" + latest_folder + "/" + filename;
            ROS_INFO("[field_map_csv_viz] using latest scan folder: %s", latest_folder.c_str());
        } else {
            ROS_WARN("[field_map_csv_viz] no scan folders found, using base path: %s", csv_.c_str());
        }
    }

    pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
    if(!loadCsv()){
      ROS_ERROR("[field_map_csv_viz] aborting due to CSV load failure: %s", csv_.c_str());
      ros::shutdown();
      return;
    }
    publishOnce();
  }
private:
  bool loadCsv(){
    std::ifstream f(csv_);
    if(!f.is_open()){
      ROS_ERROR("[field_map_csv_viz] 无法打开CSV: %s", csv_.c_str());
      return false;
    }
    std::string line; bool first=true; samples_.clear(); samples_.reserve(100000);
    while(std::getline(f,line)){
      if(line.empty()) continue; if(first){first=false; if(line.find("timestamp")!=std::string::npos) continue;} // skip header
      std::stringstream ss(line);
      Sample s; char c;
      if(!(ss>>s.t)) continue; ss>>c; ss>>s.bx; ss>>c; ss>>s.by; ss>>c; ss>>s.bz; ss>>c; ss>>s.x; ss>>c; ss>>s.y; ss>>c; ss>>s.z;
      samples_.push_back(s);
    }
    ROS_INFO("[field_map_csv_viz] loaded %zu samples from %s", samples_.size(), csv_.c_str());
    return true;
  }
  Color colormap(double mag) const{
    double m = std::max(0.0, std::min(mag, color_max_));
    double t = color_max_>1e-9 ? m/color_max_ : 0.0;
    return t<=0.5 ? lerp({0,0,1,1},{0,1,0,1}, t*2.0) : lerp({0,1,0,1},{1,0,0,1}, (t-0.5)*2.0);
  }
  void publishOnce(){
    visualization_msgs::MarkerArray arr; arr.markers.reserve(samples_.size());
    int id=0; for(size_t i=0;i<samples_.size();++i){ if(stride_>1 && (int(i)%stride_!=0)) continue; const auto& s=samples_[i];
      double norm = std::sqrt(s.bx*s.bx + s.by*s.by + s.bz*s.bz); double dx=s.bx,dy=s.by,dz=s.bz; if(norm>1e-9){dx/=norm;dy/=norm;dz/=norm;}
      visualization_msgs::Marker m; m.header.frame_id=frame_; m.header.stamp=ros::Time::now(); m.ns="field_map_csv"; m.id=id++; m.type=visualization_msgs::Marker::ARROW; m.action=visualization_msgs::Marker::ADD;
      m.scale.x=0.0005; m.scale.y=0.002; m.scale.z=0.002; m.lifetime=ros::Duration(marker_lifetime_);
      geometry_msgs::Pose p; p.position.x=s.x; p.position.y=s.y; p.position.z=s.z; p.orientation.w=1.0; m.pose=p; m.points.resize(2); m.points[0].x=0; m.points[0].y=0; m.points[0].z=0; m.points[1].x=dx*field_scale_; m.points[1].y=dy*field_scale_; m.points[1].z=dz*field_scale_;
      auto c = colormap(norm); m.color.r=c.r; m.color.g=c.g; m.color.b=c.b; m.color.a=c.a; arr.markers.push_back(m);
    }
    pub_.publish(arr);
  }
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::string csv_, frame_, marker_topic_;
  double field_scale_{}, marker_lifetime_{}, color_max_{};
  int stride_{1};
  std::vector<Sample> samples_;
};

int main(int argc,char** argv){
  setlocale(LC_ALL, "zh_CN.UTF-8");
  ros::init(argc, argv, "field_map_csv_viz");
  ros::NodeHandle nh("~");
  FieldMapCsvViz node(nh);
  ros::spin();
  return 0;
}
