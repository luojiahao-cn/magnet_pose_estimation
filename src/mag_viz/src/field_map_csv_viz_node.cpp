#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

struct Sample{ double t; double bx,by,bz; double x,y,z; };

struct Color{ double r,g,b,a; };
static Color lerp(const Color&a,const Color&b,double t){return {a.r+(b.r-a.r)*t,a.g+(b.g-a.g)*t,a.b+(b.b-a.b)*t,a.a+(b.a-a.a)*t};}

class FieldMapCsvViz{
public:
  FieldMapCsvViz(ros::NodeHandle& nh): nh_(nh){
    if(!nh_.getParam("csv_path", csv_)) throw std::runtime_error("缺少参数: ~csv_path");
    if(!nh_.getParam("frame", frame_)) throw std::runtime_error("缺少参数: ~frame");
    if(!nh_.getParam("marker_topic", marker_topic_)) throw std::runtime_error("缺少参数: ~marker_topic");
    if(!nh_.getParam("field_scale", field_scale_)) throw std::runtime_error("缺少参数: ~field_scale");
    if(!nh_.getParam("marker_lifetime", marker_lifetime_)) throw std::runtime_error("缺少参数: ~marker_lifetime");
    if(!nh_.getParam("color_max", color_max_)) throw std::runtime_error("缺少参数: ~color_max");
    nh_.param<int>("stride", stride_, 1);

    pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
    loadCsv();
    publishOnce();
  }
private:
  void loadCsv(){
    std::ifstream f(csv_);
    if(!f.is_open()) throw std::runtime_error(std::string("无法打开CSV: ")+csv_);
    std::string line; bool first=true; samples_.clear(); samples_.reserve(100000);
    while(std::getline(f,line)){
      if(line.empty()) continue; if(first){first=false; if(line.find("timestamp")!=std::string::npos) continue;} // skip header
      std::stringstream ss(line);
      Sample s; char c;
      if(!(ss>>s.t)) continue; ss>>c; ss>>s.bx; ss>>c; ss>>s.by; ss>>c; ss>>s.bz; ss>>c; ss>>s.x; ss>>c; ss>>s.y; ss>>c; ss>>s.z;
      samples_.push_back(s);
    }
    ROS_INFO("[field_map_csv_viz] loaded %zu samples from %s", samples_.size(), csv_.c_str());
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
