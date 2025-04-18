#include "magnetic_pose_estimation/magnetic_field_panel.hpp"

namespace magnetic_pose_estimation
{

MagneticFieldPanel::MagneticFieldPanel(QWidget *parent)
    : rviz::Panel(parent), nh_()
{
    // 创建按钮
    calibrate_button_ = new QPushButton("地磁场校准");
    restore_button_ = new QPushButton("恢复原始数据");
    reset_localization_button_ = new QPushButton("定位重置");

    // 创建水平布局
    QHBoxLayout *layout = new QHBoxLayout;
    layout->addWidget(calibrate_button_);
    layout->addWidget(restore_button_);
    layout->addWidget(reset_localization_button_);
    setLayout(layout);

    // 连接按钮信号
    connect(calibrate_button_, SIGNAL(clicked()), this, SLOT(onCalibrateClicked()));
    connect(restore_button_, SIGNAL(clicked()), this, SLOT(onRestoreClicked()));
    connect(reset_localization_button_, SIGNAL(clicked()), this, SLOT(onResetLocalizationClicked()));

    // 创建服务客户端
    calibrate_client_ = nh_.serviceClient<std_srvs::Empty>("/magnetic_field/calibrate_earth_field");
    reset_client_ = nh_.serviceClient<std_srvs::Empty>("/magnetic_field/reset_to_initial");
    reset_localization_client_ = nh_.serviceClient<std_srvs::Empty>("/magnet_pose/reset_localization");
}

MagneticFieldPanel::~MagneticFieldPanel()
{
}

void MagneticFieldPanel::onCalibrateClicked()
{
    ROS_INFO("开始地磁场校准...");
    std_srvs::Empty srv;
    if (calibrate_client_.call(srv)) {
        ROS_INFO("地磁场校准服务已调用");
    } else {
        ROS_ERROR("地磁场校准服务调用失败！");
    }
}

void MagneticFieldPanel::onRestoreClicked()
{
    ROS_INFO("请求恢复原始数据...");
    std_srvs::Empty srv;
    if (reset_client_.call(srv)) {
        ROS_INFO("已恢复原始数据状态");
    } else {
        ROS_ERROR("恢复原始数据服务调用失败！");
    }
}

void MagneticFieldPanel::onResetLocalizationClicked()
{
    ROS_INFO("请求定位重置...");
    std_srvs::Empty srv;
    if (reset_localization_client_.call(srv))
    {
        ROS_INFO("定位重置服务已调用");
    }
    else
    {
        ROS_ERROR("定位重置服务调用失败！");
    }
}

} // end namespace magnetic_pose_estimation

// 必须保留此宏以注册RViz插件
PLUGINLIB_EXPORT_CLASS(magnetic_pose_estimation::MagneticFieldPanel, rviz::Panel)