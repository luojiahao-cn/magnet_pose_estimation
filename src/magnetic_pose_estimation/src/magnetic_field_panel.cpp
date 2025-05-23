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

    // 在按钮下方增加一个垂直布局用于显示数值
    QVBoxLayout *main_layout = new QVBoxLayout;
    main_layout->addLayout(layout);

    QWidget *sensor_widget = new QWidget;
    QVBoxLayout *sensor_layout = new QVBoxLayout(sensor_widget);
    main_layout->addWidget(sensor_widget);
    setLayout(main_layout);

    // 连接按钮信号
    connect(calibrate_button_, SIGNAL(clicked()), this, SLOT(onCalibrateClicked()));
    connect(restore_button_, SIGNAL(clicked()), this, SLOT(onRestoreClicked()));
    connect(reset_localization_button_, SIGNAL(clicked()), this, SLOT(onResetLocalizationClicked()));

    // 创建服务客户端
    calibrate_client_ = nh_.serviceClient<std_srvs::Empty>("/magnetic_field/calibrate_earth_field");
    reset_client_ = nh_.serviceClient<std_srvs::Empty>("/magnetic_field/reset_to_initial");
    reset_localization_client_ = nh_.serviceClient<std_srvs::Empty>("/magnet_pose/reset_localization");

    // 订阅磁场话题
    magnetic_field_sub_ = nh_.subscribe("/magnetic_field", 10, &MagneticFieldPanel::onMagneticFieldMsg, this);
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

void MagneticFieldPanel::onMagneticFieldMsg(const magnetic_pose_estimation::MagneticField::ConstPtr& msg)
{
    int id = msg->sensor_id;
    QString text = QString("传感器%1: [%.3f, %.3f, %.3f]").arg(id).arg(msg->mag_x).arg(msg->mag_y).arg(msg->mag_z);

    if (!sensor_value_labels_.contains(id)) {
        QLabel *label = new QLabel;
        sensor_value_labels_[id] = label;
        // 假设 sensor_layout_ 是你在构造函数中保存的 QVBoxLayout*
        sensor_layout_->addWidget(label);
    }
    sensor_value_labels_[id]->setText(text);
}

} // end namespace magnetic_pose_estimation

// 必须保留此宏以注册RViz插件
PLUGINLIB_EXPORT_CLASS(magnetic_pose_estimation::MagneticFieldPanel, rviz::Panel)