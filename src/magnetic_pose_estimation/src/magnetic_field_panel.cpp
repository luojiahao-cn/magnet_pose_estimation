#include "magnetic_pose_estimation/magnetic_field_panel.hpp"
#include <QLabel>
#include <QVector>

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
    QGridLayout *sensor_layout = new QGridLayout(sensor_widget);

    // 以5行，每行5个传感器，每个传感器3个数值框，前面加标号
    sensor_value_labels_.resize(25 * 3);
    for (int i = 0; i < 25; ++i) {
        int row = i / 5;
        int col_base = (i % 5) * 4; // 每组4列：标号+X+Y+Z

        // 标号
        QLabel* label_sensor = new QLabel(QString("%1").arg(i + 1));
        label_sensor->setAlignment(Qt::AlignCenter);
        // 去掉边框和背景色
        label_sensor->setStyleSheet(""); // 或直接不设置样式
        label_sensor->setMinimumWidth(70);
        label_sensor->setMinimumHeight(30);
        sensor_layout->addWidget(label_sensor, row, col_base);

        // X/Y/Z数值框
        for (int j = 0; j < 3; ++j) {
            int idx = i * 3 + j;
            QString axis = (j == 0) ? "X" : (j == 1) ? "Y" : "Z";
            sensor_value_labels_[idx] = new QLabel(QString("%1: --").arg(axis));
            // 设置右对齐
            sensor_value_labels_[idx]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            sensor_value_labels_[idx]->setStyleSheet("border: 1px solid gray; border-radius: 4px; background: #f9f9f9;");
            sensor_value_labels_[idx]->setMinimumWidth(60);
            sensor_value_labels_[idx]->setMinimumHeight(30);
            sensor_layout->addWidget(sensor_value_labels_[idx], row, col_base + j + 1);
        }
    }

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
    magnetic_field_sub_ = nh_.subscribe("/magnetic_field/raw_data", 30, &MagneticFieldPanel::onMagneticFieldMsg, this);
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
    int idx = static_cast<int>(msg->sensor_id) - 1;
    if (idx >= 0 && idx < 25)
    {
        sensor_value_labels_[idx * 3 + 0]->setText(QString("%1").arg(static_cast<int>(msg->mag_x)));
        sensor_value_labels_[idx * 3 + 1]->setText(QString("%1").arg(static_cast<int>(msg->mag_y)));
        sensor_value_labels_[idx * 3 + 2]->setText(QString("%1").arg(static_cast<int>(msg->mag_z)));
    }
}

} // end namespace magnetic_pose_estimation

// 必须保留此宏以注册RViz插件
PLUGINLIB_EXPORT_CLASS(magnetic_pose_estimation::MagneticFieldPanel, rviz::Panel)