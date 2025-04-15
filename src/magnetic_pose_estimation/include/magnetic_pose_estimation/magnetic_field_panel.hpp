#ifndef MAGNETIC_FIELD_PANEL_HPP
#define MAGNETIC_FIELD_PANEL_HPP

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QString>
#include <magnetic_pose_estimation/MagneticField.h>
#include <std_msgs/String.h>

namespace magnetic_pose_estimation
{

    class MagneticFieldPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        MagneticFieldPanel(QWidget *parent = 0);
        virtual ~MagneticFieldPanel();

    protected Q_SLOTS:
        void onCalibrateClicked();
        void onRestoreClicked();

    private:
        ros::NodeHandle nh_;

        QPushButton *calibrate_button_;
        QPushButton *restore_button_;
        ros::ServiceClient calibrate_client_;
        ros::ServiceClient reset_client_;
    };

} // end namespace magnetic_pose_estimation

#endif // MAGNETIC_FIELD_PANEL_HPP