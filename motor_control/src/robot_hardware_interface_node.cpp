#include <motor_control/robot_hardware_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_joint_hardware_interface");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2); 
    ROBOTHardwareInterface ROBOT(nh);
    spinner.spin();
    return 0;
}