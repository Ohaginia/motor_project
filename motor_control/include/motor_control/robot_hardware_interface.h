/*インクルードガード*/
#ifndef ROS_CONTROL_HARDWARE_H       
#define ROS_CONTROL_HARDWARE_H
/********/

/*hardware interface用の必須ヘッダファイル*/
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

/*通信用*/
#include <rospy_tutorials/Floats.h>
#include <motor_control/Floats_array.h>

#include<angles/angles.h>

class ROBOTHardwareInterface : public hardware_interface::RobotHW
{
    public:
        ROBOTHardwareInterface(ros::NodeHandle& nh_ , int control_mode = 1, int input_mode = 1);           //コンストラクタ
        void init();                            //ロボットの全てのJointHandles、JointInterfaces、JointLimitInterfacesを宣言する
        void update(const ros::TimerEvent& e);  //定周期で関節情報の読み込みと書き込み
        void read();                            //関節情報の読み込み
        void write(ros::Duration elapsed_time); //コントローラからロボットに対し制御を指令

    protected:
        /*ジョイントインターフェースの宣言*/
        hardware_interface::JointStateInterface     joint_state_interface_;
        hardware_interface::EffortJointInterface    effort_joint_interface_;
        hardware_interface::VelocityJointInterface  velocity_joint_interface_;
        hardware_interface::PositionJointInterface  position_joint_interface_;
        
        /*ジョイントインターフェースの可動域制限の宣言*/
		joint_limits_interface::EffortJointSaturationInterface   effort_joint_saturation_interface_;
		joint_limits_interface::EffortJointSoftLimitsInterface   effort_joint_limits_interface_;
		joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
		joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
		joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
		joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;
        

        ros::Publisher               pub;            //jointの制御指令値を出版
        ros::ServiceClient           client;         //関節の角度情報を
        rospy_tutorials::Floats      joints_pub;     
        motor_control::Floats_array  joint_read;

        int num_joints_;                       //ジョイント数
        int joint_control_mode_;                       //位置制御か速度制御か、トルク制御か
        int joint_input_mode_;                       //位置制御か速度制御か、トルク制御か
        std::vector<std::string>                     joint_names_;
		std::vector<int>                             joint_types_;
		std::vector<double>                          joint_position_;
		std::vector<double>                          joint_velocity_;
		std::vector<double>                          joint_effort_;
		std::vector<double>                          joint_position_command_;
		std::vector<double>                          joint_velocity_command_;
		std::vector<double>                          joint_effort_command_;
		std::vector<double>                          joint_lower_limits_;
		std::vector<double>                          joint_upper_limits_;
		std::vector<double>                          joint_effort_limits_;
        
        ros::NodeHandle nh_;            //ノードハンドルの宣言
        ros::Timer my_control_loop_;    //一定間隔で実行するためのオブジェクトの宣言
        ros::Duration elapsed_time_;    //経過時間用のオブジェクト（PID制御に用いる？）
        double loop_hz_;                //コールバックを発生させるための周波数用オブジェクト
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;   //コントローラマネージャーの宣言
};

#endif