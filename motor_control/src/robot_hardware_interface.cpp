#include <motor_control/robot_hardware_interface.h>

/*コンストラクタの関数定義*/
ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh, int control_mode, int input_mode): nh_(nh), joint_control_mode_(control_mode), joint_input_mode_(input_mode) {         //クラスオブジェクト生成時にノードハンドルを引数として受取、nh_を初期化
    init();                                                                             //初期化関数
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));    //コントローラマネージャーの初期化
    loop_hz_=5;                                                                         //ループ周波数の定義
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);                            //ループ周期の定義
	
	pub = nh_.advertise<rospy_tutorials::Floats>("/ros_control_value",10);              // ros_controlで計算された値をパブリッシュする
	client = nh_.serviceClient<motor_control::Floats_array>("/read_joint_state");       // 実機の情報を配列で受け取るサービス
	
    my_control_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);   //update関数を定周期(update_freq)で呼び出すタイマー
}


void ROBOTHardwareInterface::init() {
    	nh_.getParam("/motor/hardware_interface/joints", joint_names_);
        num_joints_ = joint_names_.size();
		if (num_joints_ == 0)
		{
		  ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
		}
		// Resize vectors
		joint_position_.resize(num_joints_);
		joint_velocity_.resize(num_joints_);
		joint_effort_.resize(num_joints_);
		joint_position_command_.resize(num_joints_);
		joint_velocity_command_.resize(num_joints_);
		joint_effort_command_.resize(num_joints_);

		for (int i = 0; i < num_joints_; ++i){

		    ROS_DEBUG_STREAM_NAMED("constructor","Loading joint name: " << joint_names_[i]);

          		  // Create joint state interface
	        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
		    joint_state_interface_.registerHandle(jointStateHandle);
            registerInterface(&joint_state_interface_);
        
            switch (joint_input_mode_)
            {
                case 1:     //　位置制御
    	    		{   
                        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
                        position_joint_interface_.registerHandle(jointPositionHandle);

                        joint_limits_interface::JointLimits limits;
                        joint_limits_interface::getJointLimits(joint_names_[i], nh_, limits);
	                    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle(jointPositionHandle, limits);
	                    position_joint_saturation_interface_.registerHandle(jointLimitsHandle);
                        registerInterface(&position_joint_interface_);
                        registerInterface(&position_joint_saturation_interface_);
                    }
                        break;
                case 2:     //  速度制御
    	    		{
                        hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
                        velocity_joint_interface_.registerHandle(jointVelocityHandle);

                        joint_limits_interface::JointLimits limits;
                        joint_limits_interface::getJointLimits(joint_names_[i], nh_, limits);
	                    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
	                    velocity_joint_saturation_interface_.registerHandle(jointLimitsHandle);
                        registerInterface(&velocity_joint_interface_);
                        registerInterface(&velocity_joint_saturation_interface_);
                    }
                        break;
                case 3:     //  トルク制御
    	    		{
                        hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
                        effort_joint_interface_.registerHandle(jointEffortHandle);

                        joint_limits_interface::JointLimits limits;
                        joint_limits_interface::getJointLimits(joint_names_[i], nh_, limits);
	                    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle(jointEffortHandle, limits);
	                    effort_joint_saturation_interface_.registerHandle(jointLimitsHandle);
                        registerInterface(&effort_joint_interface_);
                        registerInterface(&effort_joint_saturation_interface_);
                    }
                        break;
            default:        //  位置制御（デフォルト値）
    	    		{
                        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
                        position_joint_interface_.registerHandle(jointPositionHandle);

                        joint_limits_interface::JointLimits limits;
                        joint_limits_interface::getJointLimits(joint_names_[i], nh_, limits);
	                    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle(jointPositionHandle, limits);
	                    position_joint_saturation_interface_.registerHandle(jointLimitsHandle);
                        registerInterface(&position_joint_interface_);
                        registerInterface(&position_joint_saturation_interface_);
                    }
                        break;
            }
		}
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {
	joint_read.request.req=joint_control_mode_;        //サーバからジョイント情報をリクエスト
    switch (joint_control_mode_)
    {
    case 1:
            for(int i=0 ; i < num_joints_; i++){
                if(client.call(joint_read))   
                    joint_position_[i] = joint_read.response.res[i];
                else{
                    joint_position_[i] = 0;
                    ROS_INFO("Service not found ");
                }
            }
        break;
    case 2:
            for(int i=0 ; i < num_joints_; i++){
                if(client.call(joint_read))   
                    joint_velocity_[i] = joint_read.response.res[i];
                else{
                    joint_velocity_[i] = 0;
                    ROS_INFO("Service not found ");
                }
            }
        break;
    case 3:
            for(int i=0 ; i < num_joints_; i++){
                if(client.call(joint_read))   
                    joint_effort_[i] = joint_read.response.res[i];
                else{
                    joint_effort_[i] =0;
                    ROS_INFO("Service not found ");
                }
            }    
    default:
            for(int i=0 ; i < num_joints_; i++){
                if(client.call(joint_read))   
                    joint_position_[i] = joint_read.response.res[i];
                else{
                    joint_position_[i] = 0;
                    ROS_INFO("Service not found ");
                }
            }
        break;
    }    
}


void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
   switch (joint_input_mode_)
   {
        case 1:
            position_joint_saturation_interface_.enforceLimits(elapsed_time);    
	        joints_pub.data.clear();
            for(int i=0; i<num_joints_; i++){
	            joints_pub.data.push_back(joint_position_command_[i]);
            	ROS_INFO("Cmd: %.2f",joint_position_command_[i]);
            }
            break;
        case 2:
            velocity_joint_saturation_interface_.enforceLimits(elapsed_time);    
	        joints_pub.data.clear();
            for(int i=0; i<num_joints_; i++){
	            joints_pub.data.push_back(joint_velocity_command_[i]);
            	ROS_INFO("Cmd: %.2f",joint_velocity_command_[i]);
            }
            break;
        case 3:
            effort_joint_saturation_interface_.enforceLimits(elapsed_time);    
	        joints_pub.data.clear();
            for(int i=0; i<num_joints_; i++){
	            joints_pub.data.push_back(joint_effort_command_[i]);
            	ROS_INFO("Cmd: %.2f",joint_effort_command_[i]);
            }
            break;
        default:
            position_joint_saturation_interface_.enforceLimits(elapsed_time);    
	        joints_pub.data.clear();
            for(int i=0; i<num_joints_; i++){
	            joints_pub.data.push_back(joint_position_command_[i]);
            	ROS_INFO("Cmd: %.2f",joint_position_command_[i]);
            }
            break;
   }
	pub.publish(joints_pub);		
}

//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "robot_hardware_interface");
//    ros::NodeHandle nh;
//    ros::MultiThreadedSpinner spinner(2); 
////    if(argc>1)
////        ROBOTHardwareInterface ROBOT(nh,atoi(argv[1]));
////    else
//        ROBOTHardwareInterface ROBOT(nh);
//    spinner.spin();
//    return 0;
//}