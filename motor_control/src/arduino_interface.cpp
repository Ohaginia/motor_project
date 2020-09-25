#include <ros/ros.h>
#include "motor_control/Floats_array.h"
#include <std_msgs/Float32MultiArray.h>
#include <angles/angles.h>

std_msgs::Float32MultiArray pos;
std_msgs::Float32MultiArray vel;
std_msgs::Float32MultiArray eff;

const int motor_num=1;

bool state_server(motor_control::Floats_array::Request  &req, motor_control::Floats_array::Response &res)
{
    res.res.resize(motor_num);
    switch(req.req){
        case 1: {
                    for(int i=0; i< motor_num; i++)
                        res.res[i]=pos.data[i];
                }
                break ;
        case 2: {
                    for(int i=0; i< motor_num; i++)
                        res.res[i]=vel.data[i];
                }
                break ;
        case 3: {
                    for(int i=0; i< motor_num; i++)
                        res.res[i]=eff.data[i];
                }
                break ;
        default : {
                    for(int i=0; i< motor_num; i++)
                        res.res[i]=pos.data[i];
                }
                break ;
    }
  
  return true;
}

void position_cb(const std_msgs::Float32MultiArray& position_msg){
    for(int i=0 ; i< motor_num; i++){
        pos.data[i]=angles::from_degrees(position_msg.data[i]);
    }
}

void velocity_cb(const std_msgs::Float32MultiArray& velocity_msg){
    for(int i=0 ; i< motor_num; i++){
        vel.data[i]=velocity_msg.data[i];
        
    }
}

void effort_cb(const std_msgs::Float32MultiArray& effort_msg){
    for(int i=0 ; i< motor_num; i++){
        eff.data[i]=effort_msg.data[i];
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arduino_state_server");
  ros::NodeHandle nh;
  pos.data.resize(motor_num);
  vel.data.resize(motor_num);
  eff.data.resize(motor_num);

  ros::ServiceServer arduino_state_server = nh.advertiseService("/read_joint_state",state_server);
  ros::Subscriber position_sub = nh.subscribe("/position", 10, position_cb);
  ros::Subscriber velocity_sub = nh.subscribe("/velocity", 10, velocity_cb);
  ros::Subscriber effort_sub = nh.subscribe("/effort", 10, effort_cb);
  ros::spin();
  return 0;
}