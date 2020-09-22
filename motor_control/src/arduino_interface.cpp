#include <ros/ros.h>
#include "motor_control/Floats_array.h"
#include <std_msgs/Float32MultiArray.h>

std_msgs::Float32MultiArray pos;
std_msgs::Float32MultiArray vel;
std_msgs::Float32MultiArray eff;

bool state_server(motor_control::Floats_array::Request  &req, motor_control::Floats_array::Response &res)
{
    swich(req){
        case 1: {
                    int motor_num= sizeof(pos.data)/sizeof(pos.data[0]);
                    for(int i=0; i< motor_num; i++)
                        res.res[i]=pos.data[i];
                }
                break ;
        case 2: {
                    int motor_num= sizeof(vel.data)/sizeof(vel.data[0]);
                    for(int i=0; i< motor_num; i++)
                        res.res[i]=vel.data[i];
                }
                break ;
        case 3: {
                    int motor_num= sizeof(eff.data)/sizeof(eff.data[0]);
                    for(int i=0; i< motor_num; i++)
                        res.res[i]=eff.data[i];
                }
                break ;
        default : {
                    int motor_num= sizeof(pos.data)/sizeof(pos.data[0]);
                    for(int i=0; i< motor_num; i++)
                        res.res[i]=pos.data[i];
                }
                break ;
    }
  return true;
}

void position_cb(const std_msgs::Float32MultiArray& position_msg){
    int motor_num=sizeof(position_msg)/sizeof(position_msg[0]);
    pos.data.resize(motor_num);
    for(int i=0 ; i< motor_num; i++){
        pos.data[i]=position_msg[i];
    }
}

void velocity_cb(const std_msgs::Float32MultiArray& velocity_msg){
    int motor_num=sizeof(velocity_msg)/sizeof(velocity_msg[0]);
    vel.data.resize(motor_num);
    for(int i=0 ; i< motor_num; i++){
        vel.data[i]=velocity_msg[i];
        
    }
}

void effort_cb(const std_msgs::Float32MultiArray& effort_msg){
    int motor_num=sizeof(effort_msg)/sizeof(effort_msg[0]);
    eff.data.resize(motor_num);
    for(int i=0 ; i< motor_num; i++){
        eff.data[i]=effort_msg[i];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arduino_state_server");
  ros::NodeHandle nh;

  ros::ServiceServer state_service = nh.advertiseService("/read_joint_state",state_server);
  ros::Subscriber position_sub = n.subscribe("/position", 10, position_cb);
  ros::Subscriber velocity_sub = n.subscribe("/velocity", 10, velocity_cb);
  ros::Subscriber effort_sub = n.subscribe("/effort", 10, effort_cb);
  ros::spin();

  return 0;
}