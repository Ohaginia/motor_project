#include <ODriveTool.h>

/* ROS */
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <ros/time.h>
/******/

// ODrive object
ODriveTool odrive(Serial2);

void ros_control_cb(const std_msgs::Float32MultiArray& roscon_value);

/* ROS */
ros::NodeHandle  nh;

/* ロスコントロールから制御 */
ros::Subscriber<std_msgs::Float32MultiArray> ros_control("/ros_control_value", ros_control_cb);

/*　位置情報配信トピック　*/
std_msgs::Int32MultiArray position_data;
ros::Publisher position_pub("/position", &position_data);

/*　速度情報配信トピック　*/
std_msgs::Float32MultiArray velocity_data;
ros::Publisher velocity_pub("/velocity", &velocity_data);

/*　トルク情報配信トピック　*/
std_msgs::Float32MultiArray effort_data;
ros::Publisher effort_pub("/effort", &effort_data);

/*　電圧情報配信トピック　*/
std_msgs::Float32 voltage_data;
ros::Publisher voltage_pub("/voltage", &voltage_data);
/******/

const int num_motor=1;
const float pi=3.14159262;

void array_init(std_msgs::Int32MultiArray& data, int array){
    data.data_length=array;
    data.data=(int32_t *)malloc(sizeof(int32_t)*array);
    for(int i=0; i<array ;i++){
        data.data[i]=0;
    }
}

void array_init(std_msgs::Float32MultiArray& data, int array){
    data.data_length=array;
    data.data=(float *)malloc(sizeof(float)*array);
    for(int i=0; i<array ;i++){
        data.data[i]=0;
    }
}
void ros_init()
{
    array_init(position_data,2);
    array_init(velocity_data,2);
    array_init(effort_data,2);
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(ros_control);
    nh.advertise(position_pub);
    nh.advertise(velocity_pub);
    nh.advertise(voltage_pub);
}


void setup() {
  odrive.odrive_reboot();
  ros_init();
  for(int motor=0; motor< num_motor; motor++){
     odrive.odrive_init(motor);
  }
}

void loop() {
  voltage_data.data=odrive.get_voltage();
  voltage_pub.publish(&voltage_data);

  for(int motor=0; motor< num_motor ;motor++){
    position_data.data[motor]=odrive.get_position(motor);
    velocity_data.data[motor]=odrive.get_velocity(motor);
    effort_data.data[motor]=odrive.get_effort(motor);    
  }
  position_pub.publish(&position_data);
  velocity_pub.publish(&velocity_data);
  effort_pub.publish(&effort_data);

  nh.spinOnce();
  delay(500);
}

void ros_control_cb(const std_msgs::Float32MultiArray& roscon_value){
  float pos[2]={};
  for(int motor=0; motor< num_motor; motor++){
        pos[motor]=(roscon_value.data[motor]*(2000/pi));
        odrive.SetPosition(motor,pos[motor]);
  }
}
