#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>

std_msgs::Float32MultiArray joy_array;
const int array_size=8;

void joy_callback(const sensor_msgs::Joy& joy_msg)
{
    for(int i=0; i < array_size ; i++){
        joy_array.data[i] = joy_msg.axes[i];
    }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_array_pub");
  ros::NodeHandle nh;
  ros::Publisher joy_array_pub = nh.advertise<std_msgs::Float32MultiArray>("joy_array", 1000);
  ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);

  joy_array.data.resize(8);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    
    joy_array_pub.publish(joy_array);
    ros::spinOnce();
    loop_rate.sleep();
  }
}