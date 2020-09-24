#include <ODriveTool.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// ODrive object
ODriveTool odrive(Serial2);

void setup() {
  //odrive.odrive_reboot();
  odrive.odrive_init(0);
  //odrive.odrive_init(1);  
}

void loop() {
//  voltage_data.data=odrive.get_voltage();
//  voltage_pub.publish(&voltage_data);
//
//  for(int motor=0; motor<2 ;motor++){
//    position_data.data[motor]=odrive.get_position(motor);
//    velocity_data.data[motor]=odrive.get_velocity(motor);
//  }
//  position_pub.publish(&position_data);
//  velocity_pub.publish(&velocity_data);
//  
//  nh.spinOnce();
//  delay(500);
}

//void odrive_cb(const std_msgs::Float32MultiArray& joy_msg){
//    joy_msg.data[1] > 0 ? vel1=pow((joy_msg.data[1]*3.0),2)*(40000.0/9.0) : vel1=pow((joy_msg.data[1]*3.0),2)*(-40000.0/9.0);
//    joy_msg.data[4] > 0 ? vel2=pow((joy_msg.data[4]*3.0),2)*(40000.0/9.0) : vel2=pow((joy_msg.data[4]*3.0),2)*(-40000.0/9.0);
//
//    odrive.SetVelocity(0,vel1);
//    odrive.SetVelocity(1,vel2);
//}
