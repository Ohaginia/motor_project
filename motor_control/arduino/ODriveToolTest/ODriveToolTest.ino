
#include <ODriveTool.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// ODrive object
ODriveTool odrive(Serial2);

void setup() {
  Serial.begin(115200);
  odrive.odrive_reboot();
  for(int i=1; i<2 ;i++){
//     odrive.odrive_init(i,5000);
      int requested_state;

      requested_state = ODriveTool::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << i << ": Requesting state " << requested_state << '\n';
      odrive.run_state(i, requested_state, true);

      requested_state = ODriveTool::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << i << ": Requesting state " << requested_state << '\n';
      odrive.run_state(i, requested_state, true);
     delay(1000);
  }
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();
        if (c == 'p') {
      Serial.println("Executing test move");
      while (true) {
        Serial << "Pos1:" << odrive.get_position(1) << '\n';
        delay(5);
      }
    }
    // Sinusoidal test move
//    if (c == 'p') {
//      Serial.println("Executing test move");
//      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
//        float pos_m0 = 20000 * cos(ph);
//        float pos_m1 = 20000 * sin(ph);
//        odrive.SetPosition(0, pos_m0);
//        odrive.SetPosition(1, pos_m1);
//        Serial << "Pos0:" << odrive.get_position(0) << '\t';
//        Serial << "Pos1:" << odrive.get_position(1) << '\n';
//        delay(5);
//      }
//    }
//
//    if (c == 'v') {
//      Serial.println("Executing test move");
//      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
//        float pos_m0 = 30000 * cos(ph);
//        float pos_m1 = 30000 * sin(ph);
//        odrive.SetPosition(0, pos_m0);
//        odrive.SetPosition(1, pos_m1);
//        Serial << "vel0:" << odrive.get_velocity(0) << '\t';
//        Serial << "vel1:" << odrive.get_velocity(1) << '\n';
//        delay(5);
//      }
//    }
  }
}
