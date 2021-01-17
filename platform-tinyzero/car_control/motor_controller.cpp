#include "motor_controller.h"
#include "Arduino.h"

MotorController::MotorController()
{
}



void MotorController::consume(const MotorCmd& motor_cmd) {

    SerialUSB.print("steer angle: ");
    SerialUSB.print(motor_cmd.steer);
    SerialUSB.print("\n");
}



void MotorController::start() {
  //BLEsetup();
}


void MotorController::do_work(Timestamp cur_time) {
}dasd
