#include "motor_controller.h"
#include "Arduino.h"

#define STEER_MARGIN 20

int steer_dir_1 = A3;
int steer_dir_2 = 8;
int steer_pw = 5;

int motor_dir_1 = A1;
int motor_dir_2 = 6;
int motor_pw = 4;

int steer_pot_pin = A2;

int center = 540;
int left = 340;
int right = 740;
//Pins IO3, IO5, IO6, IO9, SS (IO10), MOSI (IO11): These pins can provide an 8-bit PWM. See the Arduino function analogWrite() for details./
//Pins IO2 and IO3: These pins can be used as External Interrupts 2 and 3. See the Arduino function attachInterrupt() for details.
int left_range = center - left;
int right_range = right - center;

MotorController::MotorController()
{
    m_steer = 0.;
    m_motor_speed = 0;
    m_current_steer = center;
    m_target_steer = center;
}



void MotorController::consume(const MotorCmd& motor_cmd) {

    SerialUSB.print("steer angle: ");
    SerialUSB.print(motor_cmd.steer);
    SerialUSB.print("\n");

    m_motor_speed = motor_cmd.speed;

    int range = (motor_cmd.steer < 0)? left_range : right_range;
    m_target_steer = int(center + range * motor_cmd.steer);

}



void MotorController::start() {
  //BLEsetup();
    pinMode(steer_dir_1,   OUTPUT);
    pinMode(steer_dir_2,   OUTPUT);
    pinMode(steer_pw,   OUTPUT);

    pinMode(motor_dir_1,   OUTPUT);
    pinMode(motor_dir_2,   OUTPUT);
    pinMode(motor_pw,   OUTPUT);
    digitalWrite(steer_dir_1, LOW);
    digitalWrite(steer_dir_2, LOW);
    digitalWrite(motor_dir_1, LOW);
    digitalWrite(motor_dir_2, LOW);
}

void MotorController::updateSteeringAngle(){

    if (m_target_steer - STEER_MARGIN <= m_current_steer && m_current_steer <= m_target_steer + STEER_MARGIN)
    {
        digitalWrite(steer_dir_1, LOW);
        digitalWrite(steer_dir_2, LOW);
    }
    else if(m_current_steer < m_target_steer){
        digitalWrite(steer_dir_1, HIGH);
        digitalWrite(steer_dir_2, LOW);
    }else{
        digitalWrite(steer_dir_1, LOW);
        digitalWrite(steer_dir_2, HIGH);
    }
    analogWrite(steer_pw, (int)( 0.7 * 255));
}


void MotorController::updateMotor(){

    if (m_motor_speed > 0){
        digitalWrite(motor_dir_1, HIGH);
        digitalWrite(motor_dir_2, LOW);
    }
    else if (m_motor_speed < 0){
        digitalWrite(motor_dir_1, LOW);
        digitalWrite(motor_dir_2, HIGH);

    }else{
        digitalWrite(motor_dir_1, LOW);
        digitalWrite(motor_dir_2, LOW);
    }

    analogWrite(motor_pw, (int)(abs(m_motor_speed) * 255));
}

void MotorController::do_work(Timestamp cur_time) {
    m_current_steer = analogRead(steer_pot_pin);
    SerialUSB.print("current: ");
    SerialUSB.print(m_current_steer);
    SerialUSB.print(" target: ");
    SerialUSB.print(m_target_steer);
    SerialUSB.print("\n");
    updateMotor();
    updateSteeringAngle();
}
