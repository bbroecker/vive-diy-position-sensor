#pragma once
#include "messages.h"
//#include "primitives/workers.h"
#include "primitives/producer_consumer.h"
#include "geometry.h"


class MotorController
    : public WorkerNode
    , public Consumer<MotorCmd>{
public:
    MotorController();

    void consume(const MotorCmd&);
    void do_work(Timestamp cur_time);
    void start();
private:
    void updateMotor();
    void updateSteeringAngle();
    float m_motor_speed;
    float m_steer;
    int m_current_steer;
    int m_target_steer;
};

