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

};

