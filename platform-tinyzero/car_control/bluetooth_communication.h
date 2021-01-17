#pragma once
#include "messages.h"
//#include "primitives/workers.h"
#include "primitives/producer_consumer.h"
#include "geometry.h"


class BluetoothCommunication
    : public WorkerNode
    , public Consumer<ObjectPosition>
    , public Producer<MotorCmd>  {
public:
    BluetoothCommunication(uint32_t com_id);

    void consume(const ObjectPosition&);
    void do_work(Timestamp cur_time);
    void start();

protected:
    uint32_t com_id_;
};

