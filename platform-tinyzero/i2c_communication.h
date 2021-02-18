#pragma once
#include "input.h"

#include "pulse_processor.h"
#include "geometry.h"
#include "data_frame_decoder.h"

class I2CPoseSender : public Consumer<ObjectPosition>, public WorkerNode {

    public:
        I2CPoseSender(int i2c_address);
        void consume(const ObjectPosition& f);
        static void onRequest();
        void sendPose();
        virtual void start();
        virtual void do_work(Timestamp time);

    private:
        static I2CPoseSender* senderSingleton;
        int i2c_address_;
};

