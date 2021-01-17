// Including Arduino.h is required for using Serial functions
#include <memory>
#include "Arduino.h"
#include "car_control/bluetooth_communication.h"
#include "car_control/motor_controller.h"
//#include <memory>
#include "input_tim.h"
#include "primitives/workers.h"
#include "pulse_processor.h"
#include "geometry.h"
#include "data_frame_decoder.h"


std::unique_ptr<Pipeline> pipeline;


// the setup routine runs once when you press reset:
void setup() {

    pipeline = std::make_unique<Pipeline>();

    // Create central element PulseProcessor.
    auto ble_com = pipeline->add_back(std::make_unique<BluetoothCommunication>(1));
    auto motor_cnt = pipeline->add_back(std::make_unique<MotorController>());
    ble_com->pipe(motor_cnt);
    pipeline->start();


}

// the loop routine runs over and over again forever:
void loop() {
    //SerialUSB.println(";oop");
     pipeline->do_work(Timestamp::cur_time());
}
