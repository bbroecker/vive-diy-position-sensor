// Including Arduino.h is required for using Serial functions
#include <memory>
#include "Arduino.h"
//#include <memory>
#include "input_tim.h"


std::unique_ptr<InputTimNode> input;

// the setup routine runs once when you press reset:
void setup() {

    InputDef input_def {7, false, InputType::kTimer, 0};
    input = std::make_unique<InputTimNode>(0, std::move(input_def));
    input->start();
}

// the loop routine runs over and over again forever:
void loop() {
    input->do_work(Timestamp::cur_time());
}
