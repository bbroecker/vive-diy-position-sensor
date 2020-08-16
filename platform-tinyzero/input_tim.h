#pragma once
#include "input.h"

struct PinConfig;

// Input node using the tiny zero's inperrupt handler  modules. See description in .cpp file.
class InputTimNode : public InputNode {
public:
    InputTimNode(uint32_t input_idx, const InputDef &def);
    ~InputTimNode();


    void setupClock();
    void connectPortPinsToInterrupt();
    void setupEIC();
    void connectInterruptsToTimer();
    void setupTimer();
    void irqHandler(uint16_t pulse_start, uint16_t pulse_stop);

    virtual void start();
    virtual void do_work(Timestamp time);

private:
    const PinConfig* find_config_for_pin(const uint8_t &pin);
    const PinConfig* pin_config_;
    uint16_t pulse_start_;
    bool rising_received_;
    bool pulse_polarity_;
    static CreatorRegistrar creator_;
    bool found_config;
    int config_idx;
    int ioPin;
    bool nodes_set;
};
