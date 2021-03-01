#include "primitives/timestamp.h"
#include "input_tim.h"
#include "samd21_timer.h"
#include <SPI.h>


struct PinConfig
{
    uint16_t ioPin;
    uint16_t PORT_PA;
//    Tc *tc;
    uint16_t clkctrl_tc;
    uint16_t ext_int;
    uint16_t channel;
    uint32_t tc_enable_bit;
    IRQn_Type tc_irqn;
    uint16_t node_idx;
    uint16_t timer_idx;
    uint16_t clock_idx;
    uint16_t rising_event_out;
    uint16_t falling_event_out;
};


Tcc* tccTimer[] = {
    TCC0, TCC1, TCC2
};


const static PinConfig available_configs[] = {

    // clock idx tc --> GLCK0, tcc --> GLCK3
    {4, PORT_PA04,  GCLK_CLKCTRL_ID_TCC0_TCC1, EVSYS_ID_GEN_EIC_EXTINT_4,  1, PM_APBCMASK_TCC0, TCC0_IRQn,  0, 0, 3,  EVSYS_ID_USER_TCC0_MC_0, EVSYS_ID_USER_TCC0_MC_1},
    {10, PORT_PA10, GCLK_CLKCTRL_ID_TCC0_TCC1, EVSYS_ID_GEN_EIC_EXTINT_10,  4, PM_APBCMASK_TCC0, TCC0_IRQn, 1, 0, 3, EVSYS_ID_USER_TCC0_MC_2, EVSYS_ID_USER_TCC0_MC_3},
    {11, PORT_PA11, GCLK_CLKCTRL_ID_TCC0_TCC1, EVSYS_ID_GEN_EIC_EXTINT_11,  7, PM_APBCMASK_TCC1, TCC1_IRQn, 2, 1, 3, EVSYS_ID_USER_TCC1_MC_0, EVSYS_ID_USER_TCC1_MC_1},
    {2, PORT_PA02, GCLK_CLKCTRL_ID_TCC2_TC3, EVSYS_ID_GEN_EIC_EXTINT_2,  9, PM_APBCMASK_TCC2, TCC2_IRQn, 3, 2, 3, EVSYS_ID_USER_TCC2_MC_0, EVSYS_ID_USER_TCC2_MC_1},
};

static InputTimNode* connecteNodes_[6];

static bool timer_initialized = false;

InputTimNode::InputTimNode(uint32_t input_idx, const InputDef &input_def)
    : InputNode(input_idx)
    , pulse_polarity_(input_def.pulse_polarity), rising_received_(false) {


    // Find config
    pin_config_ = find_config_for_pin(input_def.pin);
    if (!pin_config_){
    //    throw_printf("Pin %d has no available config.", input_def.pin);
    }else
        found_config=true;

    if (connecteNodes_[pin_config_->node_idx]){
        //throw_printf("TCC %d is already takenk", input_idx);
    }
    else{
        nodes_set=true;
        ioPin = pin_config_->ioPin;
    }
    connecteNodes_[pin_config_->node_idx] = this;

    config_idx = pin_config_->timer_idx;

}


InputTimNode::~InputTimNode() {
//    input_cmps[cmp_idx_] = nullptr;
}

void InputTimNode::start()
{
    setupClock(pin_config_->clock_idx);
    connectPortPinsToInterrupt(pin_config_->PORT_PA, pin_config_->ioPin);
    setupEIC(pin_config_->ioPin);
    uint16_t prescalar = (F_CPU / sec);

    Tcc *tmp_tcc = tccTimer[pin_config_->timer_idx];
    setupTimer(tmp_tcc, pin_config_->clkctrl_tc, pin_config_->ext_int, pin_config_->channel, pin_config_->tc_enable_bit, pin_config_->tc_irqn, !pulse_polarity_, prescalar, pin_config_->rising_event_out, pin_config_->falling_event_out);
   /* bool invert = true;
    setupClock(0);
    uint16_t prescalar = 4;
    connectPortPinsToInterrupt(PORT_PA02, 2);
    setupEIC(2);
    setupTimer(TC3, GCLK_CLKCTRL_ID_TCC2_TC3, EVSYS_ID_GEN_EIC_EXTINT_2, EVSYS_ID_USER_TC3_EVU, 1, PM_APBCMASK_TC3, TC3_IRQn, invert, prescalar);*/

}


InputNode::CreatorRegistrar InputTimNode::creator_([](uint32_t input_idx, const InputDef &input_def) -> std::unique_ptr<InputNode> {
    if (input_def.input_type == InputType::kTimer)
        return std::make_unique<InputTimNode>(input_idx, input_def);
    return nullptr;
});


const PinConfig* InputTimNode::find_config_for_pin(const uint8_t &pin)
{
    const PinConfig *result = nullptr;
    for (auto it = std::begin(available_configs); it != std::end(available_configs); ++it)
    {
        if (it->ioPin == pin)
        {
            result = &*it;
            break;
        }
    }
    return result;
}


void InputTimNode::do_work(Timestamp cur_time) {
    InputNode::do_work(cur_time);

    //SerialUSB.println((F_CPU / sec) - 1);
  /*  SerialUSB.print("nodes_set: " );
    SerialUSB.print(nodes_set);
    SerialUSB.print(" found_config: ");
    SerialUSB.print(found_config);
    SerialUSB.print(" config_idx ");
    SerialUSB.print(config_idx);
    SerialUSB.print(" ioPin ");
    SerialUSB.print(ioPin);
    SerialUSB.print("\n");
*/
   //:w + (int)nodes_set + " found_config " + (int)found_config + " config_idx " + config_idx);
}


void InputTimNode::irqHandler(uint16_t pulse_start, uint16_t pulse_stop)
{
    //SerialUSB.println("\nirqHandler");
    {
        // We assume that the pulse length is within one timer period.
        Timestamp cur_time = Timestamp::cur_time();
        uint16_t pulse_len = pulse_stop - pulse_start;
        TimeDelta pulse_len_ts(pulse_len, (TimeUnit)1);

        // We also assume that the time between end of pulse and interrupt is also within one period.
        // Plus, the timer counter is synchronized with the Timestamp's lower 16 bits.
        uint16_t time_from_pulse_stop = (uint16_t)(cur_time.get_raw_value() & 0xFFFF) - pulse_stop;
        if (time_from_pulse_stop > 0xFF00) time_from_pulse_stop = 0;   // Overflow precaution.
        TimeDelta time_from_pulse_stop_ts(time_from_pulse_stop, (TimeUnit)1);

        Timestamp pulse_start_ts(cur_time - time_from_pulse_stop_ts - pulse_len_ts);
        TimeDelta min_long_pulse_len(40, usec);
        TimeDelta max_long_pulse_len(300, usec);
        if (pulse_len_ts > max_long_pulse_len)
        {
/*            SerialUSB.print("too long ");
            SerialUSB.print(pulse_len_ts.get_value((TimeUnit)1));
            SerialUSB.print(" >  ");
            SerialUSB.print(max_long_pulse_len.get_value((TimeUnit)1));
            SerialUSB.print("\n ");*/

        }else if(pulse_len_ts >= min_long_pulse_len){

         //   SerialUSB.println("long pulse");
        }else{
          //  SerialUSB.println("short pulse");
        }
/*
        SerialUSB.print("pulse: ");
        SerialUSB.print(pulse_len_ts.get_value(usec));
        SerialUSB.print(" ");
        SerialUSB.print(pulse_len);
        SerialUSB.print(" ");
        SerialUSB.print(pulse_start);
        //SerialUSB.print(min_long_pulse_len.get_value((TimeUnit)1));
        SerialUSB.print("\n");
*/
        InputNode::enqueue_pulse(pulse_start_ts, pulse_len_ts);
        rising_received_ = false;
    }


}


void TCC0_Handler()
{
    if (connecteNodes_[0])
    {
        if (TCC0->INTFLAG.bit.MC0 || TCC0->INTFLAG.bit.MC1) {
          //  SerialUSB.println("TCC0_handler");
            //uint16_t start =  TCC0->CC[0].bit.CC;
            uint16_t start =  TCC0->CC[0].bit.CC;
            uint16_t end  =  TCC0->CC[1].bit.CC;
    /*        SerialUSB.print("raw: ");
            SerialUSB.print(start/12);
            SerialUSB.print(" ");
            SerialUSB.print(pulse_len/12);
            SerialUSB.print("\n");*/
            connecteNodes_[0]->irqHandler(start, end);
        }
    }
    if (connecteNodes_[1])
    {
        if (TCC0->INTFLAG.bit.MC2 || TCC0->INTFLAG.bit.MC3) {
          //  SerialUSB.println("TCC0_handler");
            //uint16_t start =  TCC0->CC[0].bit.CC;
            uint16_t start =  TCC0->CC[2].bit.CC;
            uint16_t end  =  TCC0->CC[3].bit.CC;
    /*        SerialUSB.print("raw: ");
            SerialUSB.print(start/12);
            SerialUSB.print(" ");
            SerialUSB.print(pulse_len/12);
            SerialUSB.print("\n");*/
            connecteNodes_[1]->irqHandler(start, end);
        }
    }
}

void TCC1_Handler()
{
    if (connecteNodes_[2])
    {
         //   SerialUSB.println("TCC1_handler");
        uint16_t start =  TCC1->CC[0].bit.CC;
        uint16_t end =  TCC1->CC[1].bit.CC;
        connecteNodes_[2]->irqHandler(start, end);
    }
}

void TCC2_Handler()
{
    if (connecteNodes_[3])
    {
         //   SerialUSB.println("TCC1_handler");
        uint16_t start =  TCC2->CC[0].bit.CC;
        uint16_t end =  TCC2->CC[1].bit.CC;
        connecteNodes_[3]->irqHandler(start, end);
    }
}

void TC3_Handler()
{
    if (connecteNodes_[4]){
     //   if (TC3->COUNT16.INTFLAG.bit.MC0) {
          //  SerialUSB.println("TC3_handler");
          //
           // uint16_t start = TC3->COUNT16.CC[0].bit.CC;
            uint16_t start = TC3->COUNT16.COUNT.reg;
            uint16_t pulse_len = TC3->COUNT16.CC[1].bit.CC;;
            uint16_t end = start + pulse_len;
            connecteNodes_[4]->irqHandler(start, end);
      //  }
    }
}

void TC4_Handler()
{
    if (connecteNodes_[4]){
        if (TC4->COUNT16.INTFLAG.bit.MC0) {
           // SerialUSB.println("TC4_handler");
            uint16_t start = TC4->COUNT16.CC[0].bit.CC;
            uint16_t pulse_len = TC4->COUNT16.CC[1].bit.CC;;
            uint16_t end = start + pulse_len;
            connecteNodes_[4]->irqHandler(start, end);
        }
    }
}

void TC5_Handler()
{
    if (connecteNodes_[5]){
        if (TC5->COUNT16.INTFLAG.bit.MC0) {
            //SerialUSB.println("TC5_handler");
            uint16_t start = TC5->COUNT16.CC[0].bit.CC;
            uint16_t pulse_len = TC5->COUNT16.CC[1].bit.CC;;
            uint16_t end = start + pulse_len;
            connecteNodes_[5]->irqHandler(start, end);
        }
    }
}
