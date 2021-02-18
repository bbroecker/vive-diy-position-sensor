#include "primitives/timestamp.h"
#include "input_tim.h"
#include <SPI.h>

struct PinConfig
{
    unsigned int ioPin;
    unsigned int PORT_PA;
    unsigned int PORT_PA_IDX;
    unsigned int EVSYS_ID_GEN_EIC_EXTINT;
    unsigned int EXTINT;
    unsigned int SENSE_CFG_IDX;
    unsigned int SENSE_IDX;
    unsigned int channel_rising_idx;
    unsigned int channel_falling_idx;
    unsigned int GCLK_CLKCTRL_RISING;
    unsigned int GCLK_CLKCTRL_FALLING;
    unsigned int TCC_IDX;

};

struct TCC_Config
{
    unsigned int TCC_MC_RISING;
    unsigned int TCC_MC_FALLING;
};

const static TCC_Config tcc_config [] = {
    {EVSYS_ID_USER_TCC0_MC_0, EVSYS_ID_USER_TCC0_MC_1},
//    {EVSYS_ID_USER_TCC0_MC_2, EVSYS_ID_USER_TCC0_MC_3},
    {EVSYS_ID_USER_TCC1_MC_0, EVSYS_ID_USER_TCC1_MC_1},
 //   {EVSYS_ID_USER_TCC1_MC_2, EVSYS_ID_USER_TCC1_MC_3},
};


const static PinConfig available_congfigs[] = {
    //{7, PORT_PA21, 21, EVSYS_ID_GEN_EIC_EXTINT_5, 5, 0, 5, 2, 3, GCLK_CLKCTRL_ID_EVSYS_2, GCLK_CLKCTRL_ID_EVSYS_3, 0}, // TINY Pin 7
    //{3, PORT_PA09, 9, EVSYS_ID_GEN_EIC_EXTINT_9, 9, 1, 1, 0, 1, GCLK_CLKCTRL_ID_EVSYS_0, GCLK_CLKCTRL_ID_EVSYS_1, 1} // Tiny Pin 3

    {10, PORT_PA10, 10, EVSYS_ID_GEN_EIC_EXTINT_10, 10, 1, 2, 2, 3, GCLK_CLKCTRL_ID_EVSYS_2, GCLK_CLKCTRL_ID_EVSYS_3, 0}, //XIAO PIN 4
    {9, PORT_PA09, 9, EVSYS_ID_GEN_EIC_EXTINT_9, 9, 1, 1, 0, 1, GCLK_CLKCTRL_ID_EVSYS_0, GCLK_CLKCTRL_ID_EVSYS_1, 1}, // XIA PIN 5
    {21, PORT_PA21, 21, EVSYS_ID_GEN_EIC_EXTINT_5, 5, 0, 5, 2, 3, GCLK_CLKCTRL_ID_EVSYS_2, GCLK_CLKCTRL_ID_EVSYS_3, 0} // TINY Pin 7
};

static InputTimNode* connecteNodes_[2];

static bool timer_initialized = false;

InputTimNode::InputTimNode(uint32_t input_idx, const InputDef &input_def)
    : InputNode(input_idx)
    , pulse_polarity_(input_def.pulse_polarity), rising_received_(false) {


    // Find config
    pin_config_ = find_config_for_pin(input_def.pin);
    if (!pin_config_)
        throw_printf("Pin %d has no available config.", input_def.pin);
    else
        found_config=true;

    if (connecteNodes_[pin_config_->TCC_IDX]){
        throw_printf("TCC %d is already takenk", input_idx);
    }
    else{
        nodes_set=true;

    }
    connecteNodes_[pin_config_->TCC_IDX] = this;

    config_idx = pin_config_->TCC_IDX;
    ioPin = pin_config_->ioPin;

}


InputTimNode::~InputTimNode() {
//    input_cmps[cmp_idx_] = nullptr;
}

void InputTimNode::start()
{
  //configure the timing clock we'll use for counting cycles between IR pules
//  if (!timer_initialized){
    setupClock();

//    setupTimer();
    timer_initialized = true;
//  }

  connectPortPinsToInterrupt();

  //setup our external interrupt controller
    setupEIC();

    connectInterruptsToTimer();

    setupTimer();

  SerialUSB.println("start");

}


InputNode::CreatorRegistrar InputTimNode::creator_([](uint32_t input_idx, const InputDef &input_def) -> std::unique_ptr<InputNode> {
    if (input_def.input_type == InputType::kTimer)
        return std::make_unique<InputTimNode>(input_idx, input_def);
    return nullptr;
});


const PinConfig* InputTimNode::find_config_for_pin(const uint8_t &pin)
{
    const PinConfig *result = nullptr;
    for (auto it = std::begin(available_congfigs); it != std::end(available_congfigs); ++it)
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
    /*SerialUSB.print("nodes_set: " );
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

void InputTimNode::setupClock()
{

    assert(F_CPU % sec == 0); // Time resolution must be whole number of timer ticks
    /*
    // DFLL default is open loop mode:
    SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL (48000);   // Set to multiply USB SOF frequency (when USB attached)https://www.reddit.com/r/fedmyster/
    uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP4) + (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32)) >> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32) )
                   & ((1 << 6) - 1);
    if (coarse == 0x3f) {
        coarse = 0x1f;
    }
    // TODO(tannewt): Load this value from memory we've written previously. There
    // isn't a value from the Atmel factory.
    uint32_t fine = 0x1ff;
    SYSCTRL->DFLLVAL.reg =
    SYSCTRL_DFLLVAL_COARSE(coarse) |
    SYSCTRL_DFLLVAL_FINE(fine);
    */

    SYSCTRL->DFLLCTRL.reg =
    // SYSCTRL_DFLLCTRL_WAITLOCK |                     //output clock when DFLL is locked
    // SYSCTRL_DFLLCTRL_BPLCKC |                       //bypass coarse lock
    // SYSCTRL_DFLLCTRL_QLDIS |                        //disable quick lock
    SYSCTRL_DFLLCTRL_CCDIS |                        //disable chill cycle
    // SYSCTRL_DFLLCTRL_STABLE |                       //stable frequency mode; testing indicates this poorly skews detection results
    SYSCTRL_DFLLCTRL_MODE |                         //closed-loop mode
    SYSCTRL_DFLLCTRL_ENABLE;
    while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

    //setup the divisor for the GCLK0 clock source generator
   // REG_GCLK_GENDIV = GCLK_GENDIV_DIV((F_CPU / sec) * 1) |                                  //do not divide the input clock (48MHz / 1)
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(0) |                                  //do not divide the input clock (48MHz / 1)
                    GCLK_GENDIV_ID(3);                                    //for GCLK3

    //  SYSCTRL->OSC32K.bit.ENABLE = 1;
    //  SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_STARTUP(0x6u);
    //configure GCLK0 and enable it
    REG_GCLK_GENCTRL =
       GCLK_GENCTRL_IDC |                                   //50/50 duty cycles; optimization when dividing input clock by an odd number
    GCLK_GENCTRL_GENEN |                                 //enable the clock generator
    GCLK_GENCTRL_SRC_DFLL48M |                           //set the clock source to 48MHz
    //    GCLK_GENCTRL_SRC_OSC32K |                            //set the clock source to high-accuracy 32KHz clock
    //    GCLK_GENCTRL_SRC_XOSC |                              //set the clock source to 32MHz
    //    GCLK_GENCTRL_SRC_DPLL32K |                          //set the clock source to 48MHz
    //    (0x08 << 8) |
    GCLK_GENCTRL_ID(3);                                  //for GCLK3
    while (GCLK->STATUS.bit.SYNCBUSY);



    //setup the clock output to go to the EIC
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
                     GCLK_CLKCTRL_ID_EIC;                                 //to the EIC peripheral

    //setup the clock output to go to EVSYS channel
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
                    pin_config_->GCLK_CLKCTRL_RISING;              //to EVSYS channel RISING

    //setup the clock output to go to EVSYS channel
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
                    pin_config_->GCLK_CLKCTRL_FALLING;              //to EVSYS channel FALLING

    //setup the clock output to go to the TCC
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3GCLK_CLKCTRL_ID_EVSYS_5_Val
                     GCLK_CLKCTRL_ID_TCC0_TCC1;                           //to TCC0 and TCC1
    //wait for synchronization
    while (GCLK->STATUS.bit.SYNCBUSY);
}

void InputTimNode::connectPortPinsToInterrupt()
{
    //enable the PORT subsystem
    PM->APBBMASK.bit.PORT_ = 1;

    //set port A (group 0), pin 21 (PA21, Tinyduino proto board pin IO7) as an input
    PORT->Group[0].DIRCLR.reg = pin_config_->PORT_PA;      //set port to input direction
    PORT->Group[0].OUTCLR.reg = pin_config_->PORT_PA;      //pull-down when pull is enabled
    PORT->Group[0].CTRL.reg |=  pin_config_->PORT_PA;       //enable input sampling

    //configure PA21
    PORT->Group[0].PINCFG[pin_config_->PORT_PA_IDX].reg =
    PORT_PINCFG_PULLEN |         //enable pull-down
    PORT_PINCFG_INEN |           //enable input buffering
    PORT_PINCFG_PMUXEN;          //enable pin muxing

    // //mux PA21 over to EXTINT5
    PORT->Group[0].PMUX[pin_config_->PORT_PA_IDX >> 1].reg = PORT_PMUX_PMUXO(PORT_PMUX_PMUXE_A_Val);

}

void InputTimNode::setupEIC()
{
    //turn on power to the external interrupt controller (EIC)
    PM->APBAMASK.bit.EIC_ = 1;

    //disable the EIC while we configure it
    EIC->CTRL.bit.ENABLE = 0;
    while (EIC->STATUS.bit.SYNCBUSY);

    //right diode interrupt config
    //  EIC->CONFIG[1].bit.FILTEN1 = 1;
    //detect high level detection (both edges)
    EIC->CONFIG[pin_config_->SENSE_CFG_IDX].reg |= EIC_CONFIG_SENSE1_HIGH_Val << 4 * pin_config_->SENSE_IDX;
    //EIC->CONFIG[cfg_bit].bit.SENSE0 = EIC_CONFIG_SENSE1_HIGH_Val << 4 * sens;
    //generate interrupts on interrupt #9 when edges are detected

   // EIC->INTENCLR.bit.EXTINT0   = (1 << pin_config_->EXTINT); // p.468 The external interrupt 9 is enabled
    EIC->EVCTRL.reg |= (0x01 << pin_config_->EXTINT); // p.467 Event from pin 9 is enabled and will be generated when this pin matches the external interrupt sensing config.

    //enable the EIC
    EIC->CTRL.bit.ENABLE = 1;

    //wait for synchronization
    while (EIC->STATUS.bit.SYNCBUSY);
}

bool USRRDYForChannel(const uint8_t &channel)
{
    switch(channel)
    {
        case 0 :
            return EVSYS->CHSTATUS.bit.USRRDY0;
        case 1 :
            return EVSYS->CHSTATUS.bit.USRRDY1;
        case 2 :
            return EVSYS->CHSTATUS.bit.USRRDY2;
        case 3 :
            return EVSYS->CHSTATUS.bit.USRRDY3;
        case 4 :
            return EVSYS->CHSTATUS.bit.USRRDY4;
        case 5 :
            return EVSYS->CHSTATUS.bit.USRRDY5;
        case 6 :
            return EVSYS->CHSTATUS.bit.USRRDY6;
        case 7 :
            return EVSYS->CHSTATUS.bit.USRRDY7;
        case 8 :
            return EVSYS->CHSTATUS.bit.USRRDY8;
        case 9 :
            return EVSYS->CHSTATUS.bit.USRRDY9;
    }
    return false;
}

void InputTimNode::connectInterruptsToTimer()
{
    //enable the event subsystem
    PM->APBCMASK.bit.EVSYS_ = 1;

    //input config for right diode
    REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(1) |                           //detect rising edge
    //                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(pin_config_->EVSYS_ID_GEN_EIC_EXTINT) |    //from external interrupt 9
                      EVSYS_CHANNEL_CHANNEL(pin_config_->channel_rising_idx);                           //to EVSYS channel 0

    //output config for right diode
    REG_EVSYS_USER = EVSYS_USER_CHANNEL(pin_config_->channel_rising_idx + 1) |                                //attach output from channel 0 (n+1)
                   EVSYS_USER_USER(pulse_polarity_? tcc_config[pin_config_->TCC_IDX].TCC_MC_RISING : tcc_config[pin_config_->TCC_IDX].TCC_MC_FALLING);              //to user (recipient) TCC0, MC0
    while (!USRRDYForChannel(pin_config_->channel_rising_idx));

    //input config for right diode
    REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(2) |                           //detect falling edge
    //                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(pin_config_->EVSYS_ID_GEN_EIC_EXTINT) |    //from external interrupt 9
                      EVSYS_CHANNEL_CHANNEL(pin_config_->channel_falling_idx);                           //to EVSYS channel 1

    //output config for right diode
    REG_EVSYS_USER = EVSYS_USER_CHANNEL(pin_config_->channel_falling_idx + 1) |                                //attach output from channel 1 (n+1)
                   EVSYS_USER_USER(pulse_polarity_? tcc_config[pin_config_->TCC_IDX].TCC_MC_FALLING : tcc_config[pin_config_->TCC_IDX].TCC_MC_RISING);              //to user (recipient) TCC0, MC1
    while (!USRRDYForChannel(pin_config_->channel_falling_idx));// change but not tested
 //   connectExt4();

}

void InputTimNode::connectExt4()
{
    PM->APBAMASK.bit.EIC_ = 1;

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC);
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Input on PA04
    PORT->Group[0].PMUX[4/2].bit.PMUXE = MUX_PA04A_EIC_EXTINT4;
   /* const PORT_PINCFG_Type pincfg = {
        .bit.PMUXEN = 1,
        .bit.INEN = 1,
        .bit.PULLEN = 1
    };
    PORT->Group[0].PINCFG[4].reg = pincfg.reg;
*/
    EIC->CONFIG[0].bit.SENSE4 = EIC_CONFIG_SENSE4_HIGH_Val; // None of RISE,FALL or BOTH work for this
    EIC->EVCTRL.bit.EXTINTEO4 = 1;

    // The EIC interrupt is only for verifying the input
    //REG_EIC_INTENSET = EIC_INTFLAG_EXTINT4;
    //NVIC_EnableIRQ(EIC_IRQn);
    while (EIC->STATUS.reg & EIC_STATUS_SYNCBUSY);
    EIC->CTRL.bit.ENABLE = 1;
    while (EIC->STATUS.reg & EIC_STATUS_SYNCBUSY);
}

uint16_t TCC_CTRLA_PRESCALER_DIV(int div)
{
    switch (div)
    {
        case 1 : return TCC_CTRLA_PRESCALER_DIV1;
        case 2 : return TCC_CTRLA_PRESCALER_DIV2;
        case 4 : return TCC_CTRLA_PRESCALER_DIV4;
        case 8 : return TCC_CTRLA_PRESCALER_DIV8;
        default : return TCC_CTRLA_PRESCALER_DIV1;
    }
}


void InputTimNode::setupTimer()
{
    setupTimerTCC0();
    setupTimerTCC1();
  //  setupTimerTC3();
}

void InputTimNode::setupTimerTCC0()
{

    //enable the TCC0 subsystem
    PM->APBCMASK.bit.TCC0_ = 1;

    //disable TCC0 while we configure it
    REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE;

    //configure TCC0
    REG_TCC0_CTRLA =
    TCC_CTRLA_CPTEN0 |              //place MC0 into capture (not compare) mode
    TCC_CTRLA_CPTEN1 |              //place MC1 into capture (not compare) mode
    TCC_CTRLA_CPTEN2 |            //place MC2 into capture (not compare) mode
    TCC_CTRLA_CPTEN3 |              //place MC3 into capture (not compare) mode
    //    TCC_CTRLA_RESOLUTION_DITH4 |
    //    TCC_CTRLA_ALOCK |
    //    TCC_CTRLA_PRESCSYNC_GCLK |
    //TCC_CTRLA_PRESCALER_DIV4;       //set timer prescaler to 1 (48MHz)
    TCC_CTRLA_PRESCALER_DIV(F_CPU / sec);

    //set the event control register
    REG_TCC0_EVCTRL =
    TCC_EVCTRL_MCEI0 |              //when MC0 events occur, capture COUNT to CC0
    TCC_EVCTRL_MCEI1 |               //when MC1 events occur, capture COUNT to CC1
    TCC_EVCTRL_MCEI2 |             //when MC2 events occur, capture COUNT to CC2
    TCC_EVCTRL_MCEI3 ;             //when MC3 events occur, capture COUNT to CC3
  //      TCC_EVCTRL_TCEI1 |             //enable the event 1 input
  //      TCC_EVCTRL_TCEI0 |             //enable the event 0 input
   //  TCC_EVCTRL_TCINV1 |             //enable the event 1 inverted input
   //  TCC_EVCTRL_TCINV0 ;             //enable the event 0 inverted input
    //    TCC_EVCTRL_CNTEO |
    //    TCC_EVCTRL_TRGEO |
    //    TCC_EVCTRL_OVFEO |
    //    TCC_EVCTRL_CNTSEL_BOUNDARY |
    //    TCC_EVCTRL_EVACT1_RETRIGGER |  //retrigger CC1 on event 1 (each time an edge is detected)
    //    TCC_EVCTRL_EVACT0_RETRIGGER;   //retrigger CC0 on event 0 (each time an edge is detected)

    //TCC0->EVCTRL.bit.TCINV0 = 1;
    //TCC0->EVCTRL.bit.TCINV1 = 1;
    //

    //setup our desired interrupts
    REG_TCC0_INTENSET =
    TCC_INTENSET_MC0 |              //enable interrupts when a capture occurs on MC0 maybe remove ??
    TCC_INTENSET_MC1 |               //enable interrupts when a capture occurs on MC1
    TCC_INTENSET_MC2 |            //enable interrupts when a capture occurs on MC2
    TCC_INTENSET_MC3 ;            //enable interrupts when a capture occurs on MC3
    //    TCC_INTENSET_CNT |            //enable interrupts for every tick of the counter
    //    TCC_INTENSET_OVF |            //enable interrupts on overflow
    //    TCC_INTENSET_TRG;             //enable interrupts on retrigger

    //connect the interrupt handler for TCC0
    NVIC_SetPriority(TCC0_IRQn, 1);
    NVIC_EnableIRQ(TCC0_IRQn);

    //enable TCC0
    REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE;

    //wait for TCC0 synchronization
    while (TCC0->SYNCBUSY.bit.ENABLE);

}

void InputTimNode::setupTimerTCC1()
{

    //enable the TCC1 subsystem
    PM->APBCMASK.bit.TCC1_ = 1;

    //disable TCC1 while we configure it
    REG_TCC1_CTRLA &= ~TCC_CTRLA_ENABLE;

    //configure TCC1
    REG_TCC1_CTRLA =
    TCC_CTRLA_CPTEN0 |              //place MC0 into capture (not compare) mode
    TCC_CTRLA_CPTEN1 |              //place MC1 into capture (not compare) mode
    TCC_CTRLA_CPTEN2 |              //place MC2 into capture (not compare) mode
    TCC_CTRLA_CPTEN3 |              //place MC3 into capture (not compare) mode
    TCC_CTRLA_PRESCALER_DIV(F_CPU / sec);
    //TCC_CTRLA_PRESCALER_DIV4;       //set timer prescaler to 1 (48MHz)

    REG_TCC1_EVCTRL =
    TCC_EVCTRL_MCEI0 |              //when MC0 events occur, capture COUNT to CC0
    TCC_EVCTRL_MCEI1 |             //when MC1 events occur, capture COUNT to CC1
    TCC_EVCTRL_MCEI2 |             //when MC1 events occur, capture COUNT to CC1
    TCC_EVCTRL_MCEI3 ;             //when MC1 events occur, capture COUNT to CC1
    //  TCC_EVCTRL_TCINV1 |             //enable the event 1 inverted input
     //TCC_EVCTRL_TCINV0 ;             //enable the event 0 inverted input


    REG_TCC1_INTENSET =
    TCC_INTENSET_MC0 |              //enable interrupts when a capture occurs on TCC1/MC0 Maybe remove ???
    TCC_INTENSET_MC1 |               //enable interrupts when a capture occurs on TCC1/MC1
    TCC_INTENSET_MC2 |               //enable interrupts when a capture occurs on TCC1/MC1
    TCC_INTENSET_MC3;               //enable interrupts when a capture occurs on TCC1/MC1

    //connect the interrupt handler for TCC1
    NVIC_SetPriority(TCC1_IRQn, 1);
    NVIC_EnableIRQ(TCC1_IRQn);

    //enable TCC1
    REG_TCC1_CTRLA |= TCC_CTRLA_ENABLE;

    //wait for synchronization
    while (TCC1->SYNCBUSY.bit.ENABLE);

}

void InputTimNode::setupTimerTC3()
{
    PM->APBCMASK.bit.TC3_ = 1;

    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3;
    while ( GCLK->STATUS.bit.SYNCBUSY);

    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC));
    while ( GCLK->STATUS.bit.SYNCBUSY);

    TC3->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
    TC3->COUNT16.CTRLA.bit.MODE = TC_CTRLA_MODE_COUNT16;
    TC3->COUNT16.CTRLA.bit.PRESCALER |= TC_CTRLA_PRESCALER_DIV1_Val;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
    TC3->COUNT16.CTRLC.bit.CPTEN0 = 1;
    TC3->COUNT16.CTRLC.bit.CPTEN1 = 1;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
    TC3->COUNT16.EVCTRL.bit.TCEI = 1;
    //TC3->COUNT16.EVCTRL.bit.TCINV = 1; // Optional invert
    TC3->COUNT16.EVCTRL.bit.EVACT = TC_EVCTRL_EVACT_PPW_Val;
    // Interrupts
    TC3->COUNT16.INTENSET.bit.MC0 = 1;
    //TC3->COUNT16.INTENSET.bit.MC1 = 1; // Not needed, can read out both in MC0 interrupt

    // Enable InterruptVector
    NVIC_EnableIRQ(TC3_IRQn);

    // Enable TC
    TC3->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

    PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;

    // EVSYS GCLK is needed for interrupts and sync path (so not now)
    //GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EVSYS_1);
    //while (GCLK->STATUS.bit.SYNCBUSY);
    REG_EVSYS_USER = EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU)|EVSYS_USER_CHANNEL(2); // Channel n-1 selected so 1 here
    while(!EVSYS->CHSTATUS.bit.USRRDY1);

    // EVSYS_CHANNEL_EDGSEL_BOTH_EDGES // not usable with  PATH_ASYNCHRONOUS
    REG_EVSYS_CHANNEL = EVSYS_CHANNEL_PATH_ASYNCHRONOUS|EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_4)|EVSYS_CHANNEL_CHANNEL(1);
    while(EVSYS->CHSTATUS.bit.CHBUSY1);
    // Below interrupt is for testing the event (not possible with PATH_ASYNCHRONOUS)
    //EVSYS->INTENSET.reg = EVSYS_INTENSET_EVD1;
    //NVIC_EnableIRQ(EVSYS_IRQn);
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

            //SerialUSB.println("long pulse");
        }else{
            //SerialUSB.println("short pulse");
        }/*
        SerialUSB.print(pulse_len_ts.get_value(usec));
        SerialUSB.print(" ");
        SerialUSB.print(pulse_start_ts.get_value(usec));
        SerialUSB.print(" ");
        SerialUSB.print(pulse_start);
        SerialUSB.print("\n");*/
        InputNode::enqueue_pulse(pulse_start_ts, pulse_len_ts);
        rising_received_ = false;
    }


}

void TCC0_Handler()
{
    //SerialUSB.println("\nTCC0_handler");
    if (connecteNodes_[0])
    {
        uint16_t start = REG_TCC0_CC0;
        uint16_t end = REG_TCC0_CC1;
        connecteNodes_[0]->irqHandler(start, end);
    }
}

void TCC1_Handler()
{
   // SerialUSB.println("\nTCC1_handler");
    if (connecteNodes_[1])
    {
        uint16_t start = REG_TCC1_CC0;
        uint16_t end = REG_TCC1_CC1;
        connecteNodes_[1]->irqHandler(start, end);
    }
}

void TC3_Handler()
{
    /*if (TC3->COUNT16.INTFLAG.bit.MC0) {
        // The interrupt flag is cleared by reading CC
        cap0[nCap] = TC3->COUNT16.CC[0].bit.CC;
        cap1[nCap] = TC3->COUNT16.CC[1].bit.CC;
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    }*/
    SerialUSB.println("HandlerTC3");
}
