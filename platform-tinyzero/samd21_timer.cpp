//#include "sam.h"
#include "samd21_timer.h"


bool BusyForChannel(const uint8_t &channel)
{
    switch(channel)
    {
        case 0 :
            return EVSYS->CHSTATUS.bit.CHBUSY0;
        case 1 :
            return EVSYS->CHSTATUS.bit.CHBUSY1;
        case 2 :
            return EVSYS->CHSTATUS.bit.CHBUSY2;
        case 3 :
            return EVSYS->CHSTATUS.bit.CHBUSY3;
        case 4 :
            return EVSYS->CHSTATUS.bit.CHBUSY4;
        case 5 :
            return EVSYS->CHSTATUS.bit.CHBUSY5;
        case 6 :
            return EVSYS->CHSTATUS.bit.CHBUSY6;
        case 7 :
            return EVSYS->CHSTATUS.bit.CHBUSY7;
        case 8 :
            return EVSYS->CHSTATUS.bit.CHBUSY8;
        case 9 :
            return EVSYS->CHSTATUS.bit.CHBUSY9;
    }
    return false;
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

void connectPortPinsToInterrupt(uint16_t port_pa, uint16_t ext_int)
{

    PM->APBBMASK.bit.PORT_ = 1;

    //set port A (group 0), pin 21 (PA21, Tinyduino proto board pin IO7) as an input
    PORT->Group[0].DIRCLR.reg = port_pa;      //set port to input direction
    PORT->Group[0].OUTCLR.reg = port_pa;      //pull-down when pull is enabled
    PORT->Group[0].CTRL.reg |=  port_pa;       //enable input sampling

    //configure PA21
    PORT->Group[0].PINCFG[ext_int].reg =
    PORT_PINCFG_PULLEN |         //enable pull-down
    PORT_PINCFG_INEN |           //enable input buffering
    PORT_PINCFG_PMUXEN;          //enable pin muxing

    // //mux PA21 over to EXTINT5
    PORT->Group[0].PMUX[ext_int >> 1].reg = PORT_PMUX_PMUXO(PORT_PMUX_PMUXE_A_Val);

}

void setupEIC(uint16_t ext_int)
{
    //turn on power to the external interrupt controller (EIC)
    PM->APBAMASK.bit.EIC_ = 1;

    //disable the EIC while we configure it
    EIC->CTRL.bit.ENABLE = 0;
    while (EIC->STATUS.bit.SYNCBUSY);

    //right diode interrupt config
    //  EIC->CONFIG[1].bit.FILTEN1 = 1;
    //detect high level detection (both edges)
    EIC->CONFIG[(int)(ext_int / 8)].reg |= EIC_CONFIG_SENSE1_HIGH_Val << (4 * (ext_int % 8));
    //EIC->CONFIG[cfg_bit].bit.SENSE0 = EIC_CONFIG_SENSE1_HIGH_Val << 4 * sens;
    //generate interrupts on interrupt #9 when edges are detected

   // EIC->INTENCLR.bit.EXTINT0   = (1 << pin_config_->EXTINT); // p.468 The external interrupt 9 is enabled
    EIC->EVCTRL.reg |= (0x01 << ext_int); // p.467 Event from pin 9 is enabled and will be generated when this pin matches the external interrupt sensing config.

    //enable the EIC
    EIC->CTRL.bit.ENABLE = 1;

    //wait for synchronization
    while (EIC->STATUS.bit.SYNCBUSY);
}

uint16_t TC_CTRLA_PRESCALER_DIV_Val(uint16_t div)
{
    switch (div)
    {
        case 1 : return TC_CTRLA_PRESCALER_DIV1_Val;
        case 2 : return TC_CTRLA_PRESCALER_DIV2_Val;
        case 4 : return TC_CTRLA_PRESCALER_DIV4_Val;
        case 8 : return TC_CTRLA_PRESCALER_DIV8_Val;
        default : return TC_CTRLA_PRESCALER_DIV1_Val;
    }
}



void setupTimer(Tc *tc, uint16_t clkctrl_tc, uint16_t ext_int, uint16_t EVSYS_ID_USER, uint16_t channel, uint32_t tc_enable_bit, IRQn_Type tc_irqn, bool invert_input, uint16_t prescaler_div)
{
    //PM->APBCMASK.bit.TC3_ = 1;
    PM->APBCMASK.reg |= tc_enable_bit;
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | clkctrl_tc;
    while ( GCLK->STATUS.bit.SYNCBUSY);


    tc->COUNT16.CTRLA.bit.ENABLE = 0;
    while (tc->COUNT16.STATUS.bit.SYNCBUSY);

   // GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC));
    //while ( GCLK->STATUS.bit.SYNCBUSY);

    tc->COUNT16.CTRLA.bit.MODE = TC_CTRLA_MODE_COUNT16;
    tc->COUNT16.CTRLA.bit.PRESCALER |= TC_CTRLA_PRESCALER_DIV_Val(prescaler_div);
    while (tc->COUNT16.STATUS.bit.SYNCBUSY);
    tc->COUNT16.CTRLC.bit.CPTEN0 = 1;
    tc->COUNT16.CTRLC.bit.CPTEN1 = 1;
    while (tc->COUNT16.STATUS.bit.SYNCBUSY);
    tc->COUNT16.EVCTRL.bit.TCEI = 1;

    if (invert_input){
        tc->COUNT16.EVCTRL.bit.TCINV = 1; // Optional invert
    }

//       tc->EVCTRL.bit.TCINV0 = 1;
 //       tc->EVCTRL.bit.TCINV1 = 1;


    tc->COUNT16.EVCTRL.bit.EVACT = TC_EVCTRL_EVACT_PPW_Val;
    // Interrupts
    tc->COUNT16.INTENSET.bit.MC0 = 1;
    tc->COUNT16.INTENSET.bit.MC1 = 1; // Not needed, can read out both in MC0 interrupt

    // Enable InterruptVector
    NVIC_SetPriority(tc_irqn, 1);
    NVIC_EnableIRQ(tc_irqn);

    // Enable TC
    tc->COUNT16.CTRLA.bit.ENABLE = 1;
    while (tc->COUNT16.STATUS.bit.SYNCBUSY);

    PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;

    // EVSYS GCLK is needed for interrupts and sync path (so not now)
    //GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EVSYS_1);
    //while (GCLK->STATUS.bit.SYNCBUSY);
    REG_EVSYS_USER = EVSYS_USER_USER(EVSYS_ID_USER)|EVSYS_USER_CHANNEL(channel + 1); // Channel n-1 selected so 1 here
    while(!USRRDYForChannel(channel));

    // EVSYS_CHANNEL_EDGSEL_BOTH_EDGES // not usable with  PATH_ASYNCHRONOUS
    REG_EVSYS_CHANNEL = EVSYS_CHANNEL_PATH_ASYNCHRONOUS|EVSYS_CHANNEL_EVGEN(ext_int)|EVSYS_CHANNEL_CHANNEL(channel);
 //   REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EVGEN(ext_int)|EVSYS_CHANNEL_CHANNEL(channel);
    while(BusyForChannel(channel));
    // Below interrupt is for testing the event (not possible with PATH_ASYNCHRONOUS)
    //EVSYS->INTENSET.reg = EVSYS_INTENSET_EVD1;
    //NVIC_EnableIRQ(EVSYS_IRQn);
}


void setupClock(uint16_t gclk_id)
{

    //assert(F_CPU % sec == 0); // Time resolution must be whole number of timer ticks
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
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(0) |                                  //do not divide the input clock (48MHz / 1) uint16_t prescaler_div
                    GCLK_GENDIV_ID(gclk_id);                                    //for GCLK3

    //  SYSCTRL->OSC32K.bit.ENABLE = 1;
    //  SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_STARTUP(0x6u);
    //configure GCLK0 and enable it
    REG_GCLK_GENCTRL =
     //  GCLK_GENCTRL_IDC |                                   //50/50 duty cycles; optimization when dividing input clock by an odd number
    GCLK_GENCTRL_GENEN |                                 //enable the clock generator
    GCLK_GENCTRL_SRC_DFLL48M |                           //set the clock source to 48MHz
    //    GCLK_GENCTRL_SRC_OSC32K |                            //set the clock source to high-accuracy 32KHz clock
    //    GCLK_GENCTRL_SRC_XOSC |                              //set the clock source to 32MHz
    //    GCLK_GENCTRL_SRC_DPLL32K |                          //set the clock source to 48MHz
    //    (0x08 << 8) |
    GCLK_GENCTRL_ID(gclk_id);                                  //for GCLK3
    while (GCLK->STATUS.bit.SYNCBUSY);



}

uint16_t TCC_CTRLA_PRESCALER_DIV(uint16_t div)
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

uint16_t getGCLK_CLKCTRL_ID_EVSYS(uint16_t idx){
    switch (idx){
        case 0 : return GCLK_CLKCTRL_ID_EVSYS_0;
        case 1 : return GCLK_CLKCTRL_ID_EVSYS_1;
        case 2 : return GCLK_CLKCTRL_ID_EVSYS_2;
        case 3 : return GCLK_CLKCTRL_ID_EVSYS_3;
        case 4 : return GCLK_CLKCTRL_ID_EVSYS_4;
        case 5 : return GCLK_CLKCTRL_ID_EVSYS_5;
        case 6 : return GCLK_CLKCTRL_ID_EVSYS_6;
        case 7 : return GCLK_CLKCTRL_ID_EVSYS_7;
        case 8 : return GCLK_CLKCTRL_ID_EVSYS_8;
        case 9 : return GCLK_CLKCTRL_ID_EVSYS_9;
        case 10 : return GCLK_CLKCTRL_ID_EVSYS_10;
        case 11 : return GCLK_CLKCTRL_ID_EVSYS_11;
        default: __builtin_unreachable();
    }

}


void setupTimer(Tcc *tc, uint16_t clkctrl_tc, uint16_t ext_int, uint16_t channel,
        uint32_t tc_enable_bit, IRQn_Type tc_irqn, bool invert_input, uint16_t prescaler_div,
        uint16_t rising_event_out, uint16_t falling_event_out
        ){
        //PM->APBCMASK.bit.TCC0_ = 1;

    PM->APBCMASK.bit.EVSYS_ = 1;
//setup the clock output to go to the EIC
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
                     GCLK_CLKCTRL_ID_EIC;                                 //to the EIC peripheral

    //setup the clock output to go to EVSYS channel
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
                    //GCLK_CLKCTRL_ID_EVSYS_0;
                    getGCLK_CLKCTRL_ID_EVSYS(channel -1);

    //setup the clock output to go to EVSYS channel
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3
//                    GCLK_CLKCTRL_ID_EVSYS_1;
                    getGCLK_CLKCTRL_ID_EVSYS(channel);

    //setup the clock output to go to the TCC
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |                                 //enable the clock
                     GCLK_CLKCTRL_GEN_GCLK3 |                             //to send GCLK3GCLK_CLKCTRL_ID_EVSYS_5_Val
                     clkctrl_tc;                           //to TCC0 and TCC1
    //wait for synchronization
    while (GCLK->STATUS.bit.SYNCBUSY);

    PM->APBCMASK.bit.EVSYS_ = 1;

    uint16_t rising_user_recv = (invert_input) ? falling_event_out : rising_event_out;
    uint16_t falling_user_recv = (invert_input) ? rising_event_out : falling_event_out;

    //input config for right diode
    REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(1) |                           //detect rising edge (0x1)
    //                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(ext_int) |    //from external interrupt 9
                      EVSYS_CHANNEL_CHANNEL(channel - 1);                           //to EVSYS channel 0

    //output config for right diode
    REG_EVSYS_USER = EVSYS_USER_CHANNEL(channel) |                                //attach output from channel 0 (n+1)
                   EVSYS_USER_USER(rising_user_recv);              //to user (recipient) TCC0, MC0
    while (!USRRDYForChannel(channel - 1));

    //input config for right diode
    REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL(2) |                           //detect falling edge (0x2)
    //                      EVSYS_CHANNEL_PATH_SYNCHRONOUS |                    //synchronously
                      EVSYS_CHANNEL_EVGEN(ext_int) |    //from external interrupt 9
                      EVSYS_CHANNEL_CHANNEL(channel);                           //to EVSYS channel 1

    //output config for right diode
    REG_EVSYS_USER = EVSYS_USER_CHANNEL(channel + 1) |                                //attach output from channel 1 (n+1)
                   EVSYS_USER_USER(falling_user_recv);              //to user (recipient) TCC0, MC1
    while (!USRRDYForChannel(channel));// change but not tested

    //PM->APBCMASK.bit.TCC0_ = 1;
    PM->APBCMASK.reg |= tc_enable_bit;

    //disable TCC0 while we configure it
//    REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE;
    tc->CTRLA.bit.ENABLE = 0;
    while (tc->SYNCBUSY.bit.ENABLE);

    //configure TCC0
    //REG_TCC0_CTRLA =
    tc->CTRLA.reg =
    TCC_CTRLA_CPTEN0 |              //place MC0 into capture (not compare) mode
    TCC_CTRLA_CPTEN1 |              //place MC1 into capture (not compare) mode
    TCC_CTRLA_CPTEN2 |            //place MC2 into capture (not compare) mode
    TCC_CTRLA_CPTEN3 |              //place MC3 into capture (not compare) mode
    //    TCC_CTRLA_RESOLUTION_DITH4 |
    //    TCC_CTRLA_ALOCK |
    //    TCC_CTRLA_PRESCSYNC_GCLK |
    //TCC_CTRLA_PRESCALER_DIV4;       //set timer prescaler to 1 (48MHz)
    TCC_CTRLA_PRESCALER_DIV(prescaler_div);

    //set the event control register
    //REG_TCC0_EVCTRL =
    tc->EVCTRL.reg =
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
    //REG_TCC0_INTENSET =
    tc->INTENSET.reg =
    TCC_INTENSET_MC0 |              //enable interrupts when a capture occurs on MC0 maybe remove ??
    TCC_INTENSET_MC1 |               //enable interrupts when a capture occurs on MC1
    TCC_INTENSET_MC2 |            //enable interrupts when a capture occurs on MC2
    TCC_INTENSET_MC3 ;            //enable interrupts when a capture occurs on MC3
    //    TCC_INTENSET_CNT |            //enable interrupts for every tick of the counter
    //    TCC_INTENSET_OVF |            //enable interrupts on overflow
    //    TCC_INTENSET_TRG;             //enable interrupts on retrigger

    //connect the interrupt handler for TCC0
    NVIC_SetPriority(tc_irqn, 1);
    NVIC_EnableIRQ(tc_irqn);

    // Enable TCC
    tc->CTRLA.bit.ENABLE = 1;
    while (tc->SYNCBUSY.bit.ENABLE); // wait for sync

    //enable TCC0
    //REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE;

    //wait for TCC0 synchronization
    //while (TCC0->SYNCBUSY.bit.ENABLE);
     // Enable TCC


    //enable the event subsystem
 //   connectExt4();


}
/*
void setupTimer(Tcc *tc, uint16_t clkctrl_tc, uint16_t ext_int, uint16_t EVSYS_ID_USER, uint16_t channel, uint32_t tc_enable_bit, IRQn_Type tc_irqn, bool invert_input, uint16_t prescaler_div)
{
    //PM->APBCMASK.bit.TCC0_ = 1;
    PM->APBCMASK.reg |= tc_enable_bit;

    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | clkctrl_tc;
    while (GCLK->STATUS.bit.SYNCBUSY);

    tc->CTRLA.bit.ENABLE = 0;
    while (tc->SYNCBUSY.bit.ENABLE);

    tc->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV(prescaler_div) | TCC_CTRLA_CPTEN0 | TCC_CTRLA_CPTEN1;
    tc->PER.reg = 0xFFFFFF;
    //TCC0->CTRLA.bit.PRESCALER |= TC_CTRLA_PRESCALER_DIV1_Val;

    while (tc->SYNCBUSY.bit.PER);

    tc->EVCTRL.reg = TCC_EVCTRL_TCEI1 |  // enable input event
        TCC_EVCTRL_EVACT1_PPW; // event action = PPW
        // | TCC_EVCTRL_MCEI0 | TCC_EVCTRL_MCEI1;

    // Interrupts
    tc->INTENSET.bit.MC0 = 1;
    //TCC0->INTENSET.bit.MC1 = 1; // Not needed, can read out both in MC0 interrupt
    NVIC_EnableIRQ(tc_irqn);

    if (invert_input){
        tc->EVCTRL.bit.TCINV0 = 1;
        tc->EVCTRL.bit.TCINV1 = 1;
    }

    // Enable TCC
    tc->CTRLA.bit.ENABLE = 1;
    while (tc->SYNCBUSY.bit.ENABLE); // wait for sync

    PM->APBCMASK.bit.EVSYS_ = 1;

    // EVSYS GCLK is needed for interrupts and sync path (so not now)
    // GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EVSYS_1);
    // while (GCLK->STATUS.bit.SYNCBUSY);

    // TCC_EVCTRL_TCEI1 is used, must be matched with EVSYS_ID_USER_TCC0_EV_1 here
   // REG_EVSYS_USER = EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_1)|EVSYS_USER_CHANNEL(channel + 1); // Channel n-1 selected so 1 here
    REG_EVSYS_USER = EVSYS_USER_USER(EVSYS_ID_USER)|EVSYS_USER_CHANNEL(channel + 1); // Channel n-1 selected so 1 here
    while(!USRRDYForChannel(channel));
    // EVSYS_CHANNEL_EDGSEL_BOTH_EDGES // not usable with  PATH_ASYNCHRONOUS
    // For this capture it looks like it needs to be PATH_ASYNCHRONOUS
    REG_EVSYS_CHANNEL = EVSYS_CHANNEL_PATH_ASYNCHRONOUS|EVSYS_CHANNEL_EVGEN(ext_int)|EVSYS_CHANNEL_CHANNEL(channel);
    while(BusyForChannel(channel));
    // Below interrupt is for testing the event (not possible with PATH_ASYNCHRONOUS)
    //EVSYS->INTENSET.reg = EVSYS_INTENSET_EVD1;
    //NVIC_EnableIRQ(EVSYS_IRQn);
}*/
/*
void TC3_Handler()
{
    if (TC3->COUNT16.INTFLAG.bit.MC0) {
        SerialUSB.println("TC3_handler");
        // The interrupt flag is cleared by reading CC
        cap0[nCap] = TC3->COUNT16.CC[0].bit.CC;
        cap1[nCap] = TC3->COUNT16.CC[1].bit.CC;
        SerialUSB.println(cap1[nCap] / 12);
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    }
}

void TC4_Handler()
{
    if (TC4->COUNT16.INTFLAG.bit.MC0) {
        SerialUSB.println("TC4_handler");
        // The interrupt flag is cleared by reading CC
        cap0[nCap] = TC4->COUNT16.CC[0].bit.CC;
        cap1[nCap] = TC4->COUNT16.CC[1].bit.CC;
        SerialUSB.println(cap1[nCap] / 12);
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    }
}

void TC5_Handler()
{
    if (TC5->COUNT16.INTFLAG.bit.MC0) {
        SerialUSB.println("TC5_handler");
        // The interrupt flag is cleared by reading CC
        cap0[nCap] = TC5->COUNT16.CC[0].bit.CC;
        cap1[nCap] = TC5->COUNT16.CC[1].bit.CC;
        SerialUSB.println(cap1[nCap] / 12);
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    }
}

void TCC0_Handler()
{
    if (TCC0->INTFLAG.bit.MC0) {
        SerialUSB.println("TCC0_handler");
        // The interrupt flag is cleared by reading CC
        cap0[nCap] = TCC0->CC[0].bit.CC;
        cap1[nCap] = TCC0->CC[1].bit.CC;
        SerialUSB.println(cap1[nCap] / 12);
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    }
}

void TCC1_Handler()
{
    if (TCC1->INTFLAG.bit.MC0) {
        SerialUSB.println("TCC1_handler");
        // The interrupt flag is cleared by reading CC
        cap0[nCap] = TCC1->CC[0].bit.CC;
        cap1[nCap] = TCC1->CC[1].bit.CC;
        SerialUSB.println(cap1[nCap] / 12);
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    }
}

void TCC2_Handler()
{
    if (TCC2->INTFLAG.bit.MC0) {
        SerialUSB.println("TCC2_handler");
        // The interrupt flag is cleared by reading CC
        cap0[nCap] = TCC2->CC[0].bit.CC;
        cap1[nCap] = TCC2->CC[1].bit.CC;
        SerialUSB.println(cap1[nCap] / 12);
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    }
}




void setup(void)
{
    //SystemInit();
    connectPortPinsToInterrupt(PORT_PA04, 4);
    setupEIC(4);
    setupTimer(TC3, GCLK_CLKCTRL_ID_TCC2_TC3, EVSYS_ID_GEN_EIC_EXTINT_4, EVSYS_ID_USER_TC3_EVU, 1, PM_APBCMASK_TC3, TC3_IRQn, true, 4);

    connectPortPinsToInterrupt(PORT_PA10, 10);
    setupEIC(10);
    setupTimer(TC4, GCLK_CLKCTRL_ID_TC4_TC5, EVSYS_ID_GEN_EIC_EXTINT_10, EVSYS_ID_USER_TC4_EVU, 3, PM_APBCMASK_TC4, TC4_IRQn, true, 4);
   /* connectPortPinsToInterruptT4();
    setupEICT4();
    setupTimerT4();*/
  //  connectPortPinsToInterrupt(PORT_PA02, 2);
    //setupEIC(2);
    //setupTimer(TC5, GCLK_CLKCTRL_ID_TC4_TC5, EVSYS_ID_GEN_EIC_EXTINT_2, EVSYS_ID_USER_TC5_EVU, 5, PM_APBCMASK_TC5, TC5_IRQn);


   // connectPortPinsToInterruptT5();
    //setupEICT5();
    //setupTimerT5();

    //connectPortPinsToInterruptTCC0();
    //setupEICTCC0();
    /*
    setupClock();
    connectPortPinsToInterrupt(PORT_PA02, 11);
    setupEIC(11);
    setupTimer(TCC0, GCLK_CLKCTRL_ID_TCC0_TCC1, EVSYS_ID_GEN_EIC_EXTINT_11, EVSYS_ID_USER_TCC0_EV_1, 7, PM_APBCMASK_TCC0, TCC0_IRQn, true, 4);

    setupClock();
    connectPortPinsToInterrupt(PORT_PA02, 2);
    setupEIC(2);
    setupTimer(TCC1, GCLK_CLKCTRL_ID_TCC0_TCC1, EVSYS_ID_GEN_EIC_EXTINT_2, EVSYS_ID_USER_TCC1_EV_1, 5, PM_APBCMASK_TCC1, TCC1_IRQn, true, 4);

}

void loop()
{
}*/
