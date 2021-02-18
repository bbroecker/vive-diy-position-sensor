#include "samd21_timer.h"
#include "Arduino.h"

static volatile uint16_t cap0[128]; // Period with PPW capture
static volatile uint16_t cap1[128]; // Pulse width with PPW capture
static volatile size_t nCap;
//s(10);
void TC3_Handler()
{
    //if (TC3->COUNT16.INTFLAG.bit.MC1) {
        SerialUSB.println("TC3_handler");
        // The interrupt flag is cleared by reading CC
        uint16_t start = TC3->COUNT16.CC[0].bit.CC;
        uint16_t len_pulse  = TC3->COUNT16.CC[1].bit.CC;
        //uint16_t start = REG_TC3_CC0;
        //uint16_t end = REG_TC3_CC1;

        SerialUSB.print(nCap);
        SerialUSB.print(" ");
        SerialUSB.print(start / 12);
        SerialUSB.print(" ");
        SerialUSB.print(len_pulse / 12);
        SerialUSB.print("\n");
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    //}
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
        SerialUSB.println("TCC0_handler");
    if (TCC0->INTFLAG.bit.MC0 || TCC0->INTFLAG.bit.MC1) {
        // The interrupt flag is cleared by reading CC
        cap0[nCap] = TCC0->CC[0].bit.CC;
        cap1[nCap] = TCC0->CC[1].bit.CC;
        SerialUSB.print("mc0: ");
        SerialUSB.print((cap1[nCap] - cap0[nCap])/ 12);
        SerialUSB.print("\n");
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    }
    if (TCC0->INTFLAG.bit.MC2 || TCC0->INTFLAG.bit.MC3) {
        // The interrupt flag is cleared by reading CC
        cap0[nCap] = TCC0->CC[2].bit.CC;
        cap1[nCap] = TCC0->CC[3].bit.CC;
        SerialUSB.print("mc2: ");
        SerialUSB.print((cap1[nCap] - cap0[nCap])/ 12);
        SerialUSB.print("\n");
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    }
}

void TCC1_Handler()
{
   // if (TCC1->INTFLAG.bit.MC0) {
        SerialUSB.println("TCC1_handler");
        // The interrupt flag is cleared by reading CC
        cap0[nCap] = TCC1->CC[0].bit.CC;
        cap1[nCap] = TCC1->CC[1].bit.CC;
        SerialUSB.println((cap1[nCap] - cap0[nCap])/ 12);
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    //}
}

void TCC2_Handler()
{
    //if (TCC2->INTFLAG.bit.MC0) {
        SerialUSB.println("TCC2_handler");
        // The interrupt flag is cleared by reading CC
        cap0[nCap] = TCC2->CC[0].bit.CC;
        cap1[nCap] = TCC2->CC[1].bit.CC;
        //SerialUSB.println(cap1[nCap] / 12);
        SerialUSB.println((cap1[nCap] - cap0[nCap])/ 12);
        if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
            static volatile size_t done;
            done++;
            nCap = 0;
        }
    //}
}




void setup(void)
{
    //SystemInit();
    //

    bool invert = true;
   // setupClock(3);
    uint16_t prescalar = 4;
    /*connectPortPinsToInterrupt(PORT_PA04, 4);
    setupEIC(4);
    setupTimer(TC3, GCLK_CLKCTRL_ID_TCC2_TC3, EVSYS_ID_GEN_EIC_EXTINT_4, EVSYS_ID_USER_TC3_EVU, 1, PM_APBCMASK_TC3, TC3_IRQn, invert, prescalar);

    setupClock(3);
    connectPortPinsToInterrupt(PORT_PA10, 10);
    setupEIC(10);
    setupTimer(TC4, GCLK_CLKCTRL_ID_TC4_TC5, EVSYS_ID_GEN_EIC_EXTINT_10, EVSYS_ID_USER_TC4_EVU, 3, PM_APBCMASK_TC4, TC4_IRQn, invert, prescalar);
   */
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
    setupClock(3);
    connectPortPinsToInterrupt(PORT_PA11, 11);
    setupEIC(11);
    setupTimer(TCC0, GCLK_CLKCTRL_ID_TCC0_TCC1, EVSYS_ID_GEN_EIC_EXTINT_11, 5, PM_APBCMASK_TCC0, TCC0_IRQn, invert, prescalar, EVSYS_ID_USER_TCC0_MC_2, EVSYS_ID_USER_TCC0_MC_3);

    setupClock(3);
    connectPortPinsToInterrupt(PORT_PA04, 4);
    setupEIC(4);
    setupTimer(TCC0, GCLK_CLKCTRL_ID_TCC0_TCC1, EVSYS_ID_GEN_EIC_EXTINT_4, 7, PM_APBCMASK_TCC0, TCC0_IRQn, invert, prescalar, EVSYS_ID_USER_TCC0_MC_0, EVSYS_ID_USER_TCC0_MC_1);

    setupClock(3);
    connectPortPinsToInterrupt(PORT_PA10, 10);
    setupEIC(10);
    setupTimer(TCC1, GCLK_CLKCTRL_ID_TCC0_TCC1, EVSYS_ID_GEN_EIC_EXTINT_10, 1, PM_APBCMASK_TCC1, TCC1_IRQn, invert, prescalar, EVSYS_ID_USER_TCC1_MC_0, EVSYS_ID_USER_TCC1_MC_1);

    setupClock(3);
    connectPortPinsToInterrupt(PORT_PA02, 2);
    setupEIC(2);
    setupTimer(TCC2, GCLK_CLKCTRL_ID_TCC2_TC3, EVSYS_ID_GEN_EIC_EXTINT_2, 3, PM_APBCMASK_TCC2, TCC2_IRQn, invert, prescalar, EVSYS_ID_USER_TCC2_MC_0, EVSYS_ID_USER_TCC2_MC_1);


    /*setupClock(3);
    connectPortPinsToInterrupt(PORT_PA02, 2);
    setupEIC(2);
    setupTimer(TCC1, GCLK_CLKCTRL_ID_TCC0_TCC1, EVSYS_ID_GEN_EIC_EXTINT_2, EVSYS_ID_USER_TCC1_EV_1, 5, PM_APBCMASK_TCC1, TCC1_IRQn, invert, prescalar);*/

}

void loop()
{
}
