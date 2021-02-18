#include <stdlib.h>
#include "Arduino.h"
#include "samd.h"

bool BusyForChannel(const uint8_t &channel);
bool USRRDYForChannel(const uint8_t &channel);
void connectPortPinsToInterrupt(uint16_t port_pa, uint16_t ext_int);
void setupEIC(uint16_t ext_int);
uint16_t TC_CTRLA_PRESCALER_DIV_Val(uint16_t div);
void setupTimer(Tc *tc, uint16_t clkctrl_tc, uint16_t ext_int, uint16_t EVSYS_ID_USER, uint16_t channel, uint32_t tc_enable_bit, IRQn_Type tc_irqn, bool invert_input, uint16_t prescaler_div);
void setupClock(uint16_t gclk_id);
uint16_t TCC_CTRLA_PRESCALER_DIV(uint16_t div);
void setupTimer(Tcc *tc, uint16_t clkctrl_tc, uint16_t ext_int, uint16_t channel, uint32_t tc_enable_bit, IRQn_Type tc_irqn, bool invert_input, uint16_t prescaler_div, uint16_t rising_event_out, uint16_t falling_event_out);
