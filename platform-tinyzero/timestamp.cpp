#include "primitives/timestamp.h"
#include <Arduino.h>

#define SYST_CVR (*(volatile uint32_t *)0xE000E018) // SysTick Current Value Register
#define SCB_ICSR (*(volatile uint32_t *)0xE000ED04) // Interrupt Control and State
#define SCB_ICSR_PENDSTSET ((uint32_t)0x04000000)  // SysTick is pending

#define F_CPU  480000000

Timestamp Timestamp::cur_time() {
    // Repeat logic inside micros(), but get better precision, up to a single CPU tick.
    extern volatile uint32_t systick_millis_count;
	__disable_irq();
	uint32_t cpu_ticks = SYST_CVR;
	//uint32_t millis = systick_millis_count;
	uint32_t millis = cur_time_millis();
	uint32_t istatus = SCB_ICSR;  // bit 26 indicates if systick exception pending
	__enable_irq();

	if ((istatus & SCB_ICSR_PENDSTSET) && cpu_ticks > 50) millis++;
	cpu_ticks = ((F_CPU / 1000) - 1) - cpu_ticks;

    static_assert(F_CPU % sec == 0, "Please choose TimeUnit.usec to be a multiple of CPU cycles to keep timing precise");
	return millis * msec + cpu_ticks / (F_CPU / sec);
};

uint32_t Timestamp::cur_time_millis() {
    return millis();
}

