
#define __NOP __nop
#include "msp.h"
#include "main.hpp"
#include "Scheduler.hpp"
#include "Task.hpp"
#include "LED.hpp"
#include "HAL_OPT3001.hpp"

uint16_t ADC14Result = 0U;


uint8_t Task::m_u8NextTaskID = 0;
volatile static uint64_t SystemTicks = 0;

void main(void)
{
    Scheduler MainScheduler;
    LED BlinkLED;
    Setup();
    MainScheduler.attach(&BlinkLED);
    while(1){
    	__wfe();
        if(SystemTicks != MainScheduler.ticks)
        {
            MainScheduler.ticks = SystemTicks;
            MainScheduler.run();
        }
    };
}

// **********************************
// Setup function for the application
// @input - none
// @output - none
// **********************************
void Setup(void)
{
	// ****************************
	//         DEVICE CONFIG
	// ****************************
	// - Disable WDT
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;


	// ****************************
	//         PORT CONFIG
	// ****************************
	// - P1.0 is connected to the Red LED
	P1->DIR |= BIT0;
	J4->DIR |= BIT37;

	// ****************************
	//       TIMER CONFIG
	// ****************************
	// - Disable all interrupts
	// - Configure Timer32_1  with MCLK (3Mhz), Division by 1, Enable the interrupt, Periodic Mode
	// - Enable the interrupt in the NVIC
	// - Start the timer in UP mode.
	// - Re-enable interrupts
	__disable_irq();
	TIMER32_1->LOAD = 0x002DC6C0; //~1s ---> a 3Mhz
	//TIMER32_1->LOAD = 0x00000BB8; //~1ms ---> a 3Mhz
	TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
	NVIC_SetPriority(T32_INT1_IRQn,1);
	NVIC_EnableIRQ(T32_INT1_IRQn);
	__enable_irq();
	return;
}

extern "C"
{
	void T32_INT1_IRQHandler(void)
	{
		TIMER32_1->INTCLR = 0U;
		P1->OUT ^= BIT0;
		J4->OUT ^= BIT37;
		SystemTicks++;
		return;
	}
}
