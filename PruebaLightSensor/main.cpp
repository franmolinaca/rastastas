/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//****************************************************************************
//          Displays lux value measured by the OPT3001 Digital Ambient
//          Light Sensor on the colored LCD. The MSP432 communicates
//          with the sensor through I2C.
//
//          The ambient light measurement is also used to automatically
//          adjust the LCD backlight.
//
//      *** Make sure J5 jumper on the BOOSTXL-EDUMKII is connected ***
//          to 3.LCD BACKLT
//
//****************************************************************************
extern "C" {
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include "LcdDriver/HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.h"
#include <ti/grlib/grlib.h>
#include "HAL_I2C.h"
#include "HAL_OPT3001.h"
}

#include <stdio.h>

uint16_t ADC14Result = 0U;


volatile unsigned int S1buttonDebounce = 0; // Deboounce state for button S1
volatile unsigned int S2buttonDebounce = 0; // Deboounce state for button S1

volatile int32_t interval = 0;       // Elapsed interval between taps
volatile int32_t periods[4];         // LED blink periods for each color state
volatile int oldTick = 0;            // SysTick value of previous tap
volatile int newTick = 0;            // SysTick value of current tap

volatile int counting = 0;           // Whether currently counting taps and interval between taps
volatile int count = 0;              // Counts # of seconds elapsed since last tap
volatile int sysTickCount = 0;       // Counts # of SysTick interrupts since last tap
volatile int taps = 0;               // Counts # of taps from last reset


/* Graphic library context */
Graphics_Context g_sContext;

/* Variable for storing lux value returned from OPT3001 */
float lux;
int lighten=0;
char state='i';

/* Current color of the blinking RGB LED
 * 4 possible states: R, G, B, random color */
volatile unsigned int currentColor = 0;


/* Timer_A Up Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,               // ACLK Clock SOurce
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // ACLK/1 = 3MHz
        200,                                    // 200 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

/* TimerA UpMode Configuration Parameter */
const Timer_A_UpModeConfig TA1upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 3MHz
        45000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};


const Timer_A_UpModeConfig TA2upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_64,         // SMCLK/64 ~ 46.9 kMHz
        46875,                                  // 1s timer period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};


/* Timer_A Compare Configuration Parameter  (PWM) */
Timer_A_CompareModeConfig compareConfig_PWM =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_3,          // Use CCR3
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_SET,              // Toggle output but
        100                                         // 50% Duty Cycle
};









/*
 * Main function
 */
int main(void)
{
    /* Halting WDT and disabling master interrupts */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();

    /* Set the core voltage level to VCORE1 */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set 2 flash wait states for Flash bank 0 and 1*/
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(0);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
    /*Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)"Light Sensor:",
                                    AUTO_STRING_LENGTH,
                                    64,
                                    30,
                                    OPAQUE_TEXT);
*/
    /* Selecting P1.2 and P1.3 in UART mode and P1.0 as output (LED) */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configures P2.6 to PM_TA0.3 for using Timer PWM to control LCD backlight */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6 , GPIO_PRIMARY_MODULE_FUNCTION);

    /* Confinguring P1.1 & P1.4 as an input and enabling interrupts */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);


    /* Configuring Timer_A0 for Up Mode and starting */
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
    /* Configuring TimerA1 and TimerA2 for Up Mode  using Driverlib*/
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &TA1upConfig);
    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &TA2upConfig);

    /* Initialize compare registers to generate PWM */
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);

    /* Initialize I2C communication */
    Init_I2C_GPIO();
    I2C_init();

    // ****************************
    //         PORT CONFIG
    // ****************************
    // - P1.0 is connected to the Red LED
    P1->DIR |= BIT0;
    P2->DIR |= BIT0;
    P2->DIR |= BIT1;
    P2->DIR |= BIT2;


    /* Initialize OPT3001 digital ambient light sensor */
    OPT3001_init();

    __delay_cycles(100000);







    // Set P4.3 for Analog input, disabling the I/O circuit.
    P4->SEL0 = BIT3;
    P4->SEL1 = BIT3;
    P4->DIR &= ~BIT3;

    //TIMER32_1->LOAD = 0x00B71B00; //~0.5s ---> a 48Mhz
    TIMER32_1->LOAD = 0x0000BB80; //~0.5s ---> a 48Mhz
    TIMER32_1->CONTROL = TIMER32_CONTROL_SIZE | TIMER32_CONTROL_PRESCALE_0 | TIMER32_CONTROL_MODE | TIMER32_CONTROL_IE | TIMER32_CONTROL_ENABLE;
    NVIC_SetPriority(T32_INT1_IRQn,1);
    NVIC_EnableIRQ(T32_INT1_IRQn);

    ADC14->CTL0 = ADC14_CTL0_PDIV_0 | ADC14_CTL0_SHS_0 | ADC14_CTL0_DIV_7 |
                  ADC14_CTL0_SSEL__MCLK | ADC14_CTL0_SHT0_1 | ADC14_CTL0_ON
                  | ADC14_CTL0_SHP;
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_10 | ADC14_MCTLN_VRSEL_0;
    ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_ENC;
    ADC14->IER0 = ADC14_IER0_IE0;
    NVIC_SetPriority(ADC14_IRQn,1);
    NVIC_EnableIRQ(ADC14_IRQn);









    while(1)
    {



        MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
        MAP_Interrupt_enableInterrupt(INT_PORT1);
        MAP_Interrupt_enableInterrupt(INT_TA1_0);
        MAP_Interrupt_enableInterrupt(INT_TA2_0);
        MAP_Interrupt_enableMaster();

        /* Obtain lux value from OPT3001 */
        lux = OPT3001_getLux();

        switch(state) {
        case 'i':
            Graphics_drawStringCentered(&g_sContext,
                                            (int8_t *)"Configure Light",
                                            AUTO_STRING_LENGTH,
                                            64,
                                            30,
                                            OPAQUE_TEXT);
          break;
        case 'f':
            Graphics_drawStringCentered(&g_sContext,
                                            (int8_t *)"ººEnlightenMeºº",
                                            AUTO_STRING_LENGTH,
                                            64,
                                            30,
                                            OPAQUE_TEXT);
            char string[20];
            sprintf(string, "%f", lux);
            Graphics_drawStringCentered(&g_sContext,
                                            (int8_t *)string,
                                            6,
                                            48,
                                            70,
                                            OPAQUE_TEXT);

            sprintf(string, "lux");
            Graphics_drawStringCentered(&g_sContext,
                                            (int8_t *)string,
                                            3,
                                            86,
                                            70,
                                            OPAQUE_TEXT);

          break;
        case 'a':

          break;
        case 'q':

        default:
            Graphics_drawStringCentered(&g_sContext,
                                            (int8_t *)"Unknown State",
                                            AUTO_STRING_LENGTH,
                                            64,
                                            30,
                                            OPAQUE_TEXT);

        }




        /* Adjust LCD Backlight
        if (lux < 20){
            if (lighten==0){
                P1->OUT ^= BIT0;
                lighten=1;
            }
        }
        else if (lux < 2000){
            lighten=0;
            compareConfig_PWM.compareValue = ((500*0.1) + (lux*2))/2000 * 200;
        }
        else
            compareConfig_PWM.compareValue = 200;
        */
        Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
    }
}


















extern "C"
{

/*
 * Port 1 interrupt handler. This handler is called whenever switches attached
 * to P1.1 (S1) and P1.4 (S2) are pressed.
 */
void PORT1_IRQHandler(void)
{
    newTick = MAP_SysTick_getValue();
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    /* Handles S1 button press */
    if (status & GPIO_PIN1)
    {
        if (S1buttonDebounce == 0)
        {
            S1buttonDebounce = 1;

            MAP_Interrupt_disableInterrupt(INT_PORT1);

            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);


            if (state=='i'){
                state='f';
                P1->OUT ^= BIT0;
                __delay_cycles(5000000);
                P1->OUT ^= BIT0;
                __delay_cycles(5000000);
                P1->OUT ^= BIT0;
                __delay_cycles(5000000);
                P1->OUT ^= BIT0;
                __delay_cycles(5000000);
                P1->OUT ^= BIT0;
                __delay_cycles(5000000);
                P1->OUT ^= BIT0;
            }
            else{
                switch(currentColor)
                {
                    case 0:  // Red
                        P2->OUT = BIT0;
                        break;
                    case 1:  // Green
                        P2->OUT ^= BIT1;
                        break;
                    case 2:  // Blue
                        P2->OUT ^= BIT2;
                        break;
                    default:
                        break;
                }
            }


            MAP_Timer_A_stopTimer(TIMER_A2_BASE);
            MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &TA2upConfig);
            MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

            count = 0;
            taps++;

            if (counting == 0)
            {
                oldTick = newTick;
                counting = 1;
            }
            else
            {
                if (sysTickCount == 0)
                {
                    interval = oldTick - newTick;
                    periods[currentColor] = interval/2;
                    oldTick = newTick;
                }
                else
                {
                    interval = (MAP_SysTick_getPeriod() - newTick) + ((sysTickCount-1) * MAP_SysTick_getPeriod()) + oldTick;
                    oldTick = newTick;
                    sysTickCount = 0;
                }
                periods[currentColor] = ((periods[currentColor] * (taps-2)) + interval/2) / (taps-1);
                MAP_SysTick_setPeriod(periods[currentColor]);
            }

            /* Start button debounce timer */
            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
    /* Handles S2 button press */
    if (status & GPIO_PIN4)
    {
        if (S2buttonDebounce == 0)
        {
            S2buttonDebounce = 1;

            MAP_Interrupt_disableInterrupt(INT_PORT1);

            if (state=='i'){
                /* Cycle through R, G, B, random color */
                if (currentColor < 2)
                    currentColor++;
                else
                    currentColor = 0;

                switch(currentColor)
                {
                    case 0:  // Red
                        P2->OUT = BIT2;
                        P2->OUT = BIT0;
                        break;
                    case 1:  // Green
                        P2->OUT ^= BIT0;
                        P2->OUT ^= BIT1;
                        break;
                    case 2:  // Blue
                        P2->OUT ^= BIT1;
                        P2->OUT ^= BIT2;
                        break;
                    default:
                        break;
                }
            }
            else{

            }

            /* Set current color's LED blink period */
            MAP_SysTick_setPeriod(periods[currentColor]);

            /* Start button debounce timer */
            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
}

















/*
 * Timer A1 interrupt handler. This handler determines whether to reset button
 * debounce after debounce timer expires.
 */
void TA1_0_IRQHandler(void)
{
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    if (P1IN & GPIO_PIN1)
    {
        S1buttonDebounce = 0;
    }
    if (P1IN & GPIO_PIN4)
    {
        S2buttonDebounce = 0;
    }

    if ((P1IN & GPIO_PIN1) && (P1IN & GPIO_PIN4))
    {
        MAP_Timer_A_stopTimer(TIMER_A1_BASE);
    }
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

/*
 * Timer A2 interrupt handler. This handler resets tapping state variables if button S1 is
 * not pressed for more than 4 seconds.
 */
void TA2_0_IRQHandler(void)
{
    if (counting == 1)
    {
        if (count < 4)
            count++;
        else
        {
            counting = 0;
            count = 0;
            sysTickCount = 0;
            taps = 0;
            MAP_Timer_A_stopTimer(TIMER_A2_BASE);
        }
    }
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_0);
}




void T32_INT1_IRQHandler(void)
{
    __disable_irq();
    TIMER32_1->INTCLR = 0U;
    P1->OUT ^= BIT0;
    ADC14->CTL0 = ADC14->CTL0 | ADC14_CTL0_SC; // Start
    __enable_irq();
    return;
}

void ADC14_IRQHandler(void)
{
    __disable_irq();
    ADC14Result = ADC14->MEM[0];
    ADC14->CLRIFGR0 = ADC14_CLRIFGR0_CLRIFG0;
    __enable_irq();
    return;
}
}
