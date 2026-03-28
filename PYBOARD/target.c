/*
 * iNAV target for MicroPython pyboard v1.1 (STM32F405RGT6).
 *
 * Timer / DMA assignments for motor outputs:
 *
 *   Output  Pin   Timer      DMA stream
 *   M1      X1    TIM5_CH1   DMA1_Stream2_CH6
 *   M2      X2    TIM5_CH2   DMA1_Stream4_CH6
 *   M3      X3    TIM5_CH3   DMA1_Stream0_CH6
 *   M4      X4    TIM5_CH4   DMA1_Stream1_CH6
 *
 * All four channels share TIM5 — no split-timer DShot issues.
 * DMA streams 0/1/2/4 on DMA1 are unique to these channels.
 */

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/bus.h"

timerHardware_t timerHardware[] = {
    //            TIM    CH   Pin   Usage                    flags  dmaopt
    DEF_TIM(TIM5, CH1,  PA0,  TIM_USE_OUTPUT_AUTO,          0,     0),  // M1 X1
    DEF_TIM(TIM5, CH2,  PA1,  TIM_USE_OUTPUT_AUTO,          0,     0),  // M2 X2
    DEF_TIM(TIM5, CH3,  PA2,  TIM_USE_OUTPUT_AUTO,          0,     0),  // M3 X3
    DEF_TIM(TIM5, CH4,  PA3,  TIM_USE_OUTPUT_AUTO,          0,     0),  // M4 X4
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
