/*
 * Scheduler.hpp
 *
 *  Created on: Aug 31, 2016
 *      Author: eortiz
 */

#ifndef TASKS_SCHEDULER_HPP_
#define TASKS_SCHEDULER_HPP_
#define __NOP __nop
#include "msp.h"
#include "Task.hpp"

#define NUMBER_OF_SLOTS 256
#define NULL            0

class Scheduler
{
public:
    Scheduler();
    uint64_t ticks;
    uint8_t attach(Task * i_ToAttach);
    uint8_t attach(Task * i_ToAttach, uint64_t i_u64TickInterval);
    uint8_t run(void);
private:
    uint8_t mOpenSlots;
    uint8_t mNextSlot;
    Task * Schedule[NUMBER_OF_SLOTS];
    Task * NextSchedule[NUMBER_OF_SLOTS];
    uint8_t CalculateNextSchedule(void);
    uint8_t SortScheduleByPriority(Task * i_pSchedule);
};



#endif /* TASKS_SCHEDULER_HPP_ */
