#include "Scheduler.hpp"

Scheduler::Scheduler()
{
    mOpenSlots = static_cast<uint8_t>(NUMBER_OF_SLOTS);
    mNextSlot = 0;
    for(uint8_t index = 0; index < (NUMBER_OF_SLOTS-1U); index++)
    {
        Schedule[index] = (uintptr_t) 0;
    }
    return;
}

uint8_t Scheduler::attach(Task * i_ToAttach)
{
    uint8_t l_ErrorCode = NO_ERR;
    if((mOpenSlots>0) && (mNextSlot < NUMBER_OF_SLOTS))
    {
        Schedule[mNextSlot] = i_ToAttach;
        mOpenSlots--;
        mNextSlot++;
    }
    else
    {
        l_ErrorCode = RET_ERR;
    }
    return l_ErrorCode;
}

uint8_t Scheduler::attach(Task * i_ToAttach, uint64_t i_u64TickInterval)
{
    uint8_t l_ErrorCode = NO_ERR;
    if((mOpenSlots>0) && (mNextSlot < NUMBER_OF_SLOTS))
    {
        Schedule[mNextSlot] = i_ToAttach;
        mOpenSlots--;
        mNextSlot++;
    }
    else
    {
        l_ErrorCode = RET_ERR;
    }
    return l_ErrorCode;
}

uint8_t Scheduler::run(void)
{
    uint8_t NextTaskSlot = 0U;
    Task * NextTask = (uintptr_t) 0;
    uint8_t l_u8ReturnCode = NO_ERR;

    while(NextTaskSlot < (NUMBER_OF_SLOTS-1U))
    {
        NextTask = static_cast<Task *> (Schedule[NextTaskSlot]);
        if(NextTask != ((uintptr_t) 0))
        {
            NextTask->run();
            NextTaskSlot++;
        }
        else
        {
            NextTaskSlot++;
        }
    }
    return l_u8ReturnCode;
}

uint8_t Scheduler::CalculateNextSchedule(void)
{
    return(NO_ERR);
}
uint8_t Scheduler::SortScheduleByPriority(Task * i_pSchedule)
{
    return(NO_ERR);
}
