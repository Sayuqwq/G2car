#ifndef _EVENTLOOP_H
#define _EVENTLOOP_H

#include "stm32f10x.h"
#include "alias.h"


void EVENTLOOP_Init(void);

typedef struct 
{
    uint16_t eventloop1SecMonitor;
    const uint16_t eventloop1SecMonitorPeriod;

} EVENTLOOPVARS;


#endif





























