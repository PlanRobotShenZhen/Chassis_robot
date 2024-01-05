#ifndef __RCCAR_H__
#define __RCCAR_H__

#include "n32g45x.h"


void RCCAR_Init(uint16_t* p);
void RCCAR_Process(uint16_t ch1, uint16_t ch2);

#endif