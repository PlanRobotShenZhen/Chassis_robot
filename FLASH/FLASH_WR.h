#ifndef __FLASH_WR_H
#define __FLASH_WR_H

uint16_t MyFLASH_ReadHalfWord(uint32_t Address);
void MyFLASH_ErasePage(uint32_t Address);
void MyFLASH_ProgramWord(uint32_t Address, uint32_t Data);

extern uint16_t Store_Data[];
void Store_Init(void);
void Store_Save(void);
void Store_Clear(void);
#endif
