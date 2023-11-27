#ifndef __FLASH_WR_H
#define __FLASH_WR_H

#define FINAL_PAGE_ADDRESS 0x0807F800
#define STORE_COUNT 1024

uint16_t MyFLASH_ReadWord(uint32_t Address);
void MyFLASH_ErasePage(uint32_t Address);
void MyFLASH_ProgramWord(uint32_t Address, uint16_t Data);

extern uint16_t Store_Data[];
void Store_Init(void);
void Store_Save(void);
void Store_Clear(void);
#endif
