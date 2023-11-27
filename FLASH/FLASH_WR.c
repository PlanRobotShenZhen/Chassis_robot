#include "n32g45x.h"                    // Device header

#define FINAL_PAGE_ADDRESS 0x0807F800
#define STORE_COUNT 1024

uint16_t Store_Data[STORE_COUNT];

// ��ȡFLASH
uint16_t MyFLASH_ReadHalfWord(uint32_t Address)
{
    return *((__IO uint16_t *)(Address));
}

// ����FLASH
void MyFLASH_ErasePage(uint32_t Address)
{
    FLASH_Unlock();
    FLASH_EraseOnePage(Address);
    FLASH_Lock();
}

// д��FLASH
void MyFLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
    FLASH_Unlock();
    FLASH_ProgramWord(Address, Data);
    FLASH_Lock();
}

// ��ʼ������
void Store_Init(void)
{
    if (MyFLASH_ReadHalfWord(FINAL_PAGE_ADDRESS) != 0xA5A5)
    {
        MyFLASH_ErasePage(FINAL_PAGE_ADDRESS);
        MyFLASH_ProgramWord(FINAL_PAGE_ADDRESS, 0xA5A5);
        for (uint32_t i=1; i<STORE_COUNT; i++)
        {
            MyFLASH_ProgramWord(FINAL_PAGE_ADDRESS+i, 0x0000);
        }
    }
    
    for (uint32_t i=0; i<STORE_COUNT; i++)
    {
        Store_Data[i] = MyFLASH_ReadHalfWord(FINAL_PAGE_ADDRESS+i);
    }
}

// д������
void Store_Save(void)
{
    MyFLASH_ErasePage(FINAL_PAGE_ADDRESS);
    for (uint32_t i=0; i<STORE_COUNT; i++)
    {
        MyFLASH_ProgramWord(FINAL_PAGE_ADDRESS+i, Store_Data[i]);
    }
}

// �������
void Store_Clear(void)
{
    MyFLASH_ErasePage(FINAL_PAGE_ADDRESS);
    for (uint32_t i=1; i<STORE_COUNT; i++)
    {
        Store_Data[i] = 0x0000;
    }
    Store_Save();
}
