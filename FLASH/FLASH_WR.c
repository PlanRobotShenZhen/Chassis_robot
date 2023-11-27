#include "n32g45x.h"                    // Device header
#include "FLASH_WR.h"

uint16_t Store_Data[STORE_COUNT];

// 读取FLASH
uint16_t MyFLASH_ReadWord(uint32_t Address)
{
    return *((__IO uint16_t *)(Address));
}

// 擦除FLASH
void MyFLASH_ErasePage(uint32_t Address)
{
    FLASH_Unlock();
    FLASH_EraseOnePage(Address);
    FLASH_Lock();
}

// 写入FLASH
void MyFLASH_ProgramWord(uint32_t Address, uint16_t Data)
{
    FLASH_Unlock();
    FLASH_ProgramWord(Address, Data);
    FLASH_Lock();
}

// 初始化数据
void Store_Init(void)
{
    if (MyFLASH_ReadWord(FINAL_PAGE_ADDRESS) != 0xA5A5)
    {
        MyFLASH_ErasePage(FINAL_PAGE_ADDRESS);
        MyFLASH_ProgramWord(FINAL_PAGE_ADDRESS, 0xA5A5);
        for (uint16_t i=1; i<STORE_COUNT; i++)
        {
            MyFLASH_ProgramWord(FINAL_PAGE_ADDRESS+i, 0x0000);
        }
    }
    
    for (uint16_t i=0; i<STORE_COUNT; i++)
    {
        Store_Data[i] = MyFLASH_ReadWord(FINAL_PAGE_ADDRESS+i);
    }
}

// 写入数据
void Store_Save(void)
{
    MyFLASH_ErasePage(FINAL_PAGE_ADDRESS);
    for (uint16_t i=0; i<STORE_COUNT; i++)
    {
        MyFLASH_ProgramWord(FINAL_PAGE_ADDRESS+i, Store_Data[i]);
    }
}

// 清除数据
void Store_Clear(void)
{
    MyFLASH_ErasePage(FINAL_PAGE_ADDRESS);
    for (uint16_t i=1; i<STORE_COUNT; i++)
    {
        Store_Data[i] = 0x0000;
    }
    Store_Save();
}
