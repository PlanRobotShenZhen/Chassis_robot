#include "n32g45x_flash.h"
#include "FLASH_WR.h"
/**************************************************************************
函数功能：向指定地址写入数据
入口参数：addr 	写入的FLASH页的首地址
                    p	  	被写入变量的地址（数组中的必须是uint8_t类型，元素个数必须是偶数）
                    Count_To_Write 被写入变量的地址数
返 回 值：pdu成功写入个数
**************************************************************************/
int MyFLASH_WriteWord(uint32_t faddr, uint16_t* p, uint16_t Count_To_Write)
{
    //写入数据
    uint32_t d;
	uint16_t dataIndex;
    __disable_irq();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BUSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    FLASH_EraseOnePage(faddr);
    for (dataIndex = 0;dataIndex < Count_To_Write;dataIndex++)
    {
        d = p[dataIndex];
		dataIndex++;
        d |= p[dataIndex] << 16;
        FLASH_ProgramWord(faddr , d);
        faddr += 4;
    }
    FLASH_Lock();
    __enable_irq();
    return dataIndex;
}


/**************************************************************************
函数功能：从指定地址读取数据
入口参数：addr 从FLASH中读取的地址
                    p    读取后要存入变量的地址（数组中的必须是uint8_t类型）
                    Count_To_Write 要读出的字节数
返 回 值：无
**************************************************************************/
void MyFLASH_ReadByte(unsigned int addr, uint16_t* p, uint16_t Count_To_Read)
{
    //memcpy(p, (uint16_t*)addr, Count_To_Read);
    uint32_t d;
		uint32_t id=0;
		int n = Count_To_Read>>1;		
    for (uint16_t i=0; i<n; i++)
    {
			d = *((__IO uint32_t *)(addr));//__IO宏定义volatile，CM3地址是32位的，所以强制转换后再取值
			p[id++]=d;
			p[id++]=d>>16;
			addr+=4;
    }
}
