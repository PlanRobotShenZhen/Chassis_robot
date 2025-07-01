#ifndef __FLASH_WR_H
#define __FLASH_WR_H


#define FINAL_PAGE_ADDRESS 0x0803F800 //CCL7的Flash大小是256kb，擦除flash内存的最小单位是扇区，该扇区为2kb。取最后2kb作为数据储存区，而且Flash地址是从0x8000000开始的
/**************************************************************************
函数功能：向指定地址写入数据
入口参数：addr 	写入的FLASH页的首地址
                    p	  	被写入变量的地址（数组中的必须是uint8_t类型，元素个数必须是偶数）
                    Count_To_Write 被写入变量的地址数
返 回 值：成功写入个数
**************************************************************************/
int MyFLASH_WriteWord(uint32_t faddr, uint16_t* p, uint16_t Count_To_Write);

/**************************************************************************
函数功能：从指定地址读取数据
入口参数：addr 从FLASH中读取的地址
                    p    读取后要存入变量的地址（数组中的必须是uint8_t类型）
                    Count_To_Write 要读出的字节数
返 回 值：无
**************************************************************************/
void MyFLASH_ReadByte(unsigned int addr, uint16_t* p, uint16_t Count_To_Read);
#endif
