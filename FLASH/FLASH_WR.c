#include "n32g45x_flash.h"
#include "FLASH_WR.h"
/**************************************************************************
�������ܣ���ָ����ַд������
��ڲ�����addr 	д���FLASHҳ���׵�ַ
                    p	  	��д������ĵ�ַ�������еı�����uint8_t���ͣ�Ԫ�ظ���������ż����
                    Count_To_Write ��д������ĵ�ַ��
�� �� ֵ��pdu�ɹ�д�����
**************************************************************************/
int MyFLASH_WriteWord(uint32_t faddr, uint16_t* p, uint16_t Count_To_Write)
{
    //д������
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
�������ܣ���ָ����ַ��ȡ����
��ڲ�����addr ��FLASH�ж�ȡ�ĵ�ַ
                    p    ��ȡ��Ҫ��������ĵ�ַ�������еı�����uint8_t���ͣ�
                    Count_To_Write Ҫ�������ֽ���
�� �� ֵ����
**************************************************************************/
void MyFLASH_ReadByte(unsigned int addr, uint16_t* p, uint16_t Count_To_Read)
{
    //memcpy(p, (uint16_t*)addr, Count_To_Read);
    uint32_t d;
		uint32_t id=0;
		int n = Count_To_Read>>1;		
    for (uint16_t i=0; i<n; i++)
    {
			d = *((__IO uint32_t *)(addr));//__IO�궨��volatile��CM3��ַ��32λ�ģ�����ǿ��ת������ȡֵ
			p[id++]=d;
			p[id++]=d>>16;
			addr+=4;
    }
}
