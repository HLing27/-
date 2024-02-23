/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ZLG7290_H

#define __ZLG7290_H

#include "stm32f4xx_hal.h"
//����ZLG7290�ڲ��Ĵ�����ַ���ӵ�ַ��
#define ZLG7290_SystemReg		0x00		//ϵͳ�Ĵ���
#define ZLG7290_Key			0x01		//��ֵ�Ĵ���
#define ZLG7290_RepeatCnt		0x02		//���������Ĵ���
#define ZLG7290_FunctionKey		0x03		//���ܼ��Ĵ���
#define ZLG7290_CmdBuf			0x07		//���������ʼ��ַ
#define ZLG7290_CmdBuf0			0x07		//�������0
#define ZLG7290_CmdBuf1			0x08		//�������1
#define ZLG7290_FlashOnOff		0x0C		//��˸���ƼĴ���
#define ZLG7290_ScanNum			0x0D		//ɨ��λ���Ĵ���

#define ZLG7290_DpRam			0x10		//��ʾ������ʼ��ַ

#define ZLG7290_DpRam0			0x10		//��ʾ����0
#define ZLG7290_DpRam1			0x11		//��ʾ����1
#define ZLG7290_DpRam2			0x12		//��ʾ����2
#define ZLG7290_DpRam3			0x13		//��ʾ����3
#define ZLG7290_DpRam4			0x14		//��ʾ����4
#define ZLG7290_DpRam5			0x15		//��ʾ����5
#define ZLG7290_DpRam6			0x16		//��ʾ����6
#define ZLG7290_DpRam7			0x17		//��ʾ����7
#define ADDR_24LC64     0x70

#define I2C_PAGESIZE    8

void I2C_ZLG7290_Read(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint8_t num);
void I2C_ZLG7290_Write(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint8_t num);


#endif /* __24C64_OPT_H */

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
