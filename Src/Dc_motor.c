#include "Dc_motor.h"

#define I2C_Open_LONG_TIMEOUT         ((uint32_t)0xffff)

 __IO uint32_t  I2CTimeout1 = I2C_Open_LONG_TIMEOUT;


/*******************************************************************************
* Function Name  : I2C_WriteOneByte
* Description    : 
* Input          : 
* Output         : None
* Return         : 
* Attention      : None
*******************************************************************************/

void I2C_DC_Motor_WriteOneByte(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t value)
{   
	while( HAL_I2C_Mem_Write(I2Cx, I2C_Addr, addr, I2C_MEMADD_SIZE_8BIT, &value, 0x01, I2CTimeout1) != HAL_OK ){};
		
}

/*******************************************************************************
* Function Name  : I2C_Write
* Description    : 
* Input          : 
* Output         : None
* Return         : 
* Attention      : None
*******************************************************************************/

void I2C_DC_Motor_Write(I2C_HandleTypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint8_t num)
{
	while(num--)
	{
		
    I2C_DC_Motor_WriteOneByte(I2Cx, I2C_Addr,addr++,*buf++);
		HAL_Delay(5);
		
	}
}
/*******************************************************************************
* Function Name  : Dc_Motor
* Description    : 
* Input          : 
* Output         : None
* Return         : 
* Attention      : None
*******************************************************************************/
uint16_t speedcrc16(uint8_t *addr,uint8_t num)
{
	uint16_t i,j,temp;
	uint16_t crc=0xFFFF;	
	for(i=0;i<num;i++)
	{
		crc=crc^(*addr);
		for(j=0;j<8;j++)
		{
			temp=crc&0x0001;
			crc=crc>>1;
			if(temp)
			{
				crc=crc^0xA001;
			}
		}
		addr++;
	}
	return crc;
}

// 风扇转速备份
typedef struct dataBack
{
	/* data */
	uint8_t bk[3];
	uint16_t cksum;
}databk;

databk speedbk1;    // 备份1
databk speedbk2;    // 备份2
databk speed;       // 转速数据
uint8_t fs_flag = 0;
void DC_Task(uint8_t iKey)
{
	uint8_t Buffer_DC[1]={0Xff};
	uint8_t Buffer_DC_Zero[1]={0x00};
	speedbk1.bk[0] = 0x03;      // 备份1
	speedbk1.bk[1] = 0x04;
	speedbk1.bk[2] = 0x0f;
	speedbk1.cksum = speedcrc16(speedbk1.bk, 3);
	speedbk2.bk[0] = 0x03;      // 备份2
	speedbk2.bk[1] = 0x04;
	speedbk2.bk[2] = 0x0f;
	speedbk2.cksum = speedcrc16(speedbk2.bk, 3);
	speed.bk[0] = 0x03;         // 转速数据
	speed.bk[1] = 0x04;
	speed.bk[2] = 0x0f;
	speed.cksum = speedcrc16(speed.bk, 3);
	if (speedcrc16(speed.bk, 3) != speed.cksum) {       // 校验转速数据是否正确
		if (speedcrc16(speedbk1.bk, 3) == speedbk1.cksum) {
			for(int i = 0; i < 3; i++) {
				speed.bk[i] = speedbk1.bk[i];
			}
			speed.cksum = speedbk1.cksum;
		} else if (speedcrc16(speedbk2.bk, 3) == speedbk2.cksum) {
			for(int i = 0; i < 3; i++) {
				speed.bk[i] = speedbk2.bk[i];
			}
			speed.cksum = speedbk2.cksum;
			for(int i = 0; i < 3; i++) {
				speedbk1.bk[i] = speedbk2.bk[i];
			}
			speedbk1.cksum = speedbk2.cksum;	
		} else {
			printf("speed error!\n");
		}
	}
	switch(iKey)
	{
        case 0x1C:						 //1
					DC_Motor_Pin_Low();	
					fs_flag = 1;  //zheng
					I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,speed.bk[0],Buffer_DC_Zero,1);				
          break;
        case 0x1B:							//2
					DC_Motor_Pin_Low();	
					fs_flag = 1;  //zheng
				  I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,speed.bk[1],Buffer_DC,1);
          break;
        case 0x1A:							//3
					DC_Motor_Pin_Low();	
					fs_flag = 1;  //zheng
					I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,speed.bk[2],Buffer_DC,1);
          break;			
        /*case 0x14:					//4
				  DC_MOtor_Pin_High();
				  fs_flag = 0;  //zheng
					I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x0A,Buffer_DC,1);				
          break;   
				case 0x13:							//5		
				  DC_MOtor_Pin_High();
				  fs_flag = 0;  //zheng
				  I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x05,Buffer_DC,1);
					break;
        case 0x12:							//6
					DC_MOtor_Pin_High();
				  fs_flag = 0;  //zheng 
				  I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x03,Buffer_DC,1);
          break;*/
				
        default:
					DC_Motor_Pin_Low();	
				   fs_flag = 2;  //zheng
				  I2C_DC_Motor_Write(&hi2c1,DC_Motor_Addr,0x00,Buffer_DC_Zero,1);
          break;
			}
}




















