/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "LM75A.h"

/* USER CODE BEGIN Includes */
#include "zlg7290.h"
#include "stdio.h"
#include "Dc_motor.h"
#include "stm32f4xx_hal_iwdg.h"
#include "i2c.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ZLG_READ_ADDRESS1         0x01
#define ZLG_READ_ADDRESS2         0x10

#define ZLG_WRITE_ADDRESS1        0x10
#define ZLG_WRITE_ADDRESS2        0x11
#define BUFFER_SIZE1              (countof(Tx1_Buffer))
#define BUFFER_SIZE2              (countof(Rx2_Buffer))
#define countof(a) (sizeof(a) / sizeof(*(a)))
#define QUEUESIZE									5


uint8_t flag;//不同的按键有不同的标志位值
//uint8_t flag1 = 0;//中断标志位，每次按键产生一次中断，并开始读取8个数码管的值
uint8_t Rx2_Buffer[8]={0};
uint8_t Tx1_Buffer[8]={0};
uint8_t Rx1_Buffer[1]={0};

// 备份温度阈值
typedef struct dataBack
{
	/* data */
	uint8_t bk[3];
	uint16_t cksum;
}databk;

// 用于存储当前状态
typedef struct currentState {
	uint8_t state[2];
	uint16_t check;
}curState;

uint8_t seq_flag;       // 前序代码id

unsigned char seg7code[10]={ 0xFC,0x0C,0xDA,0xF2,0x66,0xB6,0xBE,0xE0,0xFE,0xE6}; //数码管字根
void Led(int date);	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SystemClock_Config1(void);
void Error_Handler(void);
uint16_t crc16(uint8_t *addr,uint8_t num);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void swtich_key(void);
void switch_flag(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
databk bk2;
uint16_t checksum_b1;
uint16_t Temp;
uint8_t count = 0;
uint8_t first = 0;
uint8_t second = 0;
uint8_t third = 0;
double tempvalue;
IWDG_HandleTypeDef hiwdg;
uint8_t threshold[3] = {26, 27, 29};    // 温度阈值
uint16_t thresholdck;                   // 温度阈值的校验值

uint8_t bufSize = 3;
uint16_t checksum;
int temp_flag = 1;
databk bk1;
uint16_t checksum_b2;
uint32_t unStartflag[64] __attribute__((at(0x10000000)));
curState curstate;
/* USER CODE END 0 */

void delay() {
	for(int i =0; i < 5000; i++) {
	}
}

// 算术平均滤波
double filter(int N, uint16_t Temp) {
	double sum = 0;
	short i;
	for(i =0; i < N; i++) {
		sum += LM75GetTempValue(Temp);
	}
	
	return sum / N;
}

//CRC-16校验
//addr:需要检验的字节数组
//num:需要检验的字节数
//返回值:16位CRC校验码
uint16_t crc16(uint8_t *addr,uint8_t num)
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

/*初始化看门狗*/
void MX_IWDG_Init(void) {
	hiwdg.Instance = IWDG;		//独立看门狗
	hiwdg.Init.Prescaler = IWDG_PRESCALER_64;		//设置IWDG分频系数为64
	hiwdg.Init.Reload = 4095;										//重装载值4095    
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
}

/*喂狗*/
void IWDG_Feed(void) {
	HAL_IWDG_Refresh(&hiwdg);
}

//备份温度阈值
void backup() {	
	bk1.bk[0] = threshold[0];
	bk1.bk[1] = threshold[1];
	bk1.bk[2] = threshold[2];
	bk1.cksum = crc16(bk1.bk, 3);
	
	bk2.bk[0] = threshold[0];
	bk2.bk[1] = threshold[1];
	bk2.bk[2] = threshold[2];
	bk2.cksum = crc16(bk2.bk, 3);
}

//初始化
void init() {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	//HAL_Delay(10);
	/* Configure the system clock */
	SystemClock_Config();
	//SystemClock_Config1();
	
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	LM75SetMode(CONF_ADDR,NORMOR_MODE);
	DC_Motor_Pin_Low();
	HAL_TIM_Base_Start_IT(&htim3);   //启动定时器3.

	seq_flag = 1;
}

//手动模式读键值


int main(void)
{

  /* USER CODE BEGIN 1 */
	if (unStartflag[0] != 0xaabbccdd) {
		//冷启动
		delay();
		init();
		MX_IWDG_Init();			//init dog
	} else {
		//热启动
		if (crc16(curstate.state, 2) != curstate.check) {
			delay();
			init();
			MX_IWDG_Init();			//init dog
		}
	}
    unStartflag[0] = 0xaabbccdd;    // 冷热启动标志
    backup();
	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		init();
		//printf("%x\n", crc16(threshold, 3));
		//printf("%x\n", unStartflag[0]);
		/* USER CODE BEGIN 3 */
		if(flag1 == 1)
		{
			flag1 = 0;
			if (seq_flag == 1 || seq_flag == 5) {
				I2C_ZLG7290_Read(&hi2c1,0x71,0x01,Rx1_Buffer,1);//读键值
				first = Rx1_Buffer[0];
				//printf("first:\n\r按键键值 = %#x\r\n",Rx1_Buffer[0]);//想串口发送键值
				I2C_ZLG7290_Read(&hi2c1,0x71,0x01,Rx1_Buffer,1);//读键值
				second = Rx1_Buffer[0];
				//printf("second:\n\r按键键值 = %#x\r\n",Rx1_Buffer[0]);//想串口发送键值
				I2C_ZLG7290_Read(&hi2c1,0x71,0x01,Rx1_Buffer,1);//读键值
				third = Rx1_Buffer[0];
				//printf("1: %#x, 2: %#x, 3: %#x\r\n",first, second, third);//想串口发送键?
				if (first != second || second != third || first != third) {
					continue;
				}
				IWDG_Feed();		//feed dog
				HAL_Delay(rand()%100);      // 随机时延
				seq_flag = 2;
			}
			if (seq_flag == 2) {
				swtich_key();
				printf("\n\r按键键值 = %d\r\n",flag);//想串口发送键值	
				//DC_Task(Rx1_Buffer[0]);
				curstate.state[0] = Rx1_Buffer[0];
				if (flag != 1 && flag != 2 && flag != 3) {
					flag = 0;
				}
				//Led(flag);
				curstate.state[1] = flag;
				curstate.check = crc16(curstate.state, 2);
				seq_flag = 3;
			}
			count = 0;
			
		}	else {
			if (seq_flag == 1 || seq_flag == 3) {
				if (LM75GetTempReg() == EVL_ER) {   // 没有读到温度
					temp_flag = 0;
					for(int i = 0; i < 5; i++) {    // 再检测5次
						if (LM75GetTempReg() != EVL_ER) {
							temp_flag = 1;
						}
					}
					if (temp_flag == 0) {       // 温度传感器出错
						printf("temp error!\n");
						Led(0x0B);
					}
				} else {
					temp_flag = 1;
				}
				seq_flag = 4;
			}
			if (seq_flag == 4) {
				if (temp_flag == 1 && count > 100) {
					tempvalue = filter((rand()%6) + 1, LM75GetTempReg());   // 对温度进行滤波
					IWDG_Feed();		//feed dog
					if (crc16(threshold, 3) != thresholdck) {   // 数据校验值错误
						if (crc16(bk1.bk, 3) == bk1.cksum) {    // 取备份1
							for(int i = 0; i < 3; i++) {        // 若备份1正确，使用备份1修正原来的数据
								threshold[i] = bk1.bk[i];
							}
							thresholdck = bk1.cksum;
						} else if (crc16(bk1.bk, 3) != bk1.cksum){  // 备份1错，取备份2
							for(int i = 0; i < 3; i++) {        // 若备份2正确，使用备份2修正原来的数据和备份1
								threshold[i] = bk2.bk[i];
							}
							thresholdck = bk2.cksum;
							for(int i = 0; i < 3; i++) {
								bk1.bk[i] = bk2.bk[i];
							}
							bk1.cksum = bk2.cksum;
						} else {                        // 都错，报错
							printf("threshold error!\n");    
							Led(0x0B);
						}
					} 
					HAL_Delay(rand()%100);      // 随机时延
					if (tempvalue > threshold[0] && tempvalue < threshold[1]) {
						//DC_Task(0x1c);
						//Led(1);
						curstate.state[0] = 0x1c;      // 存储当前状态
						curstate.state[1] = 1;
						curstate.check = crc16(curstate.state, 2);
					} else if (tempvalue >= threshold[1] && tempvalue < threshold[2]) {
						//DC_Task(0x1b);
						//Led(2);
						curstate.state[0] = 0x1b;       // 存储当前状态
						curstate.state[1] = 2;
						curstate.check = crc16(curstate.state, 2);
					} else if (tempvalue >= threshold[2]){
						//DC_Task(0x1a);
						//Led(3);
						curstate.state[0] = 0x1a;       // 存储当前状态
						curstate.state[1] = 3;
						curstate.check = crc16(curstate.state, 2);
					} else {
						//DC_Task(0x1d);
						//Led(0);
						curstate.state[0] = 0x1d;       // 存储当前状态
						curstate.state[1] = 0;
						curstate.check = crc16(curstate.state, 2);
					}
					count = 0;
					seq_flag = 5;
				}
			}
		}
        // fresh
        int seq = rand() % 2;   // 乱序执行
        switch(seq) {
        case 0:
            DC_Task(curstate.state[0]);
		    Led(curstate.state[1]);
        case 1:
            Led(curstate.state[1]);
            DC_Task(curstate.state[0]);
        }
		// DC_Task(curstate.state[0]);
		// Led(curstate.state[1]);
		count++;
	  HAL_Delay(100);
  }
  /* USER CODE END 3 */
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  //HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  //HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
	/*add*/
	//RCC_OscInitTypeDef RCC_OscInitStruct;
  //RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	/*end*/
}

/* USER CODE BEGIN 4 */

/*temp add*/
/*void SystemClock_Config1(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  
  //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
//}
end*/

void Led(int date) //显示函数
{
/*****************数据转换*****************************/ 
unsigned int qian,bai,shi,ge;
  qian=date/1000;				 //求千位
  bai=date%1000/100;		 //求百位
  shi=date%1000%100/10;			 //求十位
  ge=date%10;						 //求个位	
	
//	I2C_Gets(ZLG7290_I2C_ADDR,0x10,1,Rx2_Buffer,8);//读8位数码管
	
	if(fs_flag==0) Tx1_Buffer[3] = 0x02;	
	if(fs_flag==1) Tx1_Buffer[3] = 0x00;
	if(fs_flag ==2) Tx1_Buffer[3] =seg7code[0];

	Tx1_Buffer[4] = seg7code[qian];
	Tx1_Buffer[5] = seg7code[bai];
	Tx1_Buffer[6] = seg7code[shi];
	Tx1_Buffer[7] = seg7code[ge];
	
  I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,8);
}



void swtich_key(void)
{
	switch(Rx1_Buffer[0])
	{
        case 0x1C:
					flag = 1;					
          break;
        case 0x1B:	
					flag = 2;
          break;
        case 0x1A:	
					flag = 3;
          break;
        case 0x14:
					flag = 4;
          break;   
				case 0x13:
					flag = 5;
					break;
        case 0x12:
					flag = 6;
          break;
        case 0x0C:
					flag = 7;
          break;
        case 0x0B:
          flag = 8;
          break;
				case 0x0A:
					flag = 9;
					break;
				case 0x03:
					flag = 15;
					break;
				case 0x19:
					flag = 10;
					break;
				case 0x11:
					flag = 11;
					break;
				case 0x09:
					flag = 12;
					break;
				case 0x01:
					flag = 13;
					break;
				case 0x02:
					flag = 14;
					break;
        default:
          break;
			}
}

void switch_flag(void){
	switch(flag){
			case 1:
				Tx1_Buffer[0] = 0x0c;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{									
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);					
					}
				break;
			case 2:
				Tx1_Buffer[0] = 0xDA;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);		
					}
				break;
			case 3:
				Tx1_Buffer[0] = 0xF2;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);		
					}
				break;
			case 4:
				Tx1_Buffer[0] = 0x66;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);					
					}
				break;
			case 5:
				Tx1_Buffer[0] = 0xB6;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
			case 6:
				Tx1_Buffer[0] = 0xBE;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
			case 7:
				Tx1_Buffer[0] = 0xE0;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
			case 8:
				Tx1_Buffer[0] = 0xFE;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);					
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);							
					}
				break;
			case 9:
				Tx1_Buffer[0] = 0xE6;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);					
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);					
					}
				break;
			case 10:
				Tx1_Buffer[0] = 0xEE;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);					
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
			case 11:
				Tx1_Buffer[0] = 0x3E;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);							
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
					case 12:
				Tx1_Buffer[0] = 0x9C;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);								
					}
				break;
					case 13:
				Tx1_Buffer[0] = 0x7A;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);									
					}
				break;
					case 14:
							Tx1_Buffer[0] = 0x00;
							I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,8);
						break;
					case 15:
				Tx1_Buffer[0] = 0xFC;
				if(Rx2_Buffer[0] == 0)
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);
					}
					else
					{
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS2,Rx2_Buffer,BUFFER_SIZE2);						
						I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,1);						
					}
				break;
			default:
				break;
		}
}


int fputc(int ch, FILE *f)
{ 
  uint8_t tmp[1]={0};
	tmp[0] = (uint8_t)ch;
	HAL_UART_Transmit(&huart1,tmp,1,10);	
	return ch;
}
/* USER CODE END 4 */

/*temp add*/
int fputc1(int ch, FILE *f)
{     
    while((USART1->SR&0X40)==0);
    USART1->DR = (uint8_t) ch;      
    return ch;
}
/*end*/

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
