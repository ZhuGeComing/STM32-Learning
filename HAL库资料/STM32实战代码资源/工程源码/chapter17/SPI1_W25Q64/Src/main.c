/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const uint8_t table0[]= {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};  /* �������޵㣩*/
const uint8_t table1[]= {0xbf,0x86,0xdb,0xcf,0xe6,0xed,0xfd,0x87,0xff,0xef};  /* �������е㣩*/

uint8_t  page_buff[256];   /* ����ҳд��������256�ֽڣ�*/
uint8_t  sector_buff[4096];/* ����������������4096�ֽڣ�*/
uint8_t  num,display;      /* �����������ʾ���� */
uint8_t w25q64_status;     /* ����״̬����1 */
uint32_t status;           /* ����״̬����2 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SEG_Display(uint32_t num);     /*  �������ʾ����  */
void W25Q64_CS(uint8_t status);    	/* W25Q64Ƭѡ����  */
void W25Q64_Write_Enable(void);			/* W25Q64дʹ�ܺ���  */
void W25Q64_Chip_Erase(void);				/* W25Q64оƬ��������  */
void W25Q64_Sector_Erase(uint32_t address);/* W25Q64������������  */
uint8_t W25Q64_Read_Status_Register1(void);/* W25Q64��״̬�Ĵ���1����  */
uint8_t W25Q64_Read_Status_Register2(void);/* W25Q64��״̬�Ĵ���2����  */
uint8_t W25Q64_Read_Data(uint32_t address);/*  W25Q64��ȡ���ݺ���  */
void W25Q64_Page_Program_1Byte(uint32_t address,uint8_t prog_data);  /* W25Q64ҳд��1�ֽڣ�����  */
void W25Q64_Page_Program_256Byte(uint32_t address,uint8_t buff256[]);/* W25Q64ҳд��256�ֽڣ����� */
void W25Q64_Sector_Program_256Byte(uint32_t address,uint8_t pbuff256[],uint16_t byte_number);/* W25Q64������̺���  */
void Error(void);/*  ������  */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
	
	/* Ϊҳд���������鸳ֵ0-255 */
	uint16_t  x;
	for(x=0;x<256;x++)
	{
		page_buff[x]=x;
	}
	/* W25Q64дʹ�� */
	W25Q64_Write_Enable();	
	/* �ڵ�ַ7FFFF0λ�ÿ�ʼд��15���ֽڣ�ֵΪ0-14�� */
	W25Q64_Sector_Program_256Byte(0x007FFFF0,page_buff,15);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* ��ȡ��ַ7FFFF5��ַ�����ݣ�ӦΪ5��  */
    display=W25Q64_Read_Data(0x007FFFF5);
		/* �������ʾ��ȡ��ֵ */
		SEG_Display(display);    

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE7 PE8 PE9 
                           PE10 PE11 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF9 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*  �������ʾ����  */
void SEG_Display(uint32_t num)
{
	/* ȡ����λ����ʾֵ */
  uint8_t GeWei,ShiWei,BaiWei,QianWei;
	GeWei   = (uint8_t)(num%10);       
	ShiWei  = (uint8_t)(num%100/10);
	BaiWei  = (uint8_t)(num%1000/100); 
	QianWei = (uint8_t)(num/1000); 

	/* ʹ�������λ����  */
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET);	

	/* ��ʾ��λ */
	GPIOE->ODR |= (uint16_t)(table0[GeWei]);
	/* ʹ�ܸ�λ */
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11,GPIO_PIN_SET);    
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_Delay(3);
	/* ���� */
  GPIOE->ODR &= 0xFF00;    
	GPIOE->ODR |= 0x0F00; 
	HAL_Delay(1);
	
	/* ��ʾʮλ */
	GPIOE->ODR |= (uint16_t)(table0[ShiWei]);
	/* ʹ��ʮλ */ 
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_Delay(3);
	/* ���� */
  GPIOE->ODR &= 0xFF00;    
	GPIOE->ODR |= 0x0F00;  
	HAL_Delay(1);
	
	/* ��ʾ��λ */
	GPIOE->ODR |= (uint16_t)(table0[BaiWei]);
	/* ʹ�ܰ�λ */
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11,GPIO_PIN_SET);    
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_Delay(3);
	/* ���� */
  GPIOE->ODR &= 0xFF00;    
	GPIOE->ODR |= 0x0F00; 
	HAL_Delay(1);
	
	/* ��ʾǧλ */
	GPIOE->ODR |= (uint16_t)(table0[QianWei]);
	/* ʹ��ǧλ */ 
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
	HAL_Delay(3);
	/* ���� */
  GPIOE->ODR &= 0xFF00;    
	GPIOE->ODR |= 0x0F00; 
	HAL_Delay(1);
}

/* W25Q64Ƭѡ����  */
void W25Q64_CS(uint8_t status)
{
		if(status==0)
		{
			HAL_GPIO_WritePin(
												GPIOF,
												GPIO_PIN_9,
												GPIO_PIN_RESET
										   );
		}
		else
		{
			HAL_GPIO_WritePin(
												GPIOF,
												GPIO_PIN_9,
												GPIO_PIN_SET
										   );
		} 
}
/* W25Q64дʹ�ܺ���  */
void W25Q64_Write_Enable(void)
{
	/* ��CS���õͣ�ƬѡW25Q64 */ 
	W25Q64_CS(0);
	/* ����06Hָ���д���� */
	uint8_t command = 0x06;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* ��CS���øߣ�ȡ��ƬѡW25Q64 */ 
	W25Q64_CS(1);
}
/* W25Q64оƬ��������  */
void W25Q64_Chip_Erase(void)
{
	/* ��CS���õͣ�ƬѡW25Q64 */ 
	W25Q64_CS(0);
	/* ����C7Hָ���������оƬ */
	uint8_t command = 0xC7;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* ��CS���øߣ�ȡ��ƬѡW25Q64 */ 
	W25Q64_CS(1);
}
/* W25Q64������������  */
void W25Q64_Sector_Erase(uint32_t address)
{
	/* ��CS���õͣ�ƬѡW25Q64 */ 
	W25Q64_CS(0);
	/* ����20Hָ�����һ������*/
	uint8_t command = 0x20;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* ���ʹ洢����ַ */
	uint8_t temp;
	temp=address>>16;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address>>8;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	/* ��CS���øߣ�ȡ��ƬѡW25Q64 */ 
	W25Q64_CS(1);
}
/* W25Q64��״̬�Ĵ���1����  */
uint8_t W25Q64_Read_Status_Register1(void)
{
	/* ��CS���õͣ�ƬѡW25Q64 */ 
	W25Q64_CS(0);
	/* ����05Hָ���ȡ״̬�Ĵ���1 */
	uint8_t command = 0x05;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* �ı乤������Ϊ�½��� */	
	SPI1->CR1 |= 0x0001; 
	/* ��ȡ״̬�Ĵ���1������ */
	uint8_t reg1_status;
	status = HAL_SPI_Receive(&hspi1,&reg1_status,1,5);
	/* �ı乤������Ϊ������ */
	SPI1->CR1 &= 0xFFFE;
	/* ��CS���øߣ�ȡ��ƬѡW25Q64 */ 
	W25Q64_CS(1);
	/* ����״̬�Ĵ���1������ */
	return reg1_status;
}
/* W25Q64��״̬�Ĵ���2����  */
uint8_t W25Q64_Read_Status_Register2(void)
{
	/* ��CS���õͣ�ƬѡW25Q64 */ 
	W25Q64_CS(0);
	/* ����35Hָ���ȡ״̬�Ĵ���2 */
	uint8_t command = 0x35;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* �ı乤������Ϊ�½��� */	
	SPI1->CR1 |= 0x0001; 
	/* ��ȡ״̬�Ĵ���1������ */
	uint8_t reg2_status;
	status = HAL_SPI_Receive(&hspi1,&reg2_status,1,5);
	/* �ı乤������Ϊ������ */
	SPI1->CR1 &= 0xFFFE;
	/* ��CS���øߣ�ȡ��ƬѡW25Q64 */ 
	W25Q64_CS(1);
	/* ����״̬�Ĵ���1������ */
	return reg2_status;
}
/*  W25Q64��ȡ���ݺ���  */
uint8_t W25Q64_Read_Data(uint32_t address)
{
	/* ��CS���õͣ�ƬѡW25Q64 */ 
	W25Q64_CS(0);
	/* ����03Hָ���ȡĳһ��ַ������ */
	uint8_t command = 0x03;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
 	/* ���ʹ洢����ַ */
	uint8_t temp;
	temp=address>>16;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address>>8;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	/* �ı乤������Ϊ�½��� */	
	SPI1->CR1 |= 0x0001; 
	/* ��ȡ��ǰ��ַ������ */
	uint8_t read_data;
	status = HAL_SPI_Receive(&hspi1,&read_data,1,5);
	/* �ı乤������Ϊ������ */	
	SPI1->CR1 &= 0xFFFE;
	/* ��CS���øߣ�ȡ��ƬѡW25Q64 */ 
	W25Q64_CS(1);
	/* ����ȡ�������ݷ��� */ 
	return read_data;
}
/* W25Q64ҳд��1�ֽڣ�����  */
void W25Q64_Page_Program_1Byte(uint32_t address,uint8_t prog_data)
{
	/* ��CS���õͣ�ƬѡW25Q64 */ 
	W25Q64_CS(0);
	/* ����02Hָ������256�ֽ����� */
	uint8_t command = 0x02;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* ���ʹ洢����ַ */
	uint8_t temp;
	temp=address>>16;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address>>8;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	/* �������� */
	status = HAL_SPI_Transmit (&hspi1,&prog_data,1,5);
	/* ��CS���øߣ�ȡ��ƬѡW25Q64 */ 
	W25Q64_CS(1);
}
/* W25Q64ҳд��256�ֽڣ����� */
void W25Q64_Page_Program_256Byte(uint32_t address,uint8_t buff256[])
{
	/* ��CS���õͣ�ƬѡW25Q64 */ 
	W25Q64_CS(0);
	/* ����02Hָ������256�ֽ����� */
	uint8_t command = 0x02;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
 	/* ���ʹ洢����ַ */
	uint8_t temp;
	temp=address>>16;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address>>8;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	/* �������� */
	uint16_t  x;
	for(x=0;x<256;x++)
	{
		status = HAL_SPI_Transmit (&hspi1,&buff256[x],1,5);
	}
	/* ��CS���øߣ�ȡ��ƬѡW25Q64 */ 
	W25Q64_CS(1);
}
/* W25Q64������̺���  */
/* ������һ�������ڵ�ָ����ַ��д�����256�ֽ����ݣ������������� */
/* ���������ʣ��ռ�С�ڴ�д�������������ִ�д����� */
void W25Q64_Sector_Program_256Byte(uint32_t address,uint8_t pbuff256[],uint16_t byte_number)
{
	/* ��ȡ������ʼλ�õ�ַ  */ 	
	uint32_t sector_start_address;
	sector_start_address=address&0xFFFFF000;
	/* ��ȡ��ǰ�������������������������� */
	uint32_t x;
	for(x=0;x<4096;x++)
	{
		sector_buff[x]=W25Q64_Read_Data(sector_start_address+x);
	}
	/* �ж�������ʣ��ռ��Ƿ��㹻ʹ�� */ 
	if((4096-address&0x00000FFF)<byte_number)
	{
		Error();
	}
	/* ��ȡ����ҳ��ַ */
	uint8_t page_number;  /* ������ҳ��  */
	uint8_t page_address; /* ҳ�ڵ�ַ    */
	uint16_t temp1;
	temp1=address&0x00000FFF;
	page_number=temp1/256;
	page_address=temp1%256;
	/* ����д��������ֽ�д������������ */	
	uint16_t y;
	for(y=0;y<byte_number;y++)
	{
		sector_buff[page_number*256+page_address+y]=pbuff256[y];
	}
	/* д���� */	
	W25Q64_Write_Enable();
	/* ���������� */
  W25Q64_Sector_Erase(address);
	/* �ȴ���д��� */
	do
	{
		w25q64_status=W25Q64_Read_Status_Register1();
	}
	while(w25q64_status&0x01);
	/* д���� */	
	W25Q64_Write_Enable();
	/* ����������һ����д�� */
	uint8_t temp_buff[256];
	uint8_t z;
	uint16_t k;
	for(z=0;z<16;z++)  
	{
		for(k=0;k<256;k++)
		{
		    temp_buff[k]=sector_buff[z*256+k];
		}
		/* д���� */	
		W25Q64_Write_Enable();
		/* д��256���ֽ� */
		W25Q64_Page_Program_256Byte(sector_start_address+z*256,temp_buff);
		/* �ȴ���д��� */
		do
		{
			w25q64_status=W25Q64_Read_Status_Register1();
		}
		while(w25q64_status&0x01);
	}
}

/* ���������գ� */
void Error(void)
{

}

/* USER CODE END 4 */

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
