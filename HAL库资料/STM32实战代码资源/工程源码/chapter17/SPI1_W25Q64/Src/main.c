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
const uint8_t table0[]= {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};  /* 共阴（无点）*/
const uint8_t table1[]= {0xbf,0x86,0xdb,0xcf,0xe6,0xed,0xfd,0x87,0xff,0xef};  /* 共阴（有点）*/

uint8_t  page_buff[256];   /* 定义页写缓冲区（256字节）*/
uint8_t  sector_buff[4096];/* 定义扇区缓冲区（4096字节）*/
uint8_t  num,display;      /* 定义计数及显示变量 */
uint8_t w25q64_status;     /* 定义状态变量1 */
uint32_t status;           /* 定义状态变量2 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SEG_Display(uint32_t num);     /*  数码管显示函数  */
void W25Q64_CS(uint8_t status);    	/* W25Q64片选函数  */
void W25Q64_Write_Enable(void);			/* W25Q64写使能函数  */
void W25Q64_Chip_Erase(void);				/* W25Q64芯片擦除函数  */
void W25Q64_Sector_Erase(uint32_t address);/* W25Q64扇区擦除函数  */
uint8_t W25Q64_Read_Status_Register1(void);/* W25Q64读状态寄存器1函数  */
uint8_t W25Q64_Read_Status_Register2(void);/* W25Q64读状态寄存器2函数  */
uint8_t W25Q64_Read_Data(uint32_t address);/*  W25Q64读取数据函数  */
void W25Q64_Page_Program_1Byte(uint32_t address,uint8_t prog_data);  /* W25Q64页写（1字节）函数  */
void W25Q64_Page_Program_256Byte(uint32_t address,uint8_t buff256[]);/* W25Q64页写（256字节）函数 */
void W25Q64_Sector_Program_256Byte(uint32_t address,uint8_t pbuff256[],uint16_t byte_number);/* W25Q64扇区编程函数  */
void Error(void);/*  错误函数  */

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
	
	/* 为页写缓冲区数组赋值0-255 */
	uint16_t  x;
	for(x=0;x<256;x++)
	{
		page_buff[x]=x;
	}
	/* W25Q64写使能 */
	W25Q64_Write_Enable();	
	/* 在地址7FFFF0位置开始写入15个字节（值为0-14） */
	W25Q64_Sector_Program_256Byte(0x007FFFF0,page_buff,15);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* 读取地址7FFFF5地址的数据（应为5）  */
    display=W25Q64_Read_Data(0x007FFFF5);
		/* 数码管显示读取的值 */
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

/*  数码管显示函数  */
void SEG_Display(uint32_t num)
{
	/* 取出四位数显示值 */
  uint8_t GeWei,ShiWei,BaiWei,QianWei;
	GeWei   = (uint8_t)(num%10);       
	ShiWei  = (uint8_t)(num%100/10);
	BaiWei  = (uint8_t)(num%1000/100); 
	QianWei = (uint8_t)(num/1000); 

	/* 使能数码管位驱动  */
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET);	

	/* 显示个位 */
	GPIOE->ODR |= (uint16_t)(table0[GeWei]);
	/* 使能个位 */
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11,GPIO_PIN_SET);    
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_Delay(3);
	/* 消隐 */
  GPIOE->ODR &= 0xFF00;    
	GPIOE->ODR |= 0x0F00; 
	HAL_Delay(1);
	
	/* 显示十位 */
	GPIOE->ODR |= (uint16_t)(table0[ShiWei]);
	/* 使能十位 */ 
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_Delay(3);
	/* 消隐 */
  GPIOE->ODR &= 0xFF00;    
	GPIOE->ODR |= 0x0F00;  
	HAL_Delay(1);
	
	/* 显示百位 */
	GPIOE->ODR |= (uint16_t)(table0[BaiWei]);
	/* 使能百位 */
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11,GPIO_PIN_SET);    
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_Delay(3);
	/* 消隐 */
  GPIOE->ODR &= 0xFF00;    
	GPIOE->ODR |= 0x0F00; 
	HAL_Delay(1);
	
	/* 显示千位 */
	GPIOE->ODR |= (uint16_t)(table0[QianWei]);
	/* 使能千位 */ 
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
	HAL_Delay(3);
	/* 消隐 */
  GPIOE->ODR &= 0xFF00;    
	GPIOE->ODR |= 0x0F00; 
	HAL_Delay(1);
}

/* W25Q64片选函数  */
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
/* W25Q64写使能函数  */
void W25Q64_Write_Enable(void)
{
	/* 将CS端置低，片选W25Q64 */ 
	W25Q64_CS(0);
	/* 发送06H指令，打开写保护 */
	uint8_t command = 0x06;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* 将CS端置高，取消片选W25Q64 */ 
	W25Q64_CS(1);
}
/* W25Q64芯片擦除函数  */
void W25Q64_Chip_Erase(void)
{
	/* 将CS端置低，片选W25Q64 */ 
	W25Q64_CS(0);
	/* 发送C7H指令，擦除整个芯片 */
	uint8_t command = 0xC7;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* 将CS端置高，取消片选W25Q64 */ 
	W25Q64_CS(1);
}
/* W25Q64扇区擦除函数  */
void W25Q64_Sector_Erase(uint32_t address)
{
	/* 将CS端置低，片选W25Q64 */ 
	W25Q64_CS(0);
	/* 发送20H指令，擦除一个扇区*/
	uint8_t command = 0x20;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* 发送存储器地址 */
	uint8_t temp;
	temp=address>>16;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address>>8;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	/* 将CS端置高，取消片选W25Q64 */ 
	W25Q64_CS(1);
}
/* W25Q64读状态寄存器1函数  */
uint8_t W25Q64_Read_Status_Register1(void)
{
	/* 将CS端置低，片选W25Q64 */ 
	W25Q64_CS(0);
	/* 发送05H指令，读取状态寄存器1 */
	uint8_t command = 0x05;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* 改变工作边沿为下降沿 */	
	SPI1->CR1 |= 0x0001; 
	/* 读取状态寄存器1的内容 */
	uint8_t reg1_status;
	status = HAL_SPI_Receive(&hspi1,&reg1_status,1,5);
	/* 改变工作边沿为上升沿 */
	SPI1->CR1 &= 0xFFFE;
	/* 将CS端置高，取消片选W25Q64 */ 
	W25Q64_CS(1);
	/* 返回状态寄存器1的内容 */
	return reg1_status;
}
/* W25Q64读状态寄存器2函数  */
uint8_t W25Q64_Read_Status_Register2(void)
{
	/* 将CS端置低，片选W25Q64 */ 
	W25Q64_CS(0);
	/* 发送35H指令，读取状态寄存器2 */
	uint8_t command = 0x35;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* 改变工作边沿为下降沿 */	
	SPI1->CR1 |= 0x0001; 
	/* 读取状态寄存器1的内容 */
	uint8_t reg2_status;
	status = HAL_SPI_Receive(&hspi1,&reg2_status,1,5);
	/* 改变工作边沿为上升沿 */
	SPI1->CR1 &= 0xFFFE;
	/* 将CS端置高，取消片选W25Q64 */ 
	W25Q64_CS(1);
	/* 返回状态寄存器1的内容 */
	return reg2_status;
}
/*  W25Q64读取数据函数  */
uint8_t W25Q64_Read_Data(uint32_t address)
{
	/* 将CS端置低，片选W25Q64 */ 
	W25Q64_CS(0);
	/* 发送03H指令，读取某一地址的数据 */
	uint8_t command = 0x03;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
 	/* 发送存储器地址 */
	uint8_t temp;
	temp=address>>16;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address>>8;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	/* 改变工作边沿为下降沿 */	
	SPI1->CR1 |= 0x0001; 
	/* 读取当前地址的数据 */
	uint8_t read_data;
	status = HAL_SPI_Receive(&hspi1,&read_data,1,5);
	/* 改变工作边沿为上升沿 */	
	SPI1->CR1 &= 0xFFFE;
	/* 将CS端置高，取消片选W25Q64 */ 
	W25Q64_CS(1);
	/* 将读取到的数据返回 */ 
	return read_data;
}
/* W25Q64页写（1字节）函数  */
void W25Q64_Page_Program_1Byte(uint32_t address,uint8_t prog_data)
{
	/* 将CS端置低，片选W25Q64 */ 
	W25Q64_CS(0);
	/* 发送02H指令，最多编程256字节数据 */
	uint8_t command = 0x02;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
	/* 发送存储器地址 */
	uint8_t temp;
	temp=address>>16;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address>>8;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	/* 发送数据 */
	status = HAL_SPI_Transmit (&hspi1,&prog_data,1,5);
	/* 将CS端置高，取消片选W25Q64 */ 
	W25Q64_CS(1);
}
/* W25Q64页写（256字节）函数 */
void W25Q64_Page_Program_256Byte(uint32_t address,uint8_t buff256[])
{
	/* 将CS端置低，片选W25Q64 */ 
	W25Q64_CS(0);
	/* 发送02H指令，最多编程256字节数据 */
	uint8_t command = 0x02;
	status = HAL_SPI_Transmit (&hspi1,&command,1,5);
 	/* 发送存储器地址 */
	uint8_t temp;
	temp=address>>16;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address>>8;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	temp=address;
	status = HAL_SPI_Transmit (&hspi1,&temp,1,5);
	/* 发送数据 */
	uint16_t  x;
	for(x=0;x<256;x++)
	{
		status = HAL_SPI_Transmit (&hspi1,&buff256[x],1,5);
	}
	/* 将CS端置高，取消片选W25Q64 */ 
	W25Q64_CS(1);
}
/* W25Q64扇区编程函数  */
/* 在任意一个扇区内的指定地址，写入最多256字节数据（带扇区擦除） */
/* 如果扇区内剩余空间小于待写入的数据量，将执行错误函数 */
void W25Q64_Sector_Program_256Byte(uint32_t address,uint8_t pbuff256[],uint16_t byte_number)
{
	/* 获取扇区起始位置地址  */ 	
	uint32_t sector_start_address;
	sector_start_address=address&0xFFFFF000;
	/* 读取当前扇区内所有内容至扇区缓冲区 */
	uint32_t x;
	for(x=0;x<4096;x++)
	{
		sector_buff[x]=W25Q64_Read_Data(sector_start_address+x);
	}
	/* 判断扇区内剩余空间是否足够使用 */ 
	if((4096-address&0x00000FFF)<byte_number)
	{
		Error();
	}
	/* 获取扇内页地址 */
	uint8_t page_number;  /* 扇区内页号  */
	uint8_t page_address; /* 页内地址    */
	uint16_t temp1;
	temp1=address&0x00000FFF;
	page_number=temp1/256;
	page_address=temp1%256;
	/* 将待写入的数据字节写入扇区缓冲区 */	
	uint16_t y;
	for(y=0;y<byte_number;y++)
	{
		sector_buff[page_number*256+page_address+y]=pbuff256[y];
	}
	/* 写允许 */	
	W25Q64_Write_Enable();
	/* 擦除该扇区 */
  W25Q64_Sector_Erase(address);
	/* 等待擦写完成 */
	do
	{
		w25q64_status=W25Q64_Read_Status_Register1();
	}
	while(w25q64_status&0x01);
	/* 写允许 */	
	W25Q64_Write_Enable();
	/* 将扇区内容一次性写入 */
	uint8_t temp_buff[256];
	uint8_t z;
	uint16_t k;
	for(z=0;z<16;z++)  
	{
		for(k=0;k<256;k++)
		{
		    temp_buff[k]=sector_buff[z*256+k];
		}
		/* 写允许 */	
		W25Q64_Write_Enable();
		/* 写入256个字节 */
		W25Q64_Page_Program_256Byte(sector_start_address+z*256,temp_buff);
		/* 等待擦写完成 */
		do
		{
			w25q64_status=W25Q64_Read_Status_Register1();
		}
		while(w25q64_status&0x01);
	}
}

/* 错误函数（空） */
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
