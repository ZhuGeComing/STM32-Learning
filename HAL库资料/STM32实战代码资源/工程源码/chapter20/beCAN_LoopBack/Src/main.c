/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
	
	* beCAN ����ģʽ���ԣ���׼��ʶ����8�ֽ����ݡ��������ʾ����ֵ 
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

CAN_FilterConfTypeDef   sFilterConfig;     /* ����CAN�˲������ýṹ����� */

static CanTxMsgTypeDef  TxMessage;         /* ����CAN������Ϣ�ṹ����� */
static CanRxMsgTypeDef  RxMessage;         /* ����CAN������Ϣ�ṹ����� */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SEG_Display(uint32_t num);     /*  �������ʾ����  */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
const uint8_t table0[]= {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};  /* �������޵㣩*/
const uint8_t table1[]= {0xbf,0x86,0xdb,0xcf,0xe6,0xed,0xfd,0x87,0xff,0xef};  /* �������е㣩*/

uint32_t  display;   /* ������ʾ���� */

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
  MX_CAN_Init();

  /* USER CODE BEGIN 2 */
	
	hcan.pTxMsg = &TxMessage; /* ���ͽṹ��ָ�� */
	hcan.pRxMsg = &RxMessage; /* ���սṹ��ָ�� */
	
	/* ����beCAN������Ϣ  */
  hcan.pTxMsg->StdId = 0x123;
  hcan.pTxMsg->RTR = CAN_RTR_DATA;
  hcan.pTxMsg->IDE = CAN_ID_STD;
  hcan.pTxMsg->DLC = 8;
  hcan.pTxMsg->Data[0] = 1 ;
  hcan.pTxMsg->Data[1] = 2 ;
  hcan.pTxMsg->Data[2] = 3 ;
  hcan.pTxMsg->Data[3] = 4 ;
  hcan.pTxMsg->Data[4] = 5 ;
  hcan.pTxMsg->Data[5] = 6 ;
  hcan.pTxMsg->Data[6] = 7 ;
  hcan.pTxMsg->Data[7] = 8 ;
 
  /* ����beCAN������  */
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 0;  
  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* ��ʼbeCAN���ʹ���  */
		if(HAL_CAN_Transmit(&hcan, 10) != HAL_OK)
		{
				/* ���ʹ��� */
				Error_Handler();
		}
		if(HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_READY)
		{
				Error_Handler();
		}

		/* ��ʼbeCAN���մ���  */
		if(HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
		{
				/* ���մ��� */
				Error_Handler();   
		}
		if(HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_READY)
		{
				Error_Handler();
		}
		
		/* ������4��ֵ���������  */		
		display = hcan.pRxMsg->Data[4];
			
		/*  �������ʾ��ˢ������Ϊ16����  */
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

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 48;
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_4TQ;
  hcan.Init.BS2 = CAN_BS2_5TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
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
