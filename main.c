/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.4
  * @date    06-May-2016
  * @brief   Main program body
  *
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "main.h"

#ifdef _RTE_
#include "RTE_Components.h"             /* Component selection */
#endif
#ifdef RTE_CMSIS_RTOS                   // when RTE component CMSIS RTOS is used
#include "cmsis_os.h"                   // CMSIS RTOS header file
#endif

#ifdef RTE_CMSIS_RTOS_RTX
extern uint32_t os_time;

uint32_t HAL_GetTick(void) { 
  return os_time; 
}
#endif

#include "stm32f4xx_hal_conf.h" 
#include "stm32f4xx.h"  
#include "GPIO.h"
#include "Uart2.h"
/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void GpioConfig(void);
static void NvicConfigPB0(void);
static void NvicConfigUART();
static volatile uint32_t counter = 0;
static uint32_t aa = 0;
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

#ifdef RTE_CMSIS_RTOS                   // when using CMSIS RTOS
  osKernelInitialize();                 // initialize CMSIS-RTOS
#endif

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();
   USART_TypeDef a ;
   
		
  /* Configure the system clock to 168 MHz */
  SystemClock_Config();


  /* Add your application code here
     */

#ifdef RTE_CMSIS_RTOS                   // when using CMSIS RTOS
  // create 'thread' functions that start executing,
  // example: tid_name = osThreadCreate (osThread(name), NULL);

  osKernelStart();                      // start thread execution 
#endif
		GpioConfig();
		NvicConfigPB0();
		uint8_t  temp[] ={8,11,1,9,8,11,1,9,8,11,1,9,8,11,1,9,8,11,1,9,8,11,1,9,8,11,1,9,8,11,1,9};
	  GPIOA_INIT_USART2();
		UART2_INIT();
		//NVIC_SetPriority(USART2_IRQn,1);
		//NVIC_EnableIRQ(USART2_IRQn);
		UART_HandleTypeDef uartHandle;
		uint8_t aa = 11;
		UART2_C1 |= UART_C1_TE;
			uartHandle.Instance = USART2;
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
			while(!(uartHandle.Instance->SR & 1U<<7)){};
				
			uartHandle.Instance->DR = (aa & (uint8_t)0xFFU);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
			while(!(uartHandle.Instance->SR & 1U<<7)){};
			uartHandle.Instance->DR = (aa & (uint8_t)0xFFU);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
				//UART2_SEND_DATA(temp,2);
		
		/*UART2_SEND_DATA(temp,3);
		UART2_SEND_DATA(temp,3);
		UART2_SEND_DATA(temp,3);
		UART2_SEND_DATA(temp,3);
		UART2_SEND_DATA(temp,3);
		UART2_SEND_DATA(temp,3);
		UART2_SEND_DATA(temp,3);
		UART2_SEND_DATA(temp,3);
		UART2_SEND_DATA(temp,3);
		UART2_SEND_DATA(temp,3);
		*/
		/*
		//HAL_UART_Transmit();
		GPIO_InitTypeDef gpioInit;
		GPIO_InitTypeDef gpioInit1;
		UART_InitTypeDef uartInit;
		UART_HandleTypeDef uartHandle;
		__HAL_RCC_GPIOA_CLK_ENABLE();
		gpioInit.Alternate = GPIO_AF7_USART2;
		gpioInit.Mode = GPIO_MODE_AF_PP;
		gpioInit.Pin = GPIO_PIN_2;
		gpioInit.Pull = GPIO_PULLUP;
		gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA,&gpioInit);
		__HAL_RCC_GPIOA_CLK_ENABLE();
		gpioInit1.Alternate = GPIO_AF7_USART2;
		gpioInit1.Mode = GPIO_MODE_INPUT;
		gpioInit1.Pin = GPIO_PIN_3;
		gpioInit1.Pull = GPIO_NOPULL;
		gpioInit1.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA,&gpioInit1);
		__HAL_RCC_USART2_CLK_ENABLE();
		uartInit.BaudRate = 9600;
		uartInit.Mode = UART_MODE_TX_RX;
		uartInit.OverSampling = UART_OVERSAMPLING_16;
		uartInit.Parity = UART_PARITY_NONE;
		uartInit.StopBits = UART_STOPBITS_1;
		uartInit.WordLength = UART_WORDLENGTH_8B;
		uartInit.HwFlowCtl = UART_HWCONTROL_NONE;
		uartHandle.Init = uartInit;
		uartHandle.Instance = USART2;
		
		HAL_UART_Init(&uartHandle);
		uint8_t m [2]={8,8};
		 aa = HAL_RCC_GetPCLK1Freq();
		
		//NVIC_SetPriority(USART2_IRQn,1);
		//NVIC_EnableIRQ(USART2_IRQn);
		//NvicConfigUART();
		//HAL_UART_Transmit_IT(&uartHandle,m,2);
		
		//UART1_Write('A');
		*/
		
  /* Infinite loop */
  while (1)
  {
	
		
		
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
	static void GpioConfig(void)
	{
		__HAL_RCC_GPIOD_CLK_ENABLE();
		GPIO_InitTypeDef GPIO_initStruct;
		GPIO_InitTypeDef GPIO_initStructBtn;
		
		GPIO_initStructBtn.Pin = GPIO_PIN_0;
		GPIO_initStructBtn.Mode = GPIO_MODE_IT_FALLING;
		GPIO_initStructBtn.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA,&GPIO_initStructBtn);
		
		GPIO_initStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_12 | GPIO_PIN_15;
		GPIO_initStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_initStruct.Pull = GPIO_PULLUP;
		GPIO_initStruct.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(GPIOD,&GPIO_initStruct);
	}
	static void  NvicConfigPB0(void)
	{
		__HAL_RCC_GPIOD_CLK_ENABLE();
		GPIO_InitTypeDef GPIO_initStructBtn;
       // Keil::Device:STM32Cube Framework:Classic
		
                // Device header
		GPIO_initStructBtn.Pin = GPIO_PIN_0;
		GPIO_initStructBtn.Mode = GPIO_MODE_IT_FALLING;
		GPIO_initStructBtn.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA,&GPIO_initStructBtn);
		//__HAL_RCC_SYSCFG_CLK_ENABLE();
		//SYSCFG->CMPCR = SYSCFG_EXTICR1_EXTI0_PA;
		
		
		
                // Device header
		NVIC_SetPriority(EXTI0_IRQn,0);
		NVIC_EnableIRQ(EXTI0_IRQn);
			
	}
	static void NvicConfigUART()
	{
		RCC_APB2_UART1_ON();
		SetUart1();
		
		NVIC_SetPriority(USART1_IRQn,1);
		NVIC_EnableIRQ(USART1_IRQn);
		
	}
	void USART2_IRQHandler()
	{
		
		counter++;
		if(counter > 1)
		{
			//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		
		}
		int c = 0;
		while(c< 10000000){c++;}
		if(counter <10)
		{
				//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		}
		else
		{
			//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
		}

	}
	void EXTI0_IRQHandler()
	{
		//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
	}
		
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = 4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
