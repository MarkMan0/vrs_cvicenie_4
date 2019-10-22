/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


void SystemClock_Config(void);
uint8_t check_button_state(GPIO_TypeDef* PORT, uint8_t PIN);

volatile uint8_t interrupt = 0;

int main(void)
{
  /*Default system setup*/
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  SystemClock_Config();

  /*EXTI configuration*/
  //Set interrupt priority and enable EXTI

  NVIC_SetPriority(EXTI9_5_IRQn, 2);
  NVIC_EnableIRQ(EXTI9_5_IRQn);


  //EXTI interrupt EXTI line 6
  SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI6); //set 0
  SYSCFG->EXTICR[1] |= (0x1U << (8U));	//source to PB6

  EXTI->IMR |= EXTI_IMR_MR6;		//interrupt not masked - want to call the handler - not ignored
  //EXTI->EMR &= ~(EXTI_EMR_EM6);		//event handler masked - ignore it
  EXTI->RTSR &= ~(EXTI_RTSR_RT6);	//no rising trigger
  EXTI->FTSR |= EXTI_FTSR_FT6;		//falling trigger	- turn on



  /*GPIO configuration, PB3*/
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  GPIOB->MODER &= ~(GPIO_MODER_MODER3);
  GPIOB->MODER |= GPIO_MODER_MODER3_0;
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_3);
  GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR3);
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);

  /* GPIO PB6 input config */
  GPIOB->MODER &= ~(GPIO_MODER_MODER6);	//set zero
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6);	//set zero
  GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;	//pull up


  while (1)
  {
	  if(interrupt)
	  {
		  GPIOB->BSRR |= GPIO_BSRR_BS_3;
		  for(uint16_t i=0; i<0xFF00; i++){}
		  GPIOB->BRR |= GPIO_BRR_BR_3;
		  for(uint16_t i=0; i<0xFF00; i++){}
	  }
	  else
	  {
		  GPIOB->BRR |= GPIO_BRR_BR_3;
	  }
  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  
  }
  LL_Init1msTick(8000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(8000000);
}


uint8_t check_button_state(GPIO_TypeDef* PORT, uint8_t PIN)
{
	uint8_t button_state = 0, timeout = 0;

	while(button_state < 200 && timeout < 400)
	{
		if(LL_GPIO_IsInputPinSet(PORT, PIN))
		{
			button_state += 1;
		}
		else
		{
			button_state = 0;
		}

		timeout += 1;
		LL_mDelay(1);
	}

	if((button_state >= 200) && (timeout <= 400))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


void EXTI9_5_IRQHandler(void) {
	if(check_button_state(GPIOB, 6)) {
		interrupt ^= 1;
	}
	EXTI->PR |= EXTI_PR_PR6;
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
