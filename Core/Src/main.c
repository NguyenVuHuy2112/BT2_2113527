/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "uartstdio.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef	enum {
	STATE_LED_ON,
	STATE_WAIT_LED_ON,
	STATE_LED_OFF,
	STATE_WAIT_LED_OFF,
}sm_sw_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
xQueueHandle		queue_led;
sm_sw_t 	state;
UART_t 		data_uart_1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void LED_Task( void *pvParameters );
static void UART_Task( void *pvParameters );
static void Switch_Task( void *pvParameters );
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  queue_led = xQueueCreate(5, sizeof(led_command_t));

  data_uart_1.position_uart = 0;
  data_uart_1.uart_flag = 0;
  state = STATE_LED_OFF;

  xTaskCreate(LED_Task, "LED_Task",configMINIMAL_STACK_SIZE, NULL, 2, NULL);
  xTaskCreate(UART_Task, "UART_Task",configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(Switch_Task, "SW task",configMINIMAL_STACK_SIZE, NULL, 1, NULL);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  vTaskStartScheduler();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(16000000);
  LL_SetSystemCoreClock(16000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB6   ------> USART1_TX
  PB7   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void LED_Task( void *pvParameters )
{
  led_command_t	received_led_command;
  BaseType_t	ret;
	while(1)
	{
		ret = xQueueReceive(queue_led, &received_led_command, portMAX_DELAY);
		if(ret == pdPASS)
		{
			if (received_led_command == LED_ON)
			{
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
			}
			else 	if (received_led_command == LED_OFF)
			{
				LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
			}
			else 	if (received_led_command == LED_TOGGLE)
			{
				LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_1);
			}
		}
	}
}
/**
 * The `UART_Task` function processes incoming UART commands to control an LED based on the received
 * commands.
 * 
 * @param pvParameters In the provided code snippet, the `pvParameters` parameter in the `UART_Task`
 * function is not being used. This parameter is typically used to pass any additional parameters or
 * data to the task when it is created using FreeRTOS. If you don't have any specific parameters to
 * pass to the task
 */
static void UART_Task( void *pvParameters )
{
  led_command_t send_led_command;
	while(1)
	{
		if (data_uart_1.uart_flag == 1)
		{
		  if (memcmp(data_uart_1.uart_buffer, "ON\r", strlen((char *)data_uart_1.uart_buffer)) == 0)
		  {
			send_led_command = LED_ON;
			state = STATE_LED_ON;
		  }
		  else if (memcmp(data_uart_1.uart_buffer, "OFF\r", strlen((char *)data_uart_1.uart_buffer)) == 0)
		  {
			send_led_command = LED_OFF;
			state = STATE_LED_OFF;
		  }
		  else if (memcmp(data_uart_1.uart_buffer, "TOGGLE\r", strlen((char *)data_uart_1.uart_buffer)) == 0)
		  {
			send_led_command = LED_TOGGLE;
			if(state == STATE_LED_ON) state = STATE_LED_OFF;
			else state = STATE_LED_ON;
		  }
		  xQueueSend(queue_led,&send_led_command,0);
		  data_uart_1.uart_flag = 0;
		  memset(data_uart_1.uart_buffer, '\0', sizeof(data_uart_1.uart_buffer));
		}
		vTaskDelay(100);
	}
}

static uint32_t read_button()
{

	if (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) 	return PRESSED;
	else 												return RELEASED;
}
static void Switch_Task( void *pvParameters )
{
	uint32_t		switch_prev_state = read_button();
	uint32_t		switch_current_state;
	led_command_t	Switch_Task_led_command;


	while(1)
	{
		switch_current_state = read_button();
		if (switch_current_state != switch_prev_state)
		{
			switch_prev_state = switch_current_state;
			switch (state)
			{
				case STATE_LED_OFF:
					if(switch_current_state == PRESSED)
					{
						state = STATE_WAIT_LED_ON;
					}
					Switch_Task_led_command = LED_OFF;
					break;
				case STATE_WAIT_LED_ON:
					if(switch_current_state == RELEASED)
					{
						state = STATE_LED_ON;
					}
					Switch_Task_led_command = LED_ON;
					break;
				case STATE_LED_ON:
					if(switch_current_state == PRESSED)
					{
						state = STATE_WAIT_LED_OFF;
					}
					Switch_Task_led_command = LED_ON;
					break;
				case STATE_WAIT_LED_OFF:
					if(switch_current_state == PRESSED)
					{
						state = STATE_LED_ON;
					}
					Switch_Task_led_command = LED_OFF;
					break;
			}
			xQueueSend(queue_led,&Switch_Task_led_command,0);
		}
		vTaskDelay(10);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
