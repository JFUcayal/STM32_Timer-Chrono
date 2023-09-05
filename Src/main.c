/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "cronometro.h"
#include "temporizador.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY 100 // Atraso em milissegundos
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t byte_buffer_laps[25];
uint32_t lastDebounceTime = 0, adc_value = 0, decimos_segundo = 0,segundos = 0, dim_value = 60000, decimos_segundo_temp = 0,decimos_segundo_temp_pretendidos = 0;
int ds = 0, minutes = 0, seconds = 0, remaining_ds = 0, voltas_counter = 0,cursor_volta = 8,voltas_array[256];
bool flag_on = false, flag_temp_mode = false, flag_tx = false, flag_crescente = false, flag_first_cycle = true, flag_tempo_concluido = false, flag_send_laps = false;
TIM_OC_InitTypeDef sConfigOC = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int __io_getchar(void)
{
    uint8_t ch=0;
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == BOT_A_Pin){			//START + STOP
		uint32_t currentMillis = HAL_GetTick();
		if (currentMillis - lastDebounceTime >= DEBOUNCE_DELAY){

		if(!flag_temp_mode){			//MODO CRONOMETRO
			if(!flag_on){
				start_cronometro();
			} else {
				cursor_volta = 8;
				stop_cronometro();
				flag_send_laps = true;
			}
		} else {						//MODO TEMPORIZADOR
			if(!flag_on){		        //temporizador desligado
				if(flag_first_cycle){
					if(flag_crescente){
						decimos_segundo_temp = 0;
					}
					else{
						decimos_segundo_temp = 0;
						decimos_segundo_temp = decimos_segundo_temp_pretendidos;

					}
					HAL_ADC_Stop(&hadc1);
				}
				flag_first_cycle = false;
				flag_tempo_concluido = false;
				start_timer();
			} else {
				stop_timer();
			}
		}

		 lastDebounceTime = currentMillis;
		}
	}
	else if(GPIO_Pin == BOT_B_Pin){		//LAP + RESET
		uint32_t currentMillis = HAL_GetTick();
		if (currentMillis - lastDebounceTime >= DEBOUNCE_DELAY){

		if(!flag_temp_mode){			//MODO CRONOMETRO
			if(flag_on){
				voltas_counter++;
				lap_detected();
			} else {
				reset_cronometro();
				print_tempo();
			}
		} else {						//MODO TEMPORIZADOR
			if(!flag_on){
				if(!flag_first_cycle || flag_tempo_concluido){
					flag_tempo_concluido = false;
					reset_timer();
					flag_first_cycle = true;
				} else {
					flag_crescente = !flag_crescente;
				}
			}
		}

		lastDebounceTime = currentMillis;
	}
}
	else if(GPIO_Pin == SWITCH_ON_OFF_Pin){
		uint32_t currentMillis = HAL_GetTick();
		if (currentMillis - lastDebounceTime >= DEBOUNCE_DELAY){

		if(!flag_on){
			flag_temp_mode = !flag_temp_mode;
				if(!flag_temp_mode){
					print_tempo();
				}
				if(!flag_first_cycle && flag_temp_mode){
					print_temp();
				}
			}

		lastDebounceTime = currentMillis;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim == &htim2){
		decimos_segundo++;

		  if(flag_on){
			  print_tempo();
		  }

	} else if(htim == &htim3){
		//desligar o LED apos 1/2s
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
		HAL_TIM_Base_Stop(&htim3);

	} else if(htim == &htim5){
		dim_value -= 10000;

		 if(dim_value <= 0) {
			  dim_value = 60000;
		  }

		 else if(flag_on){
			 //config DIM Red LED
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);

				sConfigOC.OCMode = TIM_OCMODE_PWM1;
				sConfigOC.Pulse = dim_value;
				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
				sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

				HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
				HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
		 }
	} else if(htim == &htim7){
		if(flag_crescente){					///MODO CRESCENTE

			decimos_segundo_temp++;

			if(flag_on){
				print_temp();
			}
			if(decimos_segundo_temp == decimos_segundo_temp_pretendidos){
				decimos_segundo_temp = 0;
				flag_tempo_concluido = true;
				stop_timer();
				printf("TEMPO ATINGIDO\r\n");
			}

		} else {							//MODO DECRESCENTE

			decimos_segundo_temp--;

			if(flag_on){
				print_temp();
			}
			if(decimos_segundo_temp == 0){
				decimos_segundo_temp = decimos_segundo_temp_pretendidos;
				flag_tempo_concluido = true;
				stop_timer();
				printf("TEMPO ATINGIDO\r\n");
			}

		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    //usar uma flag na interrupt

    if(huart == &huart3){
        flag_tx = true;
    }
}

void reset_LEDS(){
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);    //Green LED off
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);    //Blue LED off
}
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  reset_cronometro();
  reset_timer();
  print_tempo();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  reset_cronometro();
  reset_timer();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


      if(flag_temp_mode){			//MODO TEMPORIZADOR

          HAL_GPIO_WritePin(LED_TEMP_GPIO_Port, LED_TEMP_Pin, GPIO_PIN_SET);    //LED TEMPORIZADOR ON
          HAL_GPIO_WritePin(LED_CRONO_GPIO_Port, LED_CRONO_Pin, GPIO_PIN_RESET);//LED CRONOMETRO OFF

          if(flag_first_cycle && !flag_on){
        	  reset_LEDS();
        	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);						//Red LED off
        	  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);        //Green LED on -> desligado
          }
          if(flag_on){
        	  reset_LEDS();
        	  HAL_TIM_Base_Start_IT(&htim5);									//Red LED Dim -> contar
          }
          if(!flag_first_cycle && !flag_on){
        	  reset_LEDS();
        	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);						//Red LED off
        	  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);    //Blue LED on -> pause
          }

          while(flag_first_cycle && flag_temp_mode){					//LEITURA ADC PARA DEFINIR TEMPORIZADOR
        	  HAL_ADC_Start(&hadc1);
        	  adc_value = HAL_ADC_GetValue(&hadc1);
        	  HAL_ADC_Stop(&hadc1);
        	  print_set_time();
        	  HAL_Delay(50);
          }


      } else {						//MODO CRONOMETRO

          HAL_GPIO_WritePin(LED_TEMP_GPIO_Port, LED_TEMP_Pin, GPIO_PIN_RESET);    //LED TEMPORIZADOR OFF
          HAL_GPIO_WritePin(LED_CRONO_GPIO_Port, LED_CRONO_Pin, GPIO_PIN_SET);    //LED CRONOMETRO ON


          if(flag_on){
              HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);    //Green LED off -> contar
              HAL_TIM_Base_Start_IT(&htim5);									//Red LED Dim -> contar

          } else {
              HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
              HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);        //Green LED on -> desligado
              HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);    //Blue LED off
          }

          if(flag_send_laps){
        	  print_lap();
              flag_send_laps = false;

          }
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
