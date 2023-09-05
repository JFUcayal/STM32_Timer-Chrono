/*
 * cronometro.c
 *
 *  Created on: Apr 28, 2023
 *      Author: TESTER
 */

#include "cronometro.h"
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "dma.h"
#include "tim.h"

void start_cronometro(){
	HAL_TIM_Base_Start_IT(&htim2);
	flag_on = true;
}

void stop_cronometro(){
	HAL_TIM_Base_Stop(&htim2);
	flag_on = false;
}

void reset_cronometro(){
	decimos_segundo = 0;
	cursor_volta = 8;
	voltas_array[256] = 0;
	voltas_counter = 0;
}

void lap_detected(){
	voltas_array[voltas_counter] = decimos_segundo;
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET); //ligar o LED
	HAL_TIM_Base_Start_IT(&htim3); 		//detetar 1/2 seg
}

void print_lap(){

	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);	//DESABILITAR AS INTERRUPTS DE GPIO PARA NAO INTERROMPER A IMPRESSAO DAS VOLTAS

	  printf("\033[2J\033[H");
      for (int i = 1; i < voltas_counter+1; i++) {

          int tempo_em_decimos = voltas_array[i];
          // Converter para minutos, segundos e décimos de segundo
          uint32_t minutos = tempo_em_decimos / 600;
          uint32_t segundos = (tempo_em_decimos / 10) % 60;
          uint32_t decimos_segundo = tempo_em_decimos % 10;

          // Formatar o tempo no buffer de bytesu
          snprintf((char*)byte_buffer_laps, sizeof(byte_buffer_laps), "Volta %d ---> %02lu:%02lu:%01lu\r\n", i, minutos, segundos, decimos_segundo);


          // Enviar o valor formatado por DMA
          HAL_UART_Transmit_DMA(&huart3, byte_buffer_laps, sizeof(byte_buffer_laps));


          // Aguardar a conclusão da transmissão
          while(flag_tx == false);
          flag_tx = false;
          HAL_Delay(1000);


      }
     HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);	//VOLTAR A HABILITAR AS INTERRUPTS DE GPIO
}


void print_tempo(){

		 ds = decimos_segundo;
	     minutes = ds / 600;
	     seconds = (ds / 10) % 60;
	     remaining_ds = ds % 10;

	    printf("\033[2J\033[H");	//clear console
	    printf("\033[4;10H");        //move cursor
	    printf("     MODO CRONOMETRO\r\n");  //move cursor
	    printf("\033[5;10H");		//move cursor
	    printf("**************************\r\n");
	    printf("\033[6;10H");		//move cursor
	    printf("    %d min %d sec %d ds    \r\n", minutes, seconds, remaining_ds);
	    printf("\033[7;10H");		//move cursor
	    printf("**************************\r\n");
}

