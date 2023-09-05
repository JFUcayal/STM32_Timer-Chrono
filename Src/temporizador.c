#include "temporizador.h"
#include <stdio.h>
#include <stdlib.h>
#include "tim.h"


void start_timer(){
    HAL_TIM_Base_Start_IT(&htim7);
    flag_on = true;
}

void stop_timer(){
    HAL_TIM_Base_Stop(&htim7);
    flag_on = false;
}

void reset_timer(){
    adc_value = 0;
    decimos_segundo_temp = 0;
}

void convert_ADC(){
	decimos_segundo_temp_pretendidos = (3000*adc_value)/3800;
}



void print_set_time(){

	convert_ADC();

	ds = decimos_segundo_temp_pretendidos;
	minutes = ds / 600;
	seconds = (ds / 10) % 60;
	remaining_ds = ds % 10;


	printf("\033[2J\033[H");    //clear console

	if(flag_crescente){
		printf("\033[1;1H---modo crescente---\r\n");
	}
	else{
		printf("\033[1;1H---modo decrescente---\r\n");
	}

	printf("\033[4;10H");        //move cursor
	printf("MODO TEMPORIZADOR->SET TIME\r\n");        //move cursor
	printf("\033[5;10H");        //move cursor
	printf("**************************\r\n");
	printf("\033[6;10H");        //move cursor
	printf("    %d min %d sec %d ds    \r\n", minutes, seconds, remaining_ds);
	printf("\033[7;10H");        //move cursor
	printf("**************************\r\n");
}

void print_temp(){

    ds = decimos_segundo_temp;
    minutes = ds / 600;
    seconds = (ds / 10) % 60;
    remaining_ds = ds % 10;

   printf("\033[2J\033[H");    //clear console
   printf("\033[4;10H");        //move cursor
   printf("    MODO TEMPORIZADOR\r\n");  //move cursor
   printf("\033[5;10H");        //move cursor
   printf("**************************\r\n");
   printf("\033[6;10H");        //move cursor
   printf("    %d min %d sec %d ds    \r\n", minutes, seconds, remaining_ds);
   printf("\033[7;10H");        //move cursor
   printf("**************************\r\n");

}
