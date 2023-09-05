#ifndef TEMPORIZADOR_H
#define TEMPORIZADOR_H
#include <stdio.h>
#include <stdbool.h>

extern bool flag_on, modo_temp, flag_crescente, flag_tx;
extern uint32_t decimos_segundo_temp, adc_value, decimos_segundo_temp_pretendidos;
extern int ds, minutes, seconds, remaining_ds, voltas_counter, cursor_volta,voltas_array[256];

void start_timer();

void stop_timer();

void reset_timer();

void convert_ADC();

void print_set_time();

void print_temp();


#endif
