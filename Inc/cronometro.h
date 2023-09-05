/*
 * cronometro.h
 *
 *  Created on: Apr 28, 2023
 *      Author: TESTER
 */

#ifndef INC_CRONOMETRO_H_
#define INC_CRONOMETRO_H_
#include <stdio.h>
#include <stdbool.h>

extern uint8_t byte_buffer_laps[25];
extern bool flag_on, flag_tx;
extern uint32_t decimos_segundo;
extern int ds, minutes, seconds, remaining_ds, voltas_counter,cursor_volta,voltas_array[256];

void start_cronometro();

void stop_cronometro();

void lap_detected();

void reset_cronometro();

void print_tempo();

void print_lap();

#endif /* INC_CRONOMETRO_H_ */
