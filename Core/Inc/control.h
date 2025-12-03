/*
 * control.h
 *
 *  Created on: Oct 26, 2025
 *      Author: juli
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "cmsis_os.h"

#include "mt6835.h"

#define M 				15
#define N 				16
#define FX_MAX 			(1 << M) - 1
#define FX_MIN 	   		-(1 << M)
#define PP				4			// Pares de polos
#define LAMBDA			1			// Flujo concatenado

typedef struct {
	// Lazo posicion
	int32_t fx_P;
	int32_t fx_I;
	int32_t fx_D;
	int32_t fx_prev_position;
	int32_t fx_prev_position_error;
	int32_t fx_proporcional;
	int32_t fx_integral;
	int32_t fx_derivativo;

	int32_t fx_dt;

	// Filtro LP
	int32_t fx_tau;

	// Lazo corriente
	int32_t fx_Pq;
	int32_t fx_Pd;
	int32_t fx_consigna_torque;
	int32_t fx_consigna_iq;
	int32_t fx_consigna_vq;
	int32_t fx_consigna_vd;
	int32_t fx_corrienteA; 	// Guardo corrientes en n-1 para filtro IIR
	int32_t fx_corrienteB;

	// Saturacion de la salida
	int32_t fx_maxOut;
	int32_t fx_minOut;

	// Output clamping
	int32_t fx_pid_max;
} controlador_t;

typedef struct interpolador_t {
	int32_t fx_t0;
	int32_t fx_t_act;
	int32_t fx_dt;
	int32_t fx_posicion_inicial;
	int32_t fx_posicion_final;
	int32_t fx_distancia_total;
	int32_t fx_consigna_posicion;
	int32_t fx_a_max;
	int32_t fx_v_max;
	int32_t fx_t_acc;
	int32_t fx_t_const;
	int32_t fx_t_total;
	int8_t nueva;
} interpolador_t;

void init_controlador(controlador_t *controlador);
void init_interpolador(interpolador_t *interpolador);
int32_t fp2fx(double numero);
void lazo_corriente();
void lazo_posicion();
int32_t get_corrientes_qd0(int32_t *corrientes_qd0);
void interpolar();
void set_posicion_final_nueva(int32_t fx_posicion_final);
uint32_t mi_sqrt(uint64_t numero);
int32_t get_fx_position();
HAL_StatusTypeDef alinear_rotor();
void init_lazos_control();
void deinit_lazos_control();
void get_Ld();
void get_Lq();

extern volatile int32_t overflow_encoder;
extern uint32_t lecturas_adcs;
extern controlador_t controlador;
extern interpolador_t interpolador;
extern int32_t FX_ALPHA;

#endif /* INC_CONTROL_H_ */
