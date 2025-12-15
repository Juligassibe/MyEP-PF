/*
 * control.c
 *
 *  Created on: Oct 26, 2025
 *      Author: juli
 */

#include "control.h"

int32_t FX_LAMBDA;
int32_t FX_ADC_CONST;
int32_t FX_360;
int32_t FX_2_3;
int32_t FX_1_2;
int32_t FX_KT;
int32_t FX_1_KT;
int32_t FX_Lq;
int32_t FX_Ld;
int32_t FX_Rs;
int32_t FX_1_dt;
int32_t FX_V_CC;
int32_t FX_I_MAX;
int32_t FX_I_MIN;
int32_t FX_ALPHA;
volatile int32_t fx_debug_pos;
int32_t fx_t_acc;
int32_t fx_distancia_acc;


volatile int32_t overflow_encoder = 0;
uint32_t lecturas_adcs = 0;
controlador_t controlador = {0};
interpolador_t interpolador = {0};

// Medio ciclo de coseno
const int32_t COS[360] = {
	65536 , 65533 , 65525 , 65513 , 65495 , 65473 , 65445 , 65413 , 65375 , 65332 , 65285 , 65232 , 65174 , 65112 , 65044 , 64972 , 64894 , 64812 , 64724 , 64632 ,
	64534 , 64432 , 64325 , 64213 , 64095 , 63973 , 63846 , 63715 , 63578 , 63436 , 63290 , 63139 , 62983 , 62822 , 62656 , 62486 , 62310 , 62130 , 61945 , 61756 ,
	61561 , 61362 , 61159 , 60950 , 60737 , 60519 , 60297 , 60070 , 59838 , 59602 , 59362 , 59116 , 58867 , 58612 , 58353 , 58090 , 57822 , 57550 , 57274 , 56993 ,
	56707 , 56418 , 56124 , 55826 , 55523 , 55216 , 54905 , 54590 , 54271 , 53947 , 53619 , 53288 , 52952 , 52612 , 52268 , 51920 , 51568 , 51212 , 50852 , 50489 ,
	50121 , 49750 , 49374 , 48995 , 48613 , 48226 , 47836 , 47442 , 47045 , 46644 , 46239 , 45831 , 45419 , 45004 , 44585 , 44163 , 43738 , 43309 , 42877 , 42442 ,
	42003 , 41561 , 41116 , 40668 , 40217 , 39762 , 39305 , 38845 , 38381 , 37915 , 37446 , 36974 , 36499 , 36021 , 35541 , 35057 , 34571 , 34083 , 33592 , 33098 ,
	32602 , 32103 , 31602 , 31098 , 30592 , 30084 , 29573 , 29060 , 28545 , 28028 , 27508 , 26987 , 26463 , 25937 , 25410 , 24880 , 24349 , 23815 , 23280 , 22743 ,
	22204 , 21664 , 21122 , 20578 , 20033 , 19486 , 18938 , 18388 , 17837 , 17284 , 16731 , 16175 , 15619 , 15061 , 14503 , 13943 , 13382 , 12820 , 12257 , 11693 ,
	11129 , 10563 , 9997  , 9429  , 8862  , 8293  , 7724  , 7154  , 6584  , 6013  , 5442  , 4870  , 4298  , 3725  , 3153  , 2580  , 2006  , 1433  , 860   , 286   ,
	-286  , -860  , -1433 , -2006 , -2580 , -3153 , -3725 , -4298 , -4870 , -5442 , -6013 , -6584 , -7154 , -7724 , -8293 , -8862 , -9429 , -9997 , -10563, -11129,
	-11693, -12257, -12820, -13382, -13943, -14503, -15061, -15619, -16175, -16731, -17284, -17837, -18388, -18938, -19486, -20033, -20578, -21122, -21664, -22204,
	-22743, -23280, -23815, -24349, -24880, -25410, -25937, -26463, -26987, -27508, -28028, -28545, -29060, -29573, -30084, -30592, -31098, -31602, -32103, -32602,
	-33098, -33592, -34083, -34571, -35057, -35541, -36021, -36499, -36974, -37446, -37915, -38381, -38845, -39305, -39762, -40217, -40668, -41116, -41561, -42003,
	-42442, -42877, -43309, -43738, -44163, -44585, -45004, -45419, -45831, -46239, -46644, -47045, -47442, -47836, -48226, -48613, -48995, -49374, -49750, -50121,
	-50489, -50852, -51212, -51568, -51920, -52268, -52612, -52952, -53288, -53619, -53947, -54271, -54590, -54905, -55216, -55523, -55826, -56124, -56418, -56707,
	-56993, -57274, -57550, -57822, -58090, -58353, -58612, -58867, -59116, -59362, -59602, -59838, -60070, -60297, -60519, -60737, -60950, -61159, -61362, -61561,
	-61756, -61945, -62130, -62310, -62486, -62656, -62822, -62983, -63139, -63290, -63436, -63578, -63715, -63846, -63973, -64095, -64213, -64325, -64432, -64534,
	-64632, -64724, -64812, -64894, -64972, -65044, -65112, -65174, -65232, -65285, -65332, -65375, -65413, -65445, -65473, -65495, -65513, -65525, -65533, -65536
};

void init_controlador(controlador_t *controlador) {
	controlador->fx_P = fp2fx(PID_P);
	controlador->fx_I = fp2fx(PID_I);
	controlador->fx_D = fp2fx(PID_D);
	controlador->fx_prev_position = 0;
	controlador->fx_prev_position_error = 0;
	controlador->fx_proporcional = 0;
	controlador->fx_integral = 0;
	controlador->fx_derivativo = 0;
	controlador->fx_wm = 0;

	controlador->fx_dt = fp2fx(0.001);

	controlador->fx_consigna_torque = 0;
	controlador->fx_consigna_iq = 0;
	controlador->fx_consigna_vq = 0;
	controlador->fx_consigna_vd = 0;
	controlador->fx_corrienteA = 0;
	controlador->fx_corrienteB = 0;

	// Inicio constantes globales
	FX_LAMBDA = fp2fx((double)LAMBDA);
	FX_ADC_CONST = fp2fx(ADC_CONST);
	FX_360 = fp2fx(360.0);
	FX_2_3 = fp2fx(2.0/3);
	FX_1_2 = fp2fx(0.5);
	FX_KT = fp2fx(1.5 * PP * LAMBDA);
	FX_1_KT = fp2fx(1.0/(1.5 * PP * LAMBDA));
	FX_Lq = 20;
	FX_Ld = 15;
	FX_Rs = fp2fx(0.8);
	FX_1_dt = fp2fx(1000.0);
	FX_V_CC = fp2fx(12.0);
	FX_I_MAX = fp2fx(3.0);
	FX_I_MIN = fp2fx(-3.0);
	FX_ALPHA = fp2fx(0.03);								// alpha = 1 - lambda en el filtro IIR (no confundir con lambda de flujo concatenado)
	fx_t_acc = fp2fx(V_MAX / A_MAX);					// t_acc = v_max / a_max = 0.24 (15728 en Q15.16)
	fx_distancia_acc = fp2fx(2*0.5*V_MAX*V_MAX/A_MAX);	// d_acc = 2 * 0.5 * a_max * t_acc^2 = 18 inicialmente (1179648 en Q15.16)

	controlador->fx_pid_max = ((int64_t)FX_I_MAX * FX_KT) >> N;
	controlador->fx_Pq = POLO_CORRIENTE * FX_Lq;
	controlador->fx_Pd = POLO_CORRIENTE * FX_Ld;
}

void init_interpolador(interpolador_t *interpolador) {
	interpolador->fx_t0 = 0;
	interpolador->fx_t_act = 0;
	interpolador->fx_dt = fp2fx(0.001);
	interpolador->fx_posicion_inicial = 0;
	interpolador->fx_posicion_final = 0;
	interpolador->fx_distancia_total = 0;
	interpolador->fx_consigna_posicion = 0;
	interpolador->fx_a_max = fp2fx(A_MAX);
	interpolador->fx_v_max = fp2fx(V_MAX);
	interpolador->fx_t_acc = 0;
	interpolador->fx_t_const = 0;
	interpolador->fx_t_total = 0;
	interpolador->nueva = 0;
}

int32_t fp2fx(double numero) {
	// Si el numero a convertir no esta en el rango [-2^M : 2^M-1] saturo
	if (numero > FX_MAX) {
		const char cadena[] = "W: fp2fx fuera de rango positivo\n";
		HAL_UART_Transmit(&huart1, (uint8_t *)cadena, sizeof(cadena), 1000);
		return INT32_MAX;
	}
	else if (numero < FX_MIN) {
		const char cadena[] = "W: fp2fx fuera de rango negativo\n";
		HAL_UART_Transmit(&huart1, (uint8_t *)cadena, sizeof(cadena), 1000);
		return INT32_MIN;
	}


	// Primero escalo
	double temp = numero * (1 << N);

	// Redondeo a entero mas cercano
	if (temp >= 0) {
		temp += 0.5;
	}
	else {
		temp -= 0.5;
	}

	return (int32_t)temp;
}

void lazo_corriente() {
	// Solo calculo iq e id ya que con neutro flotante i0 = 0 siempre
	int32_t fx_corrientes_qd0[2] = {0};

	// Devuelve el indice para sin(tita) para no recalcular indices de LUT para T. Inv. Park
	int32_t LUT_index_sin = get_corrientes_qd0(fx_corrientes_qd0);
	int32_t LUT_index_cos = LUT_index_sin + 180;

	if (LUT_index_cos > 719) {
		LUT_index_cos -= 720;
	}

	// LAZO DE CONTROL DE CORRIENTE P

	// iq*[n] = T*[n] / Kt
	controlador.fx_consigna_iq = ((int64_t)controlador.fx_consigna_torque * FX_1_KT) >> N;

	int32_t fx_error_iq = controlador.fx_consigna_iq - fx_corrientes_qd0[0];

	// vq*[n] = Pq * eq[n] + caida ohmica + desacople id + caida BEMF
	controlador.fx_consigna_vq = (((int64_t)controlador.fx_Pq * fx_error_iq) >> N) +
								 (((int64_t)FX_Rs * fx_corrientes_qd0[0]) >> N) +
								 PP * (((((int64_t)controlador.fx_wm * FX_Ld) >> N) * fx_corrientes_qd0[1]) >> N) +
								 PP * (((int64_t)FX_LAMBDA * controlador.fx_wm) >> N);

	/*
	 * vd*[n] = Pd * (-id[n]) + caida ohmica + desacople iq
	 * ed[n] = -id[n] ya que la consigna de id es 0
	 */

	controlador.fx_consigna_vd = -PP * (((((int64_t)controlador.fx_wm * FX_Lq) >> N) * fx_corrientes_qd0[0]) >> N);

	// CHECKEAR QUE LAS CUENTAS SEAN VALIDAS
	// T.inv. Park
	// va = vq * -sin(tita) + vd * cos(tita)
	int32_t fx_consigna_va = ((-(int64_t)controlador.fx_consigna_vq * COS[(LUT_index_sin > 359) ? (719 - LUT_index_sin) : (LUT_index_sin)]) >> N) +
							  (((int64_t)controlador.fx_consigna_vd * COS[(LUT_index_cos > 359) ? (719 - LUT_index_cos) : (LUT_index_cos)]) >> N);

	// Adelantar 240 es igual que atrasar 120
	LUT_index_cos += 480;
	LUT_index_sin += 480;

	if (LUT_index_cos > 719) {
		LUT_index_cos -= 720;
	}

	if (LUT_index_sin > 719) {
		LUT_index_sin -= 720;
	}
	// vb = vq * -sin(tita-2pi/3) + vd * cos(tita-2pi/3)
	int32_t fx_consigna_vb = ((-(int64_t)controlador.fx_consigna_vq * COS[(LUT_index_sin > 359) ? (719 - LUT_index_sin) : (LUT_index_sin)]) >> N) +
			 	 	 	 	  (((int64_t)controlador.fx_consigna_vd * COS[(LUT_index_cos > 359) ? (719 - LUT_index_cos) : (LUT_index_cos)]) >> N);

	// Adelantar 240 es igual que atrasar 120
	LUT_index_cos += 480;
	LUT_index_sin += 480;

	if (LUT_index_cos > 719) {
		LUT_index_cos -= 720;
	}

	if (LUT_index_sin > 719) {
		LUT_index_sin -= 720;
	}
	// vc = vq * -sin(tita+2pi/3) + vd * cos(tita+2pi/3)
	int32_t fx_consigna_vc = ((-(int64_t)controlador.fx_consigna_vq * COS[(LUT_index_sin > 359) ? (719 - LUT_index_sin) : (LUT_index_sin)]) >> N) +
			 	 	 	 	  (((int64_t)controlador.fx_consigna_vd * COS[(LUT_index_cos > 359) ? (719 - LUT_index_cos) : (LUT_index_cos)]) >> N);

	// Hago SPWM
	int32_t fx_duty_a = (FX_1_2 + ((int64_t)fx_consigna_va << N) / FX_V_CC);
	int32_t fx_duty_b = (FX_1_2 + ((int64_t)fx_consigna_vb << N) / FX_V_CC);
	int32_t fx_duty_c = (FX_1_2 + ((int64_t)fx_consigna_vc << N) / FX_V_CC);

	htim3.Instance->CCR1 = fx_duty_a * htim3.Instance->ARR / (1 << N);
	htim3.Instance->CCR2 = fx_duty_b * htim3.Instance->ARR / (1 << N);
	htim3.Instance->CCR3 = fx_duty_c * htim3.Instance->ARR / (1 << N);
}

void lazo_posicion() {
	int32_t fx_position = get_fx_position();

	// Velocidad del rotor
	controlador.fx_wm = ((int64_t)(fx_position - controlador.fx_prev_position) * FX_1_dt) >> N;

	// LAZO CONTROL DE POSICION PI + D

	// Genero nueva consigna de posicion
	interpolar();

	int32_t fx_error_pos = interpolador.fx_consigna_posicion - fx_position;

	// P[n] = Kp * e[n]
	controlador.fx_proporcional = ((int64_t)controlador.fx_P * fx_error_pos) >> N;

	// I[n] = Ki * dt * (e[n] + e[n-1]) / 2 + I[n-1]
	int64_t tempI = ((int64_t)controlador.fx_I * controlador.fx_dt) >> N;
	tempI = (tempI * (fx_error_pos + controlador.fx_prev_position_error)) >> N;
	controlador.fx_integral += (int32_t)(tempI >> 1);

	// D[n] = Kd * (-wm) (No hago parte derivativa con el error por ser PI + D)
	controlador.fx_derivativo = ((int64_t)controlador.fx_D * (-controlador.fx_wm)) >> N;

	// Antiwindup del integrador
	int32_t fx_integral_max, fx_integral_min;

	// Limite superior del integrador
	if (controlador.fx_pid_max > controlador.fx_proporcional + controlador.fx_derivativo) {
		fx_integral_max = controlador.fx_pid_max - controlador.fx_proporcional - controlador.fx_derivativo;
	}
	else {
		fx_integral_max = 0;
	}
	// Limite inferior del integrador
	if (-controlador.fx_pid_max < controlador.fx_proporcional + controlador.fx_derivativo) {
		fx_integral_min = -controlador.fx_pid_max - controlador.fx_proporcional - controlador.fx_derivativo;
	}
	else {
		fx_integral_min = 0;
	}

	if (controlador.fx_integral > fx_integral_max) {
		controlador.fx_integral = fx_integral_max;
	}
	else if (controlador.fx_integral < fx_integral_min) {
		controlador.fx_integral = fx_integral_min;
	}

	// T*[n] = P[n] + I[n] + D[n]
	controlador.fx_consigna_torque = controlador.fx_proporcional + controlador.fx_integral + controlador.fx_derivativo;

	// Clamping de la salida por si hay saturacion por accion P o D
	if (controlador.fx_consigna_torque > controlador.fx_pid_max) {
		controlador.fx_consigna_torque = controlador.fx_pid_max;
	}
	else if (controlador.fx_consigna_torque < -controlador.fx_pid_max) {
		controlador.fx_consigna_torque = -controlador.fx_pid_max;
	}

	// Actualizo valores previos
	controlador.fx_prev_position = fx_position;
	controlador.fx_prev_position_error = fx_error_pos;
}

int32_t get_corrientes_qd0(int32_t *corrientes) {
	int32_t fx_rotor_position = get_fx_position();

	uint16_t lectura_faseA = (uint16_t)(lecturas_adcs & 0xFFFC) + adc_offsets[0];
	uint16_t lectura_faseB = (uint16_t)((lecturas_adcs >> 16) & 0xFFFC) + adc_offsets[1];

//	lectura_faseA &= 0xFFFC;
//	lectura_faseB &= 0xFFFC;

	// La lectura nueva del ADC seria xn en el filtro IIR
	int32_t fx_xnA = FX_ADC_CONST * (lectura_faseA - ADC_0A);
	int32_t fx_xnB = FX_ADC_CONST * (lectura_faseB - ADC_0A);

	// Filtro IIR
	controlador.fx_corrienteA = controlador.fx_corrienteA + (((int64_t)FX_ALPHA * (fx_xnA - controlador.fx_corrienteA)) >> N);
	controlador.fx_corrienteB = controlador.fx_corrienteB + (((int64_t)FX_ALPHA * (fx_xnB - controlador.fx_corrienteB)) >> N);
//	char cadena[16];
//	int len;
//
//	len = snprintf(cadena, sizeof(cadena), "%ld\n", controlador.fx_corrienteA);
//	HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);

	int32_t fx_corrienteC = -1 * (controlador.fx_corrienteA + controlador.fx_corrienteB);

	if (controlador.fx_corrienteA > FX_I_MAX || controlador.fx_corrienteA < FX_I_MIN ||
		controlador.fx_corrienteB > FX_I_MAX || controlador.fx_corrienteB < FX_I_MIN ||
					fx_corrienteC > FX_I_MAX || 			fx_corrienteC < FX_I_MIN) {
		mensaje_t mensaje = {
			.estado = FAULT,
			.origen = CORRIENTES
		};

		BaseType_t tarea_mayor_prioridad = pdFALSE;
		xQueueSendFromISR(cola_estados, &mensaje, &tarea_mayor_prioridad);
		portYIELD_FROM_ISR(tarea_mayor_prioridad);
	}

	/*
	 * La T. Park la calculo con el angulo electrico Tita_e = Pp * Tita_r
	 * Como PP = 4, es igual a hacer << 2
	 * Multiplico por 2 por que el elemento 360 de la LUT es cos(180)
	 */
	int32_t fx_LUT_index = ((fx_rotor_position << 2) % FX_360) * 2;
//	float fp_LUT_index = ((float)fx_LUT_index / (1 << N));

	// Redondeo al entero mas cercano, sumo 0.5 en Q15.16
	if (fx_LUT_index > 0){
		fx_LUT_index += 32768;
	}
	else{
		fx_LUT_index -= 32768;
	}

	int16_t LUT_index1 = (int16_t)(fx_LUT_index / (1 << N));

	// Llevo a angulo positivo
	if (LUT_index1 < 0) {
		LUT_index1 += 720;
	}

	// Atrasar 120 es igual que adelantar 240
	int16_t LUT_index2 = LUT_index1 + 480;
	// Adelanto 120
	int16_t LUT_index3 = LUT_index1 + 239;

	if (LUT_index2 > 719) {
		LUT_index2 -= 720;
	}

	if (LUT_index3 > 719) {
		LUT_index3 -= 720;
	}

	// Hago T. de Park para obtener corrientes qd0
	// id = 2/3 * (ia * cos(tita) + ib * cos(tita - 2/3 pi) + ic * cos(tita + 2/3 pi)
	corrientes[1] = (int32_t)(((int64_t)FX_2_3 * ((((int64_t)controlador.fx_corrienteA * COS[(LUT_index1 > 359) ? (719 - LUT_index1) : (LUT_index1)]) >> N) +
												  (((int64_t)controlador.fx_corrienteB * COS[(LUT_index2 > 359) ? (719 - LUT_index2) : (LUT_index2)]) >> N) +
															  (((int64_t)fx_corrienteC * COS[(LUT_index3 > 359) ? (719 - LUT_index3) : (LUT_index3)]) >> N))) >> N);

	// Para el seno solo atraso 90 grados en la lut (180 elementos)
	LUT_index1 -= 180;

	// Llevo a angulo positivo
	if (LUT_index1 < 0) {
		LUT_index1 += 720;
	}

	// Atrasar 120 es igual que adelantar 240
	LUT_index2 = LUT_index1 + 480;
	// Adelanto 120
	LUT_index3 = LUT_index1 + 239;

	if (LUT_index2 > 719) {
		LUT_index2 -= 720;
	}

	if (LUT_index3 > 719) {
		LUT_index3 -= 720;
	}

	// iq = -2/3 * (ia * sin(tita) + ib * sin(tita - 2/3 pi) + ic * sin(tita + 2/3 pi)
	corrientes[0] = -1 * (int32_t)(((int64_t)FX_2_3 * ((((int64_t)controlador.fx_corrienteA * COS[(LUT_index1 > 359) ? (719 - LUT_index1) : (LUT_index1)]) >> N) +
													   (((int64_t)controlador.fx_corrienteB * COS[(LUT_index2 > 359) ? (719 - LUT_index2) : (LUT_index2)]) >> N) +
																   (((int64_t)fx_corrienteC * COS[(LUT_index3 > 359) ? (719 - LUT_index3) : (LUT_index3)]) >> N))) >> N);

	// Retorno indice de LUT para no recalcular en T. inv. Park
	return LUT_index1;
}

void interpolar() {
	// Si hay consigna nueva modifico parametros del perfil
	if (interpolador.nueva) {
		interpolador.fx_posicion_inicial = get_fx_position();

		int32_t desplazamiento = interpolador.fx_posicion_final - interpolador.fx_posicion_inicial;
		interpolador.fx_distancia_total = desplazamiento > 0 ? desplazamiento : (-desplazamiento);

		// Perfil triangular de velocidad
		if (fx_distancia_acc >= interpolador.fx_distancia_total) {
			// Uso d_total por que despejo de d_acc = 0.5*a_max*t_acc^2 y 2*d_acc = d_total
			// t_acc = sqrt((d_acc * 2) / a_max))
			interpolador.fx_t_acc = mi_sqrt(((int64_t)interpolador.fx_distancia_total << N) / interpolador.fx_a_max) << 8;
			interpolador.fx_t_const = 0;
			interpolador.fx_v_max = (((int64_t)interpolador.fx_a_max * interpolador.fx_t_acc) >> N);
		}

		// Perfil trapezoidal de velocidad
		else {
			interpolador.fx_t_acc = fx_t_acc;
			interpolador.fx_v_max = fp2fx(V_MAX);
			// t_const = d_const / v_max = (d_total - d_acc) / v_max
			interpolador.fx_t_const = (((int64_t)(interpolador.fx_distancia_total - fx_distancia_acc)) << N) / interpolador.fx_v_max;
		}

		interpolador.fx_t0 = 0;
		interpolador.fx_t_act = 0;
		interpolador.fx_t_total = interpolador.fx_t_acc * 2 + interpolador.fx_t_const;
		interpolador.fx_v_max *= (desplazamiento < 0 ? (-1) : 1);
		interpolador.fx_a_max = (desplazamiento < 0 ? (fp2fx(-A_MAX)) : fp2fx(A_MAX));
		interpolador.nueva = 0;
	}

	// Comienzo a calcular perfil
	interpolador.fx_t_act += interpolador.fx_dt;

	// Aceleracion
	if (interpolador.fx_t_act <= interpolador.fx_t_acc) {
		interpolador.fx_consigna_posicion = interpolador.fx_posicion_inicial +
											(((((int64_t)(interpolador.fx_a_max >> 1) * interpolador.fx_t_act) >> N) * interpolador.fx_t_act) >> N);
	}

	// Velocidad constante
	else if (interpolador.fx_t_act <= interpolador.fx_t_acc + interpolador.fx_t_const) {
		interpolador.fx_consigna_posicion = interpolador.fx_posicion_inicial +
											(((((int64_t)(interpolador.fx_a_max >> 1) * interpolador.fx_t_acc) >> N) * interpolador.fx_t_acc) >> N) +
											(((int64_t)interpolador.fx_v_max * (interpolador.fx_t_act - interpolador.fx_t_acc)) >> N);
	}

	// Desaceleracion
	else if (interpolador.fx_t_act <= interpolador.fx_t_total){
		interpolador.fx_consigna_posicion = interpolador.fx_posicion_final -
											(((((int64_t)(interpolador.fx_a_max >> 1) * (interpolador.fx_t_total - interpolador.fx_t_act + interpolador.fx_dt)) >> N) *
																					    (interpolador.fx_t_total - interpolador.fx_t_act + interpolador.fx_dt)) >> N);
	}
}

void set_posicion_final_nueva() {
	HAL_UART_Transmit(&huart1, (uint8_t *)"Ej: 123.45\nENTER para cancelar\n", 31, 1000);
	xSemaphoreTake(semaforo_consola, osWaitForever);

	if (buffer_rx[0] == '\0') {
		HAL_UART_Transmit(&huart1, (uint8_t *)"Cancelado\n", 10, 1000);
		return;
	}

	char *endptr = NULL;
	float fp_nueva_consigna = strtod((char *)buffer_rx, &endptr);

	if (*endptr == buffer_rx[0]) {
		HAL_UART_Transmit(&huart1, (uint8_t *)"Consigna no valida\n", 19, 1000);
		return;
	}

	int32_t fx_nueva_consigna = fp2fx(fp_nueva_consigna);

	taskENTER_CRITICAL();
	interpolador.fx_posicion_final = fx_nueva_consigna;
	interpolador.fx_posicion_inicial = get_fx_position();
	interpolador.nueva = 1;
	taskEXIT_CRITICAL();
}

uint32_t mi_sqrt(uint64_t numero) {
	uint64_t rem = 0, root = 0;

	for (int i = 64 / 2; i > 0; i--) {
		root <<= 1;
		rem = (rem << 2) | (numero >> (64 - 2));
		numero <<= 2;
		if (root < rem) {
			rem -= root | 1;
			root += 2;
		}
	}
	return root >> 1;
}

int32_t get_fx_position() {
//	int32_t steps = (overflow_encoder < 0 ? (-65536) : (0)) + __HAL_TIM_GET_COUNTER(&htim2) +
//					(overflow_encoder < 0 ? (overflow_encoder + 1) : (overflow_encoder)) * 65536;
	int32_t steps = __HAL_TIM_GET_COUNTER(&htim2) - 32768;

	fx_debug_pos = fp2fx(steps * 0.15);

	return fx_debug_pos;
}

HAL_StatusTypeDef alinear_rotor() {
	// Alimento solo fase A para alinear eje d con fase A
	htim3.Instance->CCR1 = 500;

	// Doy tiempo al rotor para alinearse
	osDelay(pdMS_TO_TICKS(1000));

	// Pongo el 0 del encoder en esa posicion
	if (mt6835_set_zero(&hspi1, CS_MT6835_GPIO_Port, CS_MT6835_Pin) != MT6835_ZERO_OK) {
		return HAL_ERROR;
	}

	return HAL_OK;
}

void init_lazos_control() {
	// Habilito interrupciones para empezar a ejecutar lazos de control
	HAL_StatusTypeDef error;
	char cadena[16];
	int len;

	error = HAL_TIM_Base_Start_IT(&htim1);

	if (error != HAL_OK) {
		len = snprintf(cadena, sizeof(cadena), "E: Timer 1 (%d)\n", error);
		HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
		return;
	}

	__HAL_TIM_SET_COUNTER(&htim3, 0);
	error = HAL_TIM_Base_Start_IT(&htim3);

	if (error != HAL_OK) {
		HAL_TIM_Base_Stop_IT(&htim1);
		len = snprintf(cadena, sizeof(cadena), "E: Timer 3 (%d)\n", error);
		HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
		return;
	}

	// Para reinterpolar en caso de venir de haber detenido los lazos de control
	interpolador.nueva = 1;

	mensaje_t mensaje = {
		.estado = CLOSED_LOOP
	};

	xQueueSend(cola_estados, &mensaje, 1000);
}

void deinit_lazos_control() {
	// Deshabilito interrupciones para dejar de ejecutar lazos de control
	HAL_StatusTypeDef error;
	char cadena[16];
	int len;

	error = HAL_TIM_Base_Stop_IT(&htim1);

	if (error != HAL_OK) {
		len = snprintf(cadena, sizeof(cadena), "E: Timer 1 (%d)\n", error);
		HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
		return;
	}

	error = HAL_TIM_Base_Stop_IT(&htim3);

	if (error != HAL_OK) {
		len = snprintf(cadena, sizeof(cadena), "E: Timer 3 (%d)\n", error);
		HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
		return;
	}

	mensaje_t mensaje = {
		.estado = IDLE,
		.origen = CLI
	};

	xQueueSend(cola_estados, &mensaje, 1000);
}

void get_Ld() {
	if (estado_sistema != IDLE) {
		HAL_UART_Transmit(&huart1, (uint8_t *)"Poner sistema en IDLE\n", 22, 1000);
		return;
	}

	// Inicio TIM3 para disparar lecturas de ADCs
	HAL_TIM_Base_Start(&htim3);

	// Verifico offsets de ADCs
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	uint16_t lectura_faseA = (uint16_t)(lecturas_adcs & 0xFFFF) + adc_offsets[0];

	if (abs(FX_ADC_CONST * ((uint16_t)(lectura_faseA - 2048)) > 4000)) {
		HAL_UART_Transmit(&huart1, (uint8_t *)"Verificar ADC offsets\n", 22, 1000);
		return;
	}

	// Con el rotor libre alimento PWM a fase A y fases B y C a GND
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	// Doy tiempo a que se alinie el rotor y pasen los transitorios de corriente
	osDelay(pdMS_TO_TICKS(1000));

	// Como el rotor esta libre, ia = id
	lectura_faseA = (uint16_t)(lecturas_adcs & 0xFFFF) + adc_offsets[0];

	int32_t fx_id0 = FX_ADC_CONST * (lectura_faseA - 2048);
	int32_t fx_idf = fx_id0;
	int32_t FX_1_e = fp2fx(0.3679);

	// Luego de leer id(0) elimino la alimentacion y leo la corriente hasta el instante justo despues en que id(t) < id(0)

	// Inicio tim1 para llevar tiempo
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	tim1OF = 0;
	HAL_TIM_Base_Start_IT(&htim1);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	while (fx_idf > ((int64_t)FX_1_e * fx_id0) >> N) {
		lectura_faseA = (uint16_t)(lecturas_adcs & 0xFFFF) + adc_offsets[0];
		fx_idf = FX_ADC_CONST * (lectura_faseA - 2048);
	}

	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop(&htim3);

	int32_t fx_tau = fp2fx((double)(__HAL_TIM_GET_COUNTER(&htim1) + tim1OF * 36000) / 72000000.0);

	char cadena[64];
	int len;

	len = snprintf(cadena, sizeof(cadena), "%ld\n%ld\n%ld\n%ld (Q15.16)\n", fx_id0,
																			fx_idf,
																			fx_tau,
																			(int32_t)((int64_t)fx_tau * FX_Rs) >> N);
	HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
}

void get_Lq() {
	if (estado_sistema != IDLE) {
		HAL_UART_Transmit(&huart1, (uint8_t *)"Poner sistema en IDLE\n", 22, 1000);
		return;
	}

	// Inicio TIM3 para disparar lecturas de ADCs
	HAL_TIM_Base_Start(&htim3);

	// Verifico offsets de ADCs
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	uint16_t lectura_faseA = (uint16_t)(lecturas_adcs & 0xFFFF) + adc_offsets[0];

	if (abs(FX_ADC_CONST * ((uint16_t)(lectura_faseA - 2048)) > 4000)) {
		HAL_UART_Transmit(&huart1, (uint8_t *)"Verificar ADC offsets\n", 22, 1000);
		return;
	}

	// Con el rotor libre alimento PWM a fase A y fases B y C a GND
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	// Doy tiempo a que se alinie el rotor y pasen los transitorios de corriente
	osDelay(pdMS_TO_TICKS(1000));

	// Luego de alinear eje d con fase A, hay que dejar flotando la fase A y bloquear el rotor
	char cadena[64];
	int len;

	len = snprintf(cadena, sizeof(cadena), "Quitar EN fase A y bloquear rotor. ENTER para seguir\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);
	xSemaphoreTake(semaforo_consola, osWaitForever);

	// Alimento solo fase B con fase A flotando hace que id = 0 e iq = 1.155 ib
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	// Dejo que pasen los transitorios antes de medir
	osDelay(pdMS_TO_TICKS(1000));

	lectura_faseA = (uint16_t)(lecturas_adcs & 0xFFFF) + adc_offsets[0];

	int32_t fx_iq0 = FX_ADC_CONST * (lectura_faseA - 2048);
	int32_t fx_iqf = fx_iq0;
	int32_t FX_1_e = fp2fx(0.3679);

	// Luego de leer iq(0) elimino la alimentacion y leo la corriente hasta el instante justo despues en que iq(t) < iq(0)

	// Inicio tim1 para llevar tiempo
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	tim1OF = 0;
	HAL_TIM_Base_Start_IT(&htim1);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

	while (fx_iqf > ((int64_t)FX_1_e * fx_iq0) >> N) {
		lectura_faseA = (uint16_t)(lecturas_adcs & 0xFFFF) + adc_offsets[0];
		fx_iqf = FX_ADC_CONST * (lectura_faseA - 2048);
	}

	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop(&htim3);

	int32_t fx_tau = fp2fx((double)(__HAL_TIM_GET_COUNTER(&htim1) + tim1OF * 36000) / 72000000.0);

	len = snprintf(cadena, sizeof(cadena), "%ld\n%ld\n%ld\n%ld (Q15.16)\n", fx_iq0,
																			fx_iqf,
																			fx_tau,
																			(int32_t)((int64_t)fx_tau * FX_Rs) >> N);
	HAL_UART_Transmit(&huart1, (uint8_t *)cadena, len, 1000);

}

void modificar_constantes() {
	if (estado_sistema != IDLE) {
		HAL_UART_Transmit(&huart1, (uint8_t *)"Llevar el sistema a IDLE\n", 25, 1000);
		return;
	}
	HAL_UART_Transmit(&huart1, (uint8_t *)"\n>> ", 3, 1000);

	xSemaphoreTake(semaforo_consola, osWaitForever);

	char *endptr = NULL;
	float nueva = strtod((char *)&buffer_rx[1], &endptr);

	if (*endptr == buffer_rx[0]) {
		HAL_UART_Transmit(&huart1, (uint8_t *)"Constante no valida\n", 20, 1000);
		return;
	}

	switch (buffer_rx[0]) {
		case 'p':
		case 'P':
			controlador.fx_P = fp2fx(nueva);
			break;

		case 'i':
		case 'I':
			controlador.fx_I = fp2fx(nueva);
			break;

		case 'd':
		case 'D':
			controlador.fx_D = fp2fx(nueva);
			break;

		case 'q':
		case 'Q':
			controlador.fx_Pq = fp2fx(nueva);
			break;

		case 'e':
		case 'E':
			controlador.fx_Pd = fp2fx(nueva);
			break;
	}
}




