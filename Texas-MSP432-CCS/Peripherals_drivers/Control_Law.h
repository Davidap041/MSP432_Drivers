/*
 * Control_Law.h
 *
 *  Created on: 5 de mar de 2020
 *      Author: davia
 */

#ifndef CONTROL_LAW_H_
#define CONTROL_LAW_H_

typedef struct {
//	Variaveis de Controle PD
	int identification;	// link identification

	int dif_ruido;
	float estimativa_motor;
	float angulo_motor;

	float kp_pd;
	float kv_pd;
	float m;	
	float b;	
	float k;	
	float u;	// Control Signal
	float means;

	float erro_atual;
	float erro_derivadas;

	float pd_pp1;
	float pd_pp2;

	float Th; // Ângulo do servo atual
	float Th_anterior; // Ângulo do servo da amostra passada

	float dTh; 	// Diferencial do Ângulo do Servo
	float dTh_ref;
	float Th_ref; // Referências do Ângulo do Servo;
	float Th_ref_anterior; // Referências do Ângulo do Servo;

//	Variáveis do Saturador
	float upper_limit;
	float lower_limit;

} Ctrl_Law_angle_data;

/*Calculate Control Signal "u" by PD Control (look if the order is right!)*/
void Pd_Control_Law(Ctrl_Law_angle_data *link, float reference) {
//	Update Reference Value
	link->Th_ref = reference;
// Update and calculate Reference with delay
	link->dTh_ref = link->Th_ref - link->Th_ref_anterior;
// Store Reference Value for next interation
	link->Th_ref_anterior = link->Th_ref;
// Update and calculate dTh (Angle with delay)
	link->dTh = link->Th - link->Th_anterior;
//	Store Angle Value for next interation
	link->Th_anterior = link->Th;

// Calculate Signal Control u[k]
	link->erro_atual = (link->Th_ref - link->Th);  
	link->erro_derivadas = (link->dTh_ref - link->dTh);

	link->pd_pp1 = (link->k * link->Th) + (link->b * link->dTh);
	link->pd_pp2 = link->m
			* (link->kp_pd * link->erro_atual + link->kv_pd * link->erro_derivadas);

	link->u = link->pd_pp1 + link->pd_pp2 + link->means;

// Implementação dos saturadores (look if need store the true value of u[k])
	if (link->u > link->upper_limit) {
		link->u = link->upper_limit;
	}
	if (link->u < link->lower_limit) {
		link->u = link->lower_limit;
	}
}
/* Calculate Control Signal "u" by Open Loop Control (look if the order is right!)*/
void Open_Loop_Control_Law(Ctrl_Law_angle_data *link, float reference) {
// Update Reference Value
	link->Th_ref = reference;
// Set u with Reference Value	
	link->u = link->Th_ref;
}


#endif /* CONTROL_LAW_H_ */
