/* DriverLib Includes */
#include <ti/devices/msp432p4xx/inc/msp432p401r.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "Drivers_Control.h"
#include "mpu6050.h"
/* Global Variables */
uint16_t contador = 0; // Contador usado para alterar os leds

/* Flags */
uint_fast8_t status_flag_uart = 0;

/* Interruptions Handler */
void EUSCIA0_IRQHandler(void)
{
	UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
	status_flag_uart = 1; // Flag usada na função main
}


// PWM Configuration for Channel
dr_pwm_parameters PWM_0 = {
	.timer = TIMER_A0_BASE,
	.fast_mode = true,
	.timer_Prescaler = 4,
	.true_Sawtooth_not_triangular = true,
	.period_count = 60000, //60360 = 50 Hz(fechado, osciloscópio)
	.pwm_channel = 1,
	.outputmode = TIMER_A_OUTPUTMODE_RESET_SET
};
dr_pwm_parameters PWM_1 = {
	.timer = TIMER_A0_BASE,
	.fast_mode = true,
	.timer_Prescaler = 4,
	.true_Sawtooth_not_triangular = true,
	.period_count = 60000,	// 60000 = 50,33Hz (osciloscópio)
	.pwm_channel = 2,
	.outputmode = TIMER_A_OUTPUTMODE_RESET_SET
};
dr_pwm_parameters PWM_2 = {
	.timer = TIMER_A2_BASE,
	.fast_mode = true,
	.timer_Prescaler = 4,
	.true_Sawtooth_not_triangular = true,
	.period_count = 59990,
	.pwm_channel = 1,
	.outputmode = TIMER_A_OUTPUTMODE_RESET_SET
};

int main(void)
{

	WDT_A_holdTimer();
	/* Pin Config */
	DR_leds_sw_pin();
	DR_pwm_pin();
	DR_uart_pin();


	/* Peripherals Config */
	DR_uart_config(true);
	DR_pwm_config(&PWM_0);
	DR_pwm_config(&PWM_1);
	DR_pwm_config(&PWM_2);

	/* Inicializar Programas*/
	DR_leds_init();
	DR_uart_init();
	DR_pwm_init(&PWM_0, 30000);
	DR_pwm_init(&PWM_1, 30000);
	DR_pwm_init(&PWM_2, 30000);

	/* Interrupt Config */
	DR_uart_interrupt_receive();
	DR_interrupt_on();

		while (1)
	{
		DR_leds_alterar(contador);
		DR_delay_s(1); // deixar para ter alguma referência
		if (status_flag_uart)
		{
			contador = UART_receiveData(EUSCI_A0_BASE) - 48;
			printf("\n\n\rValor Leds em: %d", contador);
			status_flag_uart = 0;
			// Atualizar valores dos PWMs valores dos PWMs
			uint16_t duty_1 = DR_pwm_getDuty(&PWM_0);
			uint16_t duty_2 = DR_pwm_getDuty(&PWM_1);
			uint16_t duty_3 = DR_pwm_getDuty(&PWM_2);

			uint16_t period_timer_0 = DR_pwm_getPeriod(&PWM_0);
			uint16_t period_timer_2 = DR_pwm_getPeriod(&PWM_2);

			 if (contador == 1) // Incrementar o Duty_Cycle
			{
 				if (duty_1 < period_timer_0)
 				{
 					duty_1 += 10000;
					duty_2 += 10000;
					duty_3 += 10000;
 					DR_PWM_setDuty(&PWM_0, duty_1);
					DR_PWM_setDuty(&PWM_1, duty_2);
					DR_PWM_setDuty(&PWM_2, duty_3);
 				}
 			}	// end if contador == 1
			 if (contador == 2 )
			 { /* Decrementar Duty_Cycle*/
				if (duty_1 > 0){
					duty_1 -= 10000;
					duty_2 -= 10000;
					duty_3 -= 10000;
					DR_PWM_setDuty(&PWM_0, duty_1);
					DR_PWM_setDuty(&PWM_1, duty_2);
					DR_PWM_setDuty(&PWM_2, duty_3);
				}
			 } // end if contador == 2
			 if (contador == 3)
			 { /*PWM inforations */
			 	printf("\n\r CLK_freq :%d", CS_getSMCLK());
				printf("\n\r Pwm_fast_mode :%x", PWM_0.fast_mode);
			 	printf("\n\r Pwm_Prescaler :%d", PWM_0.timer_Prescaler);
			 	printf("\n\r Pwm_Sawtooth :%d", PWM_0.true_Sawtooth_not_triangular);
 			 	printf("\n\r Pwm_Duty1 :%d", duty_1);
				printf("\n\r Pwm_Duty2 :%d", duty_2);
				printf("\n\r Pwm_Duty3 :%d", duty_3);
 			 	printf("\n\r Pwm_Period0 :%d", period_timer_0);
				printf("\n\r Pwm_Period1 :%d", period_timer_2);
				printf("\n\r Pwm_freq1 :%.3fHz", DR_pwm_getfreq(&PWM_0));
				printf("\n\r Pwm_freq2 :%.3fHz", DR_pwm_getfreq(&PWM_1));
				printf("\n\r Pwm_freq3 :%.3fHz", DR_pwm_getfreq(&PWM_2));
				printf("\n\r Pwm_duty1 :%.2f%%", DR_pwm_getDuty_percent(&PWM_0));
				printf("\n\r Pwm_duty2 :%.2f%%", DR_pwm_getDuty_percent(&PWM_1));
				printf("\n\r Pwm_duty3 :%.2f%%", DR_pwm_getDuty_percent(&PWM_2));
			 }// end if contador == 3
			 
		}// end if status_flag 
	}// end while(1)
}// end int main()
