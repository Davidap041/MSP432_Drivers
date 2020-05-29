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
uint32_t status_flag_uart = 0;

/* Interruptions Handler */
void EUSCIA0_IRQHandler(void)
{
	UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
	status_flag_uart = 1; // Flag usada na função main
}

/*PWM functions test*/
float DR_pwm_getfreq(uint32_t timer, const dr_pwm_parameters *PWM)
{
	uint16_t PWM_period = TIMER_A0->CCR[0]; /* PWM period*/
	uint32_t PWM_clk = CS_getSMCLK(); /* Timer CLK*/
	/* Timer Prescaler 
	 PWM_prescaler = (TIMER_A0->TACL.ID*TIMER_A0->TACL.ID)* (TIMER_A0->TAEX0.TAIDEX + 1)
	 */
	/* Timer Period 
	 TIMER_A0->CCR[0] + 1
	 */
	return 0;
}

float DR_pwm_getDuty_percent(uint16_t Pwm_Channel)
{
	// dependendo do modo tem formas diferentes de calcular
	/* se for no modo reset/set */
	uint16_t PWM_period = TIMER_A_CMSIS(TIMER_A0_BASE)->CCR[0]; /* PWM period*/
	uint16_t PWM_set_period = TIMER_A_CMSIS(TIMER_A0_BASE)->CCR[Pwm_Channel]; /* Depende do modo */
	float duty_Cycle = PWM_set_period / PWM_period;

	return duty_Cycle;
}
void DR_pwm_pin(){
	/* PWM : Pin Config */
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
	GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.1
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5,
	GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.2
	// GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6,
	// GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.3
	// GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
	// GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.4
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN6,
	GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA2.1
}

dr_pwm_parameters PWM = { .fast_mode = true, .timer_Prescaler = 64,
							.true_Sawtooth_not_triangular = true,
							.period_count = 60000 };

int main(void)
{

	WDT_A_holdTimer();
	/* Pin Config */
	DR_leds_sw_pin();
	DR_pwm_pin();
	DR_uart_pin();


	/* Peripherals Config */
	DR_uart_config(true);
	DR_pwm_config(0,&PWM);
	DR_pwm_config(2,&PWM);

	/* Interrupt Config */
	DR_uart_interrupt_receive();
	DR_interrupt_on();
	
	/* Inicializar Programas*/
	DR_leds_init();
	DR_uart_init();
	
	// //	dr_PWM_init(60000,30000,64);
	// DR_pwm_init(TIMER_A0_BASE, 1, TIMER_A_OUTPUTMODE_RESET_SET, 30000);
	// DR_pwm_init(TIMER_A0_BASE, 2, TIMER_A_OUTPUTMODE_RESET_SET, 30000);
	// DR_pwm_init(TIMER_A2_BASE, 1, TIMER_A_OUTPUTMODE_RESET_SET, 30000);
	
		while (1)
	{
		DR_leds_alterar(contador);
		DR_delay_s(2); // deixar para ter alguma referência
		if (status_flag_uart)
		{
			contador = UART_receiveData(EUSCI_A0_BASE) - 48;
			printf("\n\n\rValor Leds em: %d", contador);
			status_flag_uart = 0;
			if (contador == 1) // Incrementar o Duty_Cycle
			{
// //				uint16_t PWM_period = DR_PWM_getperiod_count(TIMER_A2_BASE);
// //				uint16_t PWM_Duty = DR_PWM_getDuty(TIMER_A2_BASE, 1);
// 				if (PWM_Duty < PWM_period)
// 				{
// 					PWM_Duty += 1000;
// 					DR_PWM_setDuty(TIMER_A0_BASE, 1, PWM_Duty);
// 					DR_PWM_setDuty(TIMER_A0_BASE, 2, PWM_Duty);
// 					DR_PWM_setDuty(TIMER_A2_BASE, 1, PWM_Duty);
// 				}
			}
			if (contador == 2) // Decrementar o Duty Cycle
			{
				// uint16_t PWM_Duty = DR_PWM_getDuty(TIMER_A2_BASE, 1);
				// if (PWM_Duty > 0)
				// {
				// 	PWM_Duty -= 1000;
				// 	DR_PWM_setDuty(TIMER_A0_BASE, 1, PWM_Duty);
				// 	DR_PWM_setDuty(TIMER_A0_BASE, 2, PWM_Duty);
				// 	DR_PWM_setDuty(TIMER_A2_BASE, 1, PWM_Duty);
					
				// }
			}
			// if (contador == 3)
			// { /* PWM information */
			// 	printf("\n\r Pwm_Source :%x", pwmConfig.clockSource);
			// 	printf("\n\r Pwm_Divider :%d", pwmConfig.timer_prescaler);
			// 	printf("\n\r Pwm_Mode :%d", pwmConfig.compareOutputMode);
			// 	printf("\n\r Pwm_Register :%d", pwmConfig.compareRegister);
			// 	printf("\n\r Pwm_Duty :%d", pwmConfig.dutyCycle);
			// 	printf("\n\r Pwm_Period :%d", pwmConfig.timerPeriod);
			// 	printf("\n\r CLK_freq :%d", CS_getSMCLK());
			// }
		}
	}
}

