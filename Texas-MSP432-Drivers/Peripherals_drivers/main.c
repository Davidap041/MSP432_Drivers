/* DriverLib Includes */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
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
/* Timer A PWM configure */
#define PWM_config_period  60000 // count Value
#define PWM_config_prescale  20
#define PWM_config_duty  60000 // pwm count in count Value

Timer_A_PWMConfig pwmConfig = { // Timer Configuration in 10 HZ, period = 6000 e duty 30000
		TIMER_A_CLOCKSOURCE_SMCLK,
		TIMER_A_CLOCKSOURCE_DIVIDER_64,
		PWM_config_period,
		TIMER_A_CAPTURECOMPARE_REGISTER_1,
		TIMER_A_OUTPUTMODE_RESET_SET,
		PWM_config_duty };

int main(void)
{

	WDT_A_holdTimer();
	/* Pin Config */
	dr_Leds_sw_init();

	/* Peripherals Config */
	dr_Uart_init();

	/* Interrupt Config */
	dr_Uart_interrupt_receive();

	/* Inicializar Programas*/
	/* Programa de PWM */
	// Timer_A_PWMConfig(
	// Timer_A_generatePWM()
	/* Cálculo do PWM */
	uint32_t SMCLK_freq = CS_getSMCLK();
	/* PWM : Pin Config */
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
	GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.1
	/* PWM : Peripheral Config */
	Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

	while (1)
	{
		dr_Leds_alterar(contador);
		dr_Delay_s(0.5);

		if (status_flag_uart)
		{
			contador = UART_receiveData(EUSCI_A0_BASE) - 48;
			printf("\n\n\rValor Leds em: %d", contador);
			status_flag_uart = 0;
			if (contador == 1) // Incrementar o Duty_Cycle
			{
				if (pwmConfig.dutyCycle != PWM_config_period)
				{
					pwmConfig.dutyCycle += 1000;
					Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
				}

			}
			if (contador == 2) // Decrementar o Duty Cycle
			{
				if (pwmConfig.dutyCycle > 0)
					pwmConfig.dutyCycle -= 1000;
				Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
			}
			if (contador == 3)
			{ /* PWM information */
				printf("\n\r Pwm_Source :%x", pwmConfig.clockSource);
				printf("\n\r Pwm_Divider :%d", pwmConfig.clockSourceDivider);
				printf("\n\r Pwm_Mode :%d", pwmConfig.compareOutputMode);
				printf("\n\r Pwm_Register :%d", pwmConfig.compareRegister);
				printf("\n\r Pwm_Duty :%d", pwmConfig.dutyCycle);
				printf("\n\r Pwm_Period :%d", pwmConfig.timerPeriod);
				printf("\n\r CLK_freq :%d", CS_getSMCLK());
			}

		}
	}
}
