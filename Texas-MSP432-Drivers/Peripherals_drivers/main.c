/* DriverLib Includes */
#include <ti/devices/msp432p4xx/inc/msp432p401r.h>
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
//#define PWM_config_period  60000 // count Value
//#define PWM_config_prescale  20
//#define PWM_config_duty  60000 // pwm count in count Value

float dr_PWM_getfreq(){
	uint16_t PWM_period = TIMER_A0->CCR[0]; /* PWM period*/
	uint32_t PWM_clk = CS_getSMCLK(); 		/* Timer CLK*/
	/* Timer Prescaler 
	PWM_prescaler = (TIMER_A0->TACL.ID*TIMER_A0->TACL.ID)* (TIMER_A0->TAEX0.TAIDEX + 1)
	*/
	/* Timer Period 
	TIMER_A0->CCR[0] + 1
	*/
return 0;
}
float dr_PWM_getDuty(uint16_t Pwm_Channel){
	// dependendo do modo tem formas diferentes de calcular
	/* se for no modo reset/set */
	uint16_t PWM_period = TIMER_A_CMSIS(TIMER_A0_BASE)->CCR[0]; /* PWM period*/
	uint16_t PWM_set_period = TIMER_A_CMSIS(TIMER_A0_BASE)->CCR[Pwm_Channel]; /* Depende do modo */ 
	float duty_Cycle = PWM_set_period/PWM_period;

	return duty_Cycle;
}
void dr_PWM_init(uint_fast16_t PWM_config_period, uint_fast16_t PWM_config_duty,uint16_t timer_divider){
Timer_A_PWMConfig pwmConfig = { // Timer Configuration in 10 HZ, period = 6000 e duty 30000
		TIMER_A_CLOCKSOURCE_SMCLK,	// Selecionando o Oscilador
		timer_divider,				
		PWM_config_period,			//
		TIMER_A_CAPTURECOMPARE_REGISTER_1,
		TIMER_A_OUTPUTMODE_RESET_SET,
		PWM_config_duty };
/* PWM in Up mode, para configurar em contiunos mode ou UP/Down mode vai ser dps */		
uint16_t timer = TIMER_A0_BASE;
	Timer_A_generatePWM(timer, &pwmConfig);
/* Aditional Configuration */
	
/* Configure the others channels Output*/
//	TIMER_A_CMSIS(timer)->CCTL[1] = 0xE0; /* CCR1 reset/set mode */
	TIMER_A_CMSIS(timer)->CCTL[2] = 0xE0; /* CCR2 reset/set mode */
	TIMER_A_CMSIS(timer)->CCTL[3] = 0xE0; /* CCR3 reset/set mode */
	TIMER_A_CMSIS(timer)->CCTL[4] = 0xE0; /* CCR4 reset/set mode */
	
/* Configurar outro limite de contagem, ou seja outra frequência*/
	// TIMER_A_CMSIS(timer)->CCR[0] = 60000 - 1;
/* Inicializar com diferentes duty_Cycles */	
	// TIMER_A_CMSIS(timer)->CCR[1] = 30000; /* Depende do modo */ 

	/* Configurações para serem estudadas depois */
//	TIMER_A_CMSIS(timer)->CCTL[1] = 0x40; /* CCR1 tougle/reset mode */
// TIMER_A_CMSIS(timer)->CTL = 0x0234;	/* Use SMCLK, up/downmode,clear TA0R register */

// privateTimer_AProcessClockSourceDivider(timer,timer_divider);/* Setar o prescaler diretamente*/

/* Aprender a Configurar esse CTL

 TIMER_A_CMSIS(timer)->CTL &=
            ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK + TIMER_A_UPDOWN_MODE
                    + TIMER_A_DO_CLEAR + TIMER_A_TAIE_INTERRUPT_ENABLE);

    TIMER_A_CMSIS(timer)->CTL |= (config->clockSource + TIMER_A_UP_MODE
            + TIMER_A_DO_CLEAR);

			/* Selecionar os outros canais da forla da SDK
			ASSERT(
            (TIMER_A_CAPTURECOMPARE_REGISTER_0 == config->compareRegister)
            || (TIMER_A_CAPTURECOMPARE_REGISTER_1
                    == config->compareRegister)
            || (TIMER_A_CAPTURECOMPARE_REGISTER_2
                    == config->compareRegister)
            || (TIMER_A_CAPTURECOMPARE_REGISTER_3
                    == config->compareRegister)
            || (TIMER_A_CAPTURECOMPARE_REGISTER_4
                    == config->compareRegister)
            || (TIMER_A_CAPTURECOMPARE_REGISTER_5
                    == config->compareRegister)
            || (TIMER_A_CAPTURECOMPARE_REGISTER_6
                    == config->compareRegister));
 			uint8_t idx = (config->compareRegister>> 1) - 1;
    TIMER_A_CMSIS(timer)->CCTL[idx] |= config->compareOutputMode; // Configura a saída do Canal
    TIMER_A_CMSIS(timer)->CCR[idx] = config->dutyCycle;			  // Configura o duty Cycle
*/
}
int main(void)
{

	WDT_A_holdTimer();
	/* Pin Config */
	dr_Leds_sw_init();
	/* PWM : Pin Config */
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
	GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.1
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5,
	GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.2
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6,
	GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.3
	// GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
	// GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.4


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
	
	/* PWM : Peripheral Config */
	dr_PWM_init(60000,30000,64);

	while (1)
	{
		dr_Leds_alterar(contador);
		dr_Delay_s(2); // deixar para ter alguma referência

		if (status_flag_uart)
		{
			contador = UART_receiveData(EUSCI_A0_BASE) - 48;
			printf("\n\n\rValor Leds em: %d", contador);
			status_flag_uart = 0;
			// if (contador == 1) // Incrementar o Duty_Cycle
			// {
			// 	if (pwmConfig.dutyCycle != PWM_config_period)
			// 	{
			// 		pwmConfig.dutyCycle += 1000;
			// 		Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
			// 	}

			// }
			// if (contador == 2) // Decrementar o Duty Cycle
			// {
			// 	if (pwmConfig.dutyCycle > 0)
			// 		pwmConfig.dutyCycle -= 1000;
			// 	Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
			// }
			// if (contador == 3)
			// { /* PWM information */
			// 	printf("\n\r Pwm_Source :%x", pwmConfig.clockSource);
			// 	printf("\n\r Pwm_Divider :%d", pwmConfig.clockSourceDivider);
			// 	printf("\n\r Pwm_Mode :%d", pwmConfig.compareOutputMode);
			// 	printf("\n\r Pwm_Register :%d", pwmConfig.compareRegister);
			// 	printf("\n\r Pwm_Duty :%d", pwmConfig.dutyCycle);
			// 	printf("\n\r Pwm_Period :%d", pwmConfig.timerPeriod);
			// 	printf("\n\r CLK_freq :%d", CS_getSMCLK());
			// }

		}
	}
}
