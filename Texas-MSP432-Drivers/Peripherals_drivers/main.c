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
typedef struct _Dr_pwm_parameters
{
	bool fast_mode;
	uint16_t timer_Prescaler;
	bool true_Sawtooth_not_triangular;
	uint_fast16_t period_count;
} Dr_pwm_parameters;

void _Dr_pwm_set_Prescaler(uint32_t timer, uint16_t timer_prescaler)
{	/* This functions is only to be used by DR_PWM_config  */
	TIMER_A_CMSIS(timer)->CTL &= ~TIMER_A_CTL_ID__8;
	TIMER_A_CMSIS(timer)->EX0 &= ~TIMER_A_EX0_IDEX_MASK;

	switch (timer_prescaler)
	{
	case TIMER_A_CLOCKSOURCE_DIVIDER_1:
	case TIMER_A_CLOCKSOURCE_DIVIDER_2:
		TIMER_A_CMSIS(timer)->CTL |= ((timer_prescaler - 1) << 6);
		TIMER_A_CMSIS(timer)->EX0 = TIMER_A_EX0_TAIDEX_0;
		break;
	case TIMER_A_CLOCKSOURCE_DIVIDER_4:
		TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__4;
		TIMER_A_CMSIS(timer)->EX0 = TIMER_A_EX0_TAIDEX_0;
		break;
	case TIMER_A_CLOCKSOURCE_DIVIDER_8:
		TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__8;
		TIMER_A_CMSIS(timer)->EX0 = TIMER_A_EX0_TAIDEX_0;
		break;
	case TIMER_A_CLOCKSOURCE_DIVIDER_3:
	case TIMER_A_CLOCKSOURCE_DIVIDER_5:
	case TIMER_A_CLOCKSOURCE_DIVIDER_6:
	case TIMER_A_CLOCKSOURCE_DIVIDER_7:
		TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__1;
		TIMER_A_CMSIS(timer)->EX0 = (timer_prescaler - 1);
		break;

	case TIMER_A_CLOCKSOURCE_DIVIDER_10:
	case TIMER_A_CLOCKSOURCE_DIVIDER_12:
	case TIMER_A_CLOCKSOURCE_DIVIDER_14:
	case TIMER_A_CLOCKSOURCE_DIVIDER_16:
		TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__2;
		TIMER_A_CMSIS(timer)->EX0 = (timer_prescaler / 2 - 1);
		break;

	case TIMER_A_CLOCKSOURCE_DIVIDER_20:
	case TIMER_A_CLOCKSOURCE_DIVIDER_24:
	case TIMER_A_CLOCKSOURCE_DIVIDER_28:
	case TIMER_A_CLOCKSOURCE_DIVIDER_32:
		TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__4;
		TIMER_A_CMSIS(timer)->EX0 = (timer_prescaler / 4 - 1);
		break;
	case TIMER_A_CLOCKSOURCE_DIVIDER_40:
	case TIMER_A_CLOCKSOURCE_DIVIDER_48:
	case TIMER_A_CLOCKSOURCE_DIVIDER_56:
	case TIMER_A_CLOCKSOURCE_DIVIDER_64:
		TIMER_A_CMSIS(timer)->CTL |= TIMER_A_CTL_ID__8;
		TIMER_A_CMSIS(timer)->EX0 = (timer_prescaler / 8 - 1);
		break;
	}
}
uint16_t DR_PWM_getperiod_count(uint32_t timer)
{
	return TIMER_A_CMSIS(timer)->CCR[0]; /* PWM period*/
}
float DR_PWM_getfreq(uint32_t timer, const Dr_pwm_parameters *PWM)
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
void DR_PWM_setDuty(uint32_t timer, uint16_t pwm_channel,
					uint_fast16_t pwm_duty)
{
	TIMER_A_CMSIS(timer)->CCR[pwm_channel] = pwm_duty;
}
uint16_t DR_PWM_getDuty(uint32_t timer, uint16_t PWM_Channel)
{
	return TIMER_A_CMSIS(timer)->CCR[PWM_Channel];
}

float DR_PWM_getDuty_percent(uint16_t Pwm_Channel)
{
	// dependendo do modo tem formas diferentes de calcular
	/* se for no modo reset/set */
	uint16_t PWM_period = TIMER_A_CMSIS(TIMER_A0_BASE)->CCR[0]; /* PWM period*/
	uint16_t PWM_set_period = TIMER_A_CMSIS(TIMER_A0_BASE)->CCR[Pwm_Channel]; /* Depende do modo */
	float duty_Cycle = PWM_set_period / PWM_period;

	return duty_Cycle;
}

void DR_PWM_config(uint32_t timer, const Dr_pwm_parameters *pwm_config)
{
	// Selecionando clk_source
	uint_fast16_t clk_source;
	if (pwm_config->fast_mode)
	{
		clk_source = TIMER_A_CLOCKSOURCE_SMCLK;
	}
	else
	{
		clk_source = TIMER_A_CLOCKSOURCE_ACLK;
	}

	// Selecionando o modo de configuração do timer
	uint_fast16_t pwm_carrier;
	if (pwm_config->true_Sawtooth_not_triangular)
	{
		pwm_carrier = TIMER_A_UP_MODE; // set Sawtooth Carrier mode
	}
	else
	{
		pwm_carrier = TIMER_A_UPDOWN_MODE; // set Traingular Carrier mode
	}

	// Selecionar Prescaler
	_Dr_pwm_set_Prescaler(timer, pwm_config->timer_Prescaler);
	// Limpar registradores que irão ser configurados
	TIMER_A_CMSIS(timer)->CTL &= ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
			+ TIMER_A_UPDOWN_MODE + TIMER_A_DO_CLEAR
			+ TIMER_A_TAIE_INTERRUPT_ENABLE);

	// Setando as configurações recebidas
	TIMER_A_CMSIS(timer)->CTL |= (clk_source + pwm_carrier + TIMER_A_DO_CLEAR); // TACLR: IMER_A_DO_CLEAR 
	TIMER_A_CMSIS(timer)->CCR[0] = pwm_config->period_count; // Configurando Periodo PWM
}

void DR_PWM_init(uint32_t timer, uint16_t pwm_channel,
					uint_fast16_t output_Mode, uint_fast16_t pwm_init_duty)
{	// Limpando os registradores que serão usados
	TIMER_A_CMSIS(timer)->CCTL[0] &= ~(TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE
			+ TIMER_A_OUTPUTMODE_RESET_SET);
	TIMER_A_CMSIS(timer)->CCTL[pwm_channel] |= output_Mode;
	TIMER_A_CMSIS(timer)->CCR[pwm_channel] = pwm_init_duty; /*Como setar o duty do canal 0 ?*/
}

Dr_pwm_parameters PWM_1 = { .fast_mode = true, .timer_Prescaler = 64,
							.true_Sawtooth_not_triangular = true,
							.period_count = 60000 };

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
	// GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6,
	// GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.3
	// GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
	// GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA0.4
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN6,
	GPIO_PRIMARY_MODULE_FUNCTION); // PM_TA2.1

	/* Peripherals Config */
	dr_Uart_init();
	DR_PWM_config(TIMER_A0_BASE, &PWM_1);
	DR_PWM_config(TIMER_A2_BASE, &PWM_1);

	/* Interrupt Config */
	dr_Uart_interrupt_receive();
	DR_PWM_init(TIMER_A0_BASE, 1, TIMER_A_OUTPUTMODE_RESET_SET, 30000);
	DR_PWM_init(TIMER_A0_BASE, 2, TIMER_A_OUTPUTMODE_RESET_SET, 30000);
	DR_PWM_init(TIMER_A2_BASE, 1, TIMER_A_OUTPUTMODE_RESET_SET, 30000);
	/* Inicializar Programas*/

	//	dr_PWM_init(60000,30000,64);
	while (1)
	{
		dr_Leds_alterar(contador);
		dr_Delay_s(2); // deixar para ter alguma referência

		if (status_flag_uart)
		{
			contador = UART_receiveData(EUSCI_A0_BASE) - 48;
			printf("\n\n\rValor Leds em: %d", contador);
			status_flag_uart = 0;
			if (contador == 1) // Incrementar o Duty_Cycle
			{
				uint16_t PWM_period = DR_PWM_getperiod_count(TIMER_A2_BASE);
				uint16_t PWM_Duty = DR_PWM_getDuty(TIMER_A2_BASE, 1);
				if (PWM_Duty < PWM_period)
				{
					PWM_Duty += 1000;
					DR_PWM_setDuty(TIMER_A0_BASE, 1, PWM_Duty);
					DR_PWM_setDuty(TIMER_A0_BASE, 2, PWM_Duty);
					DR_PWM_setDuty(TIMER_A2_BASE, 1, PWM_Duty);
				}
			}
			if (contador == 2) // Decrementar o Duty Cycle
			{
				uint16_t PWM_Duty = DR_PWM_getDuty(TIMER_A2_BASE, 1);
				if (PWM_Duty > 0)
				{
					PWM_Duty -= 1000;
					DR_PWM_setDuty(TIMER_A0_BASE, 1, PWM_Duty);
					DR_PWM_setDuty(TIMER_A0_BASE, 2, PWM_Duty);
					DR_PWM_setDuty(TIMER_A2_BASE, 1, PWM_Duty);
					
				}
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

// Timer_A_PWMConfig pwmConfig = { // Timer Configuration in 10 HZ, period = 6000 e duty 30000
// 		TIMER_A_CLOCKSOURCE_SMCLK,	// Selecionando o Oscilador
// 		timer_divider,
// 		PWM_config_period,			//
// 		TIMER_A_CAPTURECOMPARE_REGISTER_1,
// 		TIMER_A_OUTPUTMODE_RESET_SET,
// 		PWM_config_duty };
// /* PWM in Up mode, para configurar em contiunos mode ou UP/Down mode vai ser dps */
// uint16_t timer = TIMER_A0_BASE;
// 	Timer_A_generatePWM(timer, &pwmConfig);
/* Aditional Configuration */

/* Configure the others channels Output*/
//	TIMER_A_CMSIS(timer)->CCTL[1] = 0xE0; /* CCR1 reset/set mode */
// TIMER_A_CMSIS(timer)->CCTL[2] = 0xE0; /* CCR2 reset/set mode */
// TIMER_A_CMSIS(timer)->CCTL[3] = 0xE0; /* CCR3 reset/set mode */
// TIMER_A_CMSIS(timer)->CCTL[4] = 0xE0; /* CCR4 reset/set mode */
/* Configurar outro limite de contagem, ou seja outra frequência*/
// TIMER_A_CMSIS(timer)->CCR[0] = 60000 - 1;
/* Inicializar com diferentes duty_Cycles */
// TIMER_A_CMSIS(timer)->CCR[1] = 30000; /* Depende do modo */
/* Configurações para serem estudadas depois */
//	TIMER_A_CMSIS(timer)->CCTL[1] = 0x40; /* CCR1 tougle/reset mode */
// TIMER_A_CMSIS(timer)->CTL = 0x0234;	/* Use SMCLK, up/downmode,clear TA0R register */
// privateTimer_AProcesstimer_prescaler(timer,timer_divider);/* Setar o prescaler diretamente*/
/* Aprender a Configurar esse CTL

 TIMER_A_CMSIS(timer)->CTL &=
 ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK + TIMER_A_UPDOWN_MODE
 + TIMER_A_DO_CLEAR + TIMER_A_TAIE_INTERRUPT_ENABLE);

 TIMER_A_CMSIS(timer)->CTL |= (config->clockSource + TIMER_A_UP_MODE
 + TIMER_A_DO_CLEAR);

 Selecionar os outros canais da forla da SDK
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
 TIMER_A_CMSIS(timer)->CCTL[idx] |= output_Mode; // Configura a saída do Canal
 TIMER_A_CMSIS(timer)->CCR[idx] = config->dutyCycle;			  // Configura o duty Cycle
 */
// ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == output_Mode)
// 		|| (TIMER_A_OUTPUTMODE_SET == output_Mode)
// 		|| (TIMER_A_OUTPUTMODE_TOGGLE_RESET == output_Mode)
// 		|| (TIMER_A_OUTPUTMODE_SET_RESET == output_Mode)
// 		|| (TIMER_A_OUTPUTMODE_TOGGLE == output_Mode)
// 		|| (TIMER_A_OUTPUTMODE_RESET == output_Mode)
// 		|| (TIMER_A_OUTPUTMODE_TOGGLE_SET == output_Mode)
// 		|| (TIMER_A_OUTPUTMODE_RESET_SET == output_Mode));  // Mais comum
