#include <Drivers_Control.h>

void  dr_Leds_alterar(uint16_t contador)
{
	if (contador == 1)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		dr_clc_RGB_blue;
		dr_clc_RGB_green;
		dr_clc_RGB_red;

		dr_set_RGB_blue;
	}
	if (contador == 2)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		dr_clc_RGB_blue;
		dr_set_RGB_green;
	}
	if (contador == 3)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		dr_clc_RGB_green;
		dr_set_RGB_red;
	}
	if (contador == 4)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		dr_clc_RGB_blue;
		dr_clc_RGB_green;
		dr_clc_RGB_red;
		dr_set_RGB_blue;
		dr_set_RGB_green;

	}
	if (contador == 5)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		dr_clc_RGB_blue;
		dr_clc_RGB_green;
		dr_clc_RGB_red;
		dr_set_RGB_blue;
		dr_set_RGB_red;
	}
	if (contador == 6)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		dr_clc_RGB_blue;
		dr_clc_RGB_green;
		dr_clc_RGB_red;
		dr_set_RGB_red;
		dr_set_RGB_green;
	}
	if (contador == 7)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		dr_clc_RGB_blue;
		dr_clc_RGB_green;
		dr_clc_RGB_red;
	}
}
void dr_Delay_k(double maximo)
{
	double contagem;
	contagem = 1000 * maximo;
	uint32_t ii;
	for (ii = 0; ii < contagem; ii++) ;
}
void dr_Delay_ms(int n)
{
	int i, j;
	for (j = 0; j < n; j++)
		for (i = 250; i > 0; i--) ;
}
void dr_Delay_s(int n)
{
	int i, j;
	for (j = 0; j < n; j++)
		for (i = 250000; i > 0; i--) ;
}

void dr_Leds_sw_init()
{
	// Selecionar Função dos Pinos
	GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);                // led 1
	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);                // led 2 (Vermelho)
	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);                // led 2 (Verde)
	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);                // led 2 (Azul)

	dr_clc_led1;
	dr_clc_RGB_blue;
	dr_clc_RGB_green;
	dr_clc_RGB_red;

	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);                 //sw1
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);                 //sw2


}
void  dr_Uart_init()
{
	/* Selecting P1.2 and P1.3 in UART mode */
	GPIO_setAsPeripheralModuleFunctionInputPin(
			GPIO_PORT_P1,
		GPIO_PIN2 | GPIO_PIN3,
		GPIO_PRIMARY_MODULE_FUNCTION);

	/* Setting DCO to 12MHz */
	CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

	// set BAUD_RATE_9600
	// SMCLK Clock Source
	// BRDIV = 78 // UCxBRF = 2	 // UCxBRS = 0
	// No Parity  // LSB First    // One stop bit
	// UART mode  // Oversampling // 8 bit data length
const eUSCI_UART_ConfigV1 uartConfig = {
		EUSCI_A_UART_CLOCKSOURCE_SMCLK,
		clockPrescalar,
		firstModReg,
		secondModReg,
		EUSCI_A_UART_NO_PARITY,
		EUSCI_A_UART_LSB_FIRST,
		EUSCI_A_UART_ONE_STOP_BIT,
		EUSCI_A_UART_MODE,
		EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,
		EUSCI_A_UART_8_BIT_LEN

	};
	/* Configuring UART Module */
	UART_initModule(EUSCI_A0_BASE, &uartConfig);
	/* Enable UART module */
	UART_enableModule(EUSCI_A0_BASE);

	// printf para testar
	printf("\r\nPrintf support for the launchpad\r\n");
	printf("Decimal(10) :%d\r\n", 10);
	printf("Hex(10)     :%x\r\n", 10);
	printf("float       :%f\r\n", 4.32);

}
void dr_Uart_interrupt_receive()
{
	//	void EUSCIA0_IRQHandler(void)
	/* Enabling interrupts */
	UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
	Interrupt_enableInterrupt(INT_EUSCIA0);
	//Interrupt_enableSleepOnIsrExit();

	// Tirar habilitar interrupção em master
	//Interrupt_enableMaster();
}
extern void dr_Interrupt_on() {
	Interrupt_enableMaster();
}
void dr_Leds_alterar_pela_sw()
{
	int estado_SW1 = dr_read_SW1
	;
	int estado_SW2 = dr_read_SW2
	;
	static int k;
	if (estado_SW1 == 1)
	{
		k++;
		if (k == 8)
		{
			k = 7;
		}
		dr_Leds_alterar(k);
		dr_Delay_k(5);

	}
	if (estado_SW2 == 1)
	{
		if (k == 0)
		{
			k = 1;
		}
		k--;
		dr_Leds_alterar(k);
		dr_Delay_k(5);
	}

}
int fputc(int _c, register FILE *_fp)
{
	while (!(UCA0IFG & UCTXIFG)) ;
	UCA0TXBUF = (unsigned char) _c;

	return ((unsigned char) _c);
}

int fputs(const char *_ptr, register FILE *_fp)
{
	unsigned int i, len;

	len = strlen(_ptr);

	for (i = 0; i < len; i++)
	{
		while (!(UCA0IFG & UCTXIFG)) ;
		UCA0TXBUF = (unsigned char) _ptr[i];
	}

	return len;
}

uint32_t dr_Tick_start()
{
	uint32_t period = 10* CS_getMCLK();
	SysTick_setPeriod(period);       // 10seg
	SysTick->VAL = 0;       // reiniciar contagem
	SysTick_enableInterrupt();
	SysTick_enableModule();      // inicia de fato a contagem
	return SysTick_getValue();
}
uint32_t dr_Tick_stop()
{
	uint32_t tick = SysTick_getValue();     // para a contagem

	double rate = CS_getMCLK() / 1000000;
	tick = SysTick_getPeriod() - tick;

	double tempo_ms = tick / rate;     	// tempo em us
	tempo_ms = tempo_ms / 1000;     		// tempo em ms

	SysTick_disableInterrupt();
	SysTick_disableModule();
	printf("\n\rO valor do Systick: %.3fms,MCLK:%.2fMHz", tempo_ms, rate);
	// todas essas funções duram em torno de 0.71 ms
	return tick;
}
void SysTick_Handler(void)
{
	printf("\n\r****** Tempo_do_Sys_Tick_estorou *******");
}

void  dr_Clk_print()
{
	volatile uint32_t MCLK_timer =   CS_getMCLK();
	volatile uint32_t SMCLK_timer =	 CS_getSMCLK();
	volatile uint32_t HSMCLK_timer = CS_getHSMCLK();
	volatile uint32_t DCO_timer =	 CS_getDCOFrequency();
	volatile uint32_t ACLK_timer =	 CS_getACLK();
	volatile uint32_t BCLK_timer =	 CS_getBCLK();

	printf("\n\r MCLK:%u, SMCLK:%u, HSMCLK:%u,\n\r DCO:%u, ACLK:%u, BCLK%u",
		MCLK_timer,
		SMCLK_timer,
		HSMCLK_timer,
		DCO_timer,
		ACLK_timer,
		BCLK_timer);


}
void dr_T32_init_seg(uint32_t timer, float tempo_seg)
{
	/* Seleção do Timer */
	if (timer == 0)
		timer = TIMER32_0_BASE;
	else
		timer = TIMER32_1_BASE;

	/* Cálculo do periodo do timer */
	uint32_t mclk = CS_getMCLK();
	double count_period = mclk / 256;
	count_period = count_period * tempo_seg;

	/* Inicialização e configuração do módulo */
	Timer32_initModule(timer,
		TIMER32_PRESCALER_256,
		TIMER32_32BIT,
		TIMER32_PERIODIC_MODE);
	Timer32_setCount(timer, count_period);
}
void dr_T32_init_Hz(uint32_t timer, float freq_Hz)
{
	/* Seleção do Timer */
	if (timer == 0)
		timer = TIMER32_0_BASE;
	else
		timer = TIMER32_1_BASE;

	/* Cálculo do periodo do timer */
	uint32_t mclk = CS_getMCLK();
	double count_period = mclk / freq_Hz;

	/* Inicialização e configuração do módulo */
	Timer32_initModule(timer,
		TIMER32_PRESCALER_1,
		TIMER32_32BIT,
		TIMER32_PERIODIC_MODE);
	Timer32_setCount(timer, count_period);
}

void dr_T32_start(uint32_t timer)
{
	/* Seleção do Timer */
	if (timer == 0)
		timer = TIMER32_0_BASE;
	else
		timer = TIMER32_1_BASE;

	Timer32_startTimer(timer, false);
}
void dr_T32_interrupt_init(uint32_t timer, void rotina(void))
{
	/* Seleção do Timer */
	if (timer == 0)
		timer = INT_T32_INT1;
	else
		timer = INT_T32_INT2;
	// Sempre definir uma rotina para receber a interrupção como a registrada aqui
	Interrupt_registerInterrupt(timer, rotina);
	Interrupt_enableInterrupt(timer);

}
double dr_T32_getPeriod_seg(uint32_t timer)
{
	double period;
	/* Calcular Prescaler */
	/* Preciso ver um modo de ler o valor do prescaler*/

	/* Seleção do Timer */
	if (timer == 0)
		period = TIMER32_1->LOAD * 256;
	else
		period = TIMER32_2->LOAD * 256;

	/* Cálculo do Periodo */
	period = period / (CS_getMCLK());
	return period;
}
double dr_T32_get_freq(uint32_t timer)
{
	double period;
	/* Calcular Prescaler */
	/* Preciso ver um modo de ler o valor do prescaler */

	/* Seleção do Timer */
	if (timer == 0)
		period = TIMER32_1->LOAD;
	else
		period = TIMER32_2->LOAD;

	/* Cálculo do Periodo */
	period =  CS_getMCLK() / period;
	return period;
}
