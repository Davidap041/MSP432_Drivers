#include <Drivers_Control.h>

void Alterar_LEDS(uint16_t contador)
{
	if (contador == 1)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		clc_RGB_blue;
		clc_RGB_green;
		clc_RGB_red;

		set_RGB_blue;
	}
	if (contador == 2)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		clc_RGB_blue;
		set_RGB_green;
	}
	if (contador == 3)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		clc_RGB_green;
		set_RGB_red;
	}
	if (contador == 4)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		clc_RGB_blue;
		clc_RGB_green;
		clc_RGB_red;
		set_RGB_blue;
		set_RGB_green;

	}
	if (contador == 5)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		clc_RGB_blue;
		clc_RGB_green;
		clc_RGB_red;
		set_RGB_blue;
		set_RGB_red;
	}
	if (contador == 6)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		clc_RGB_blue;
		clc_RGB_green;
		clc_RGB_red;
		set_RGB_red;
		set_RGB_green;
	}
	if (contador == 7)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
		clc_RGB_blue;
		clc_RGB_green;
		clc_RGB_red;
	}
}
void delay_k(double maximo)
{
	double contagem;
	contagem = 1000 * maximo;
	uint32_t ii;
	for (ii = 0; ii < contagem; ii++) ;
}
void Pin_config_1()
{
	// Selecionar Função dos Pinos
	GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);            // led 1
	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);            // led 2 (Vermelho)
	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);            // led 2 (Verde)
	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);            // led 2 (Azul)

	clc_led1;
	clc_RGB_blue;
	clc_RGB_green;
	clc_RGB_red;

	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);             //sw1
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);             //sw2


}
void config_uart()
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
	// BRDIV = 78 // UCxBRF = 2  // UCxBRS = 0
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
}
void interrupt_uart()
{
	/* Enabling interrupts */
	UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
	Interrupt_enableInterrupt(INT_EUSCIA0);
	Interrupt_enableSleepOnIsrExit();
	Interrupt_enableMaster();
}

void Alterar_LEDS_pela_SW()
{
	int estado_SW1 = read_SW1
	;
	int estado_SW2 = read_SW2
	;
	static int k;
	if (estado_SW1 == 1)
	{
		k++;
		if (k == 8)
		{
			k = 7;
		}
		Alterar_LEDS(k);
		delay_k(5);

	}
	if (estado_SW2 == 1)
	{
		if (k == 0)
		{
			k = 1;
		}
		k--;
		Alterar_LEDS(k);
		delay_k(5);
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

