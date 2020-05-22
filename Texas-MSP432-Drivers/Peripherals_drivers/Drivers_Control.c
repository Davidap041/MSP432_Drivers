#include <Drivers_Control.h>

void dr_Leds_alterar(uint16_t contador)
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
	for (ii = 0; ii < contagem; ii++)
		;
}
void dr_Delay_ms(int n)
{
	int i, j;
	for (j = 0; j < n; j++)
		for (i = 250; i > 0; i--)
			;
}
void dr_Delay_s(int n)
{
	int i, j;
	for (j = 0; j < n; j++)
		for (i = 250000; i > 0; i--)
			;
}

void dr_Leds_sw_init()
{
	// Selecionar Fun��o dos Pinos
	GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);                 // led 1
	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);           	 // led 2 (Vermelho)
	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);               // led 2 (Verde)
	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);                // led 2 (Azul)

	dr_clc_led1;
	dr_clc_RGB_blue;
	dr_clc_RGB_green;
	dr_clc_RGB_red;

	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);         //sw1
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);         //sw2

}
void dr_Uart_init()
{
	/* Altera o DCO para 12MHZ: afetando */
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

	// Tirar habilitar interrup��o em master
	//Interrupt_enableMaster();
}
extern void dr_Interrupt_on()
{
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
	while (!(UCA0IFG & UCTXIFG))
		;
	UCA0TXBUF = (unsigned char) _c;

	return ((unsigned char) _c);
}

int fputs(const char *_ptr, register FILE *_fp)
{
	unsigned int i, len;

	len = strlen(_ptr);

	for (i = 0; i < len; i++)
	{
		while (!(UCA0IFG & UCTXIFG))
			;
		UCA0TXBUF = (unsigned char) _ptr[i];
	}

	return len;
}

uint32_t dr_Tick_start()
{
	estouro_Systick = 10;  // 10 seg
	uint32_t period = CS_getMCLK();
	SysTick_setPeriod(period);        // 1seg
	SysTick->VAL = 0;        // reiniciar contagem
	SysTick_enableInterrupt();
	SysTick_enableModule();       // inicia de fato a contagem
	return SysTick_getValue();
}
uint32_t dr_Tick_stop()
{
	uint32_t tick = SysTick_getValue();      // para a contagem

	double rate = CS_getMCLK() / 1000000;
	tick = SysTick_getPeriod() - tick;

	double tempo_ms = tick / rate;      	// tempo em us
	tempo_ms = tempo_ms / 1000;      		// tempo em ms
	tempo_ms = (10 - estouro_Systick) * 1000 + tempo_ms;
	SysTick_disableInterrupt();
	SysTick_disableModule();
	printf("\n\rO valor do Systick: %.4fms,MCLK:%.2fMHz", tempo_ms, rate);
	// todas essas fun��es duram em torno de 0.71 ms
	return tick;
}
void SysTick_Handler(void)
{
	if (estouro_Systick == 0)
	{
		printf("\n\r****** Tempo_do_Sys_Tick_estorou *******");
		estouro_Systick = 10;
	}
	else
	{
		estouro_Systick--;
	}
}

void dr_Clk_print()
{
	volatile uint32_t MCLK_timer = CS_getMCLK();
	volatile uint32_t SMCLK_timer = CS_getSMCLK();
	volatile uint32_t HSMCLK_timer = CS_getHSMCLK();
	volatile uint32_t DCO_timer = CS_getDCOFrequency();
	volatile uint32_t ACLK_timer = CS_getACLK();
	volatile uint32_t BCLK_timer = CS_getBCLK();

	printf("\n\r MCLK:%u, SMCLK:%u, HSMCLK:%u,\n\r DCO:%u, ACLK:%u, BCLK%u",
			MCLK_timer, SMCLK_timer, HSMCLK_timer, DCO_timer, ACLK_timer,
			BCLK_timer);

}
void dr_T32_init_seg(uint32_t timer, float tempo_seg)
{
	/* Sele��o do Timer */
	if (timer == 0)
		timer = TIMER32_0_BASE;
	else
		timer = TIMER32_1_BASE;

	/* C�lculo do periodo do timer */
	uint32_t mclk = CS_getMCLK();
	double count_period = mclk / 256;
	count_period = count_period * tempo_seg;

	/* Inicializa��o e configura��o do m�dulo */
	Timer32_initModule(timer,
	TIMER32_PRESCALER_256,
						TIMER32_32BIT,
						TIMER32_PERIODIC_MODE);
	Timer32_setCount(timer, count_period);
}
void dr_T32_init_Hz(uint32_t timer, float freq_Hz)
{
	/* Sele��o do Timer */
	if (timer == 0)
		timer = TIMER32_0_BASE;
	else
		timer = TIMER32_1_BASE;

	/* C�lculo do periodo do timer */
	uint32_t mclk = CS_getMCLK();
	double count_period = mclk / freq_Hz;

	/* Inicializa��o e configura��o do m�dulo */
	Timer32_initModule(timer,
	TIMER32_PRESCALER_1,
						TIMER32_32BIT,
						TIMER32_PERIODIC_MODE);
	Timer32_setCount(timer, count_period);
}

void dr_T32_start(uint32_t timer)
{
	/* Sele��o do Timer */
	if (timer == 0)
		timer = TIMER32_0_BASE;
	else
		timer = TIMER32_1_BASE;

	Timer32_startTimer(timer, false);
}
void dr_T32_interrupt_init(uint32_t timer, void rotina(void))
{
	/* Sele��o do Timer */
	if (timer == 0)
		timer = INT_T32_INT1;
	else
		timer = INT_T32_INT2;
	// Sempre definir uma rotina para receber a interrup��o como a registrada aqui
	Interrupt_registerInterrupt(timer, rotina);
	Interrupt_enableInterrupt(timer);

}
double dr_T32_getPeriod_seg(uint32_t timer)
{
	double period;
	/* Calcular Prescaler */
	/* Preciso ver um modo de ler o valor do prescaler*/

	/* Sele��o do Timer */
	if (timer == 0)
		period = TIMER32_1->LOAD * 256;
	else
		period = TIMER32_2->LOAD * 256;

	/* C�lculo do Periodo */
	period = period / (CS_getMCLK());
	return period;
}
double dr_T32_get_freq(uint32_t timer)
{
	double period;
	/* Calcular Prescaler */
	/* Preciso ver um modo de ler o valor do prescaler */

	/* Sele��o do Timer */
	if (timer == 0)
		period = TIMER32_1->LOAD;
	else
		period = TIMER32_2->LOAD;

	/* C�lculo do Periodo */
	period = CS_getMCLK() / period;
	return period;
}
void dr_I2C_Read(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
					uint8_t *data)
{
	/* Selecionar I2C */
	EUSCI_B_Type *moduleI2C;
	if (n_I2C == 0)
	{
		moduleI2C = EUSCI_B0;
	}
	else if (n_I2C == 1)
	{
		moduleI2C = EUSCI_B1;
	}
	else if (n_I2C == 2)
	{
		moduleI2C = EUSCI_B2;
	}
	else
	{
		moduleI2C = EUSCI_B3;
	}

	// dr_I2C_read() begin
	/* Read a single byte at memAddr
	 * read: S-(slaveAddr+w)-ACK-memAddr-ACK-R-(saddr+r)-ACK-data-NACK-P
	 */

	moduleI2C->I2CSA = slaveAddr; /* setup slave address */
	moduleI2C->CTLW0 |= 0x0010; /* enable transmitter */
	moduleI2C->CTLW0 |= 0x0002; /* generate START and send slave address */
	while ((moduleI2C->CTLW0 & 2))
		; /* wait until slave address is sent */
	moduleI2C->TXBUF = memAddr; /* send memory address to slave */
	while (!(moduleI2C->IFG & 2))
		; /* wait till it's ready to transmit */
	moduleI2C->CTLW0 &= ~0x0010; /* enable receiver */
	moduleI2C->CTLW0 |= 0x0002; /* generate RESTART and send slave address */
	while (moduleI2C->CTLW0 & 2)
		; /* wait till restart is finished */
	moduleI2C->CTLW0 |= 0x0004; /* setup to send STOP after the byte is received */
	while (!(moduleI2C->IFG & 1))
		; /* wait till data is received */
	*data = moduleI2C->RXBUF; /* read the received data */
	while (moduleI2C->CTLW0 & 4)
		; /* wait until STOP is sent */
	// dr_I2C_read() end
}

void dr_I2C_Write(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
					uint8_t data)
{
	/* Selecionar I2C */
	EUSCI_B_Type *moduleI2C;
	if (n_I2C == 0)
	{
		moduleI2C = EUSCI_B0;
	}
	else if (n_I2C == 1)
	{
		moduleI2C = EUSCI_B1;
	}
	else if (n_I2C == 2)
	{
		moduleI2C = EUSCI_B2;
	}
	else
	{
		moduleI2C = EUSCI_B3;
	}

	/* Write a single byte at memAddr
	 * write: S-(slaveAddr+w)-ACK-memAddr-ACK-data-ACK-P
	 */
	moduleI2C->I2CSA = slaveAddr; /* setup slave address */
	moduleI2C->CTLW0 |= 0x0010; /* enable transmitter */
	moduleI2C->CTLW0 |= 0x0002; /* generate START and send slave address */
	while ((moduleI2C->CTLW0 & 2))
		; /* wait until slave address is sent */
	moduleI2C->TXBUF = memAddr; /* send memory address to slave */
	while (!(moduleI2C->IFG & 2))
		; /* wait till it's ready to transmit */
	moduleI2C->TXBUF = data; /* send data to slave */
	while (!(moduleI2C->IFG & 2))
		; /* wait till last transmit is done */
	moduleI2C->CTLW0 |= 0x0004; /* send STOP */
	while (moduleI2C->CTLW0 & 4)
		; /* wait until STOP is sent */
}
void dr_I2C_init(uint8_t n_I2C)
{
	/* Selecionar I2C */
	uint32_t moduleI2C;
	if (n_I2C == 0)
	{
		moduleI2C = EUSCI_B0_BASE;
	}
	else if (n_I2C == 1)
	{
		moduleI2C = EUSCI_B1_BASE;
	}
	else if (n_I2C == 2)
	{
		moduleI2C = EUSCI_B2_BASE;
	}
	else
	{
		moduleI2C = EUSCI_B3_BASE;
	}

	/* I2C Master Configuration Parameter */
	const eUSCI_I2C_MasterConfig i2cConfig = {
	EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
												// SMCLK Clock Source
			12000000,
			// SMCLK = 12MHz
			EUSCI_B_I2C_SET_DATA_RATE_400KBPS,
			// Desired I2C Clock of 400khz
			0,
			// No byte counter threshold
			EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
			};
	/*Inicializar I2C - begin*/
	I2C_disableModule(moduleI2C);
	/* Initializing I2C Master to SMCLK at 400khz with no autostop */
	I2C_initMaster(moduleI2C, &i2cConfig);
	//	// 1. Write the slave register in ucbxi2csa
	//	I2C_setSlaveAddress(moduleI2C, 0x68);
	/* Enable I2C Module to start operations */
	I2C_enableModule(moduleI2C);
	/*Inicializar I2C - end*/
}
int dr_I2C_ReadRaw(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
					uint8_t byteCount, uint8_t *data)
{
	/* Selecionar I2C */ //0.001ms
	EUSCI_B_Type *moduleI2C;
	if (n_I2C == 0)
	{
		moduleI2C = EUSCI_B0;
	}
	else if (n_I2C == 1)
	{
		moduleI2C = EUSCI_B1;
	}
	else if (n_I2C == 2)
	{
		moduleI2C = EUSCI_B2;
	}
	else
	{
		moduleI2C = EUSCI_B3;
	}

	/* Read Raw begin */
	if (byteCount <= 0)
		return -1; /* no read was performed */

	moduleI2C->I2CSA = slaveAddr; /* setup slave address */
	moduleI2C->CTLW0 |= 0x0010; /* enable transmitter */
	moduleI2C->CTLW0 |= 0x0002; /* generate START and send slave address */
	while ((moduleI2C->CTLW0 & 2))
		; /* wait until slave address is sent */
	moduleI2C->TXBUF = memAddr; /* send memory address to slave */
	while (!(moduleI2C->IFG & 2))
		; /* wait till last transmit is done */
	moduleI2C->CTLW0 &= ~0x0010; /* enable receiver */
	moduleI2C->CTLW0 |= 0x0002; /* generate RESTART and send slave address */
	while (moduleI2C->CTLW0 & 2)
		; /* wait till RESTART is finished */

	/* receive data one byte at a time */
	do
	{
		if (byteCount == 1) /* when only one byte of data is left */
			moduleI2C->CTLW0 |= 0x0004; /* setup to send STOP after the last byte is received */

		while (!(moduleI2C->IFG & 1))
			; /* wait till data is received */
		*data++ = moduleI2C->RXBUF; /* read the received data */
		byteCount--;
	}
	while (byteCount);

	while (moduleI2C->CTLW0 & 4)
		; /* wait until STOP is sent */

	return 0; /* no error */
}
void dr_PMAP_configuration()
{
	/* Pmap Code / Avaible for port 2, 3 e 7 */
	PMAP->KEYID = 0x2D52; /* Unlock PMAP */
	/* Escolha dos novos pinos */
	P2MAP->PMAP_REGISTER5 = PMAP_UCB2SDA; /* UCB0SDA, MAP2.5 to ucb2_sda */
	P3MAP->PMAP_REGISTER0 = PMAP_UCB2SCL; /* UCB0SCL, MAP3.0 to ucb2_scl */

	/* atribui��o das fun��es prim�rias */
	GPIO_setAsPeripheralModuleFunctionInputPin(
	GPIO_PORT_P2,
												GPIO_PIN5,
												GPIO_PRIMARY_MODULE_FUNCTION);
	GPIO_setAsPeripheralModuleFunctionInputPin(
	GPIO_PORT_P3,
												GPIO_PIN0,
												GPIO_PRIMARY_MODULE_FUNCTION);
	/* Lock PMAP */
	PMAP->CTL = 1;
	PMAP->KEYID = 0;
}
void dr_I2C0_pin_config()
{
	/* Select Port 1 for I2C - Set Pin 6, 7 to input Primary Module Function,
	 *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL). and setting P5.5 for input mode
	 *   with pull-up enabled
	 */

	GPIO_setAsPeripheralModuleFunctionInputPin(
	GPIO_PORT_P1,
												GPIO_PIN6 + GPIO_PIN7,
												GPIO_PRIMARY_MODULE_FUNCTION);
}
