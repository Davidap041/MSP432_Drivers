/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
//#include <driverlib.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <Drivers_Control.h>
int contador = 1;

/* Slave Address */
#define SLAVE_ADDRESS_1     0x68
#define I2C0 EUSCI_B0_BASE
#define NUM_OF_REC_BYTES 10

/* Statics */
const uint8_t TXData[] = { 0x75 };
static volatile uint32_t xferIndex;
static volatile bool stopSent;

static uint8_t RXData[NUM_OF_REC_BYTES];
uint32_t contador_RX = 0;
uint8_t RXData2;

/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig = {
EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
		12000000,                                // SMCLK = 12MHz
		EUSCI_B_I2C_SET_DATA_RATE_400KBPS,      // Desired I2C Clock of 400khz
		0,                                      // No byte counter threshold
		EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
		};

int main(void)
{

	WDT_A_holdTimer();
	dr_Pin_config_1();
	dr_Config_uart();
	printf("\r\nPrintf support for the launchpad\r\n");

	printf("Decimal(10) :%d\r\n", 10);
	printf("Hex(10)     :%x\r\n", 10);
	printf("float       :%f\r\n", 4.32);

	/* Select Port 1 for I2C - Set Pin 6, 7 to input Primary Module Function,
	 *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL). and setting P5.5 for input mode
	 *   with pull-up enabled
	 */
	GPIO_setAsPeripheralModuleFunctionInputPin(
	GPIO_PORT_P1,
												GPIO_PIN6 + GPIO_PIN7,
												GPIO_PRIMARY_MODULE_FUNCTION);

	stopSent = false;
	I2C_disableModule(I2C0);
	/* Initializing I2C Master to SMCLK at 400khz with no autostop */

	I2C_initMaster(I2C0, &i2cConfig);
	// 1. Write the slave register in ucbxi2csa
	I2C_setSlaveAddress(I2C0, SLAVE_ADDRESS_1);
	/* Enable I2C Module to start operations */
	I2C_enableModule(EUSCI_B0_BASE);

	xferIndex = 0;

	dr_Interrupt_uart();
	dr_Interrupt_on();

	while (1)
	{
		dr_Alterar_LEDS(contador);
		dr_delay_s(1);
		if (xferIndex < 9)
		{
			while (I2C_masterIsStopSent(EUSCI_B0_BASE))
				;
		}

		/* Send start and the first byte of the transmit buffer. */
		// seção que trava se o sensor não for reiniciado
		if (xferIndex == 0)
			I2C_masterSendMultiByteStart(EUSCI_B0_BASE, TXData[0]);

		/* Sent the first byte, now we need to initiate the read */
		if (xferIndex == 0)
			I2C_masterReceiveStart(EUSCI_B0_BASE);

		uint_fast16_t status = 1;

		/* Receives bytes into the receive buffer. If we have received all bytes,
		 * send a STOP condition */
		if (status & I2C_getInterruptStatus(EUSCI_B0_BASE,
		EUSCI_B_I2C_RECEIVE_INTERRUPT0))
		{
			if (xferIndex >= NUM_OF_REC_BYTES - 2)
			{
				/*
				 * Switch order so that stop is being set during reception of last
				 * byte read byte so that next byte can be read.
				 */
				RXData[xferIndex++] = I2C_masterReceiveMultiByteNext(
				EUSCI_B0_BASE);
				I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
				I2C_clearInterruptFlag(EUSCI_B0_BASE,
				EUSCI_B_I2C_RECEIVE_INTERRUPT0);

			}
			else
			{
				RXData[xferIndex++] = I2C_masterReceiveMultiByteNext(
				EUSCI_B0_BASE);

			}
		}
		else
		{

		}

	}

}

void EUSCIA0_IRQHandler(void)
{
	uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
	if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
	{
		dr_Start_tick();
		contador = UART_receiveData(EUSCI_A0_BASE) - 48;
		printf("\n\n\rAtualizando o valor para os Leds em: %d ->>>>>><<<<---",
				contador);
		dr_Stop_tick();
	}
}

