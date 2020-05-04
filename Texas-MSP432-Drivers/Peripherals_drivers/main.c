/* DriverLib Includes */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <Drivers_Control.h>

uint8_t RXData[2];
uint8_t leitura[14];
uint8_t contador = 1;

void dr_Inicializar_sensor()
{
	GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN1);
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN1);
	dr_Delay_ms(500);
	GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN1);
	dr_Delay_ms(500);

}
int main(void)
{

	WDT_A_holdTimer();
	/* Pin Config */
	dr_Leds_sw_init();
//	dr_PMAP_configuration();
	dr_I2C0_pin_config();
	/* Peripherals Config */
	dr_Uart_init();
	dr_I2C_init(0);

	dr_Uart_interrupt_receive();
	dr_Interrupt_on();
	dr_Inicializar_sensor();
	dr_I2C_Read(0, 0x69, 0x75, &RXData[0]);
	//Acelerometro mede a resolução +-2g
	dr_I2C_Write(0, 0x69, 0x1C, 0b00000000);
	//pequeno filtro digital
	dr_I2C_Write(0, 0x69, 0x1A, 0b00000001);
	//liga interrupção dado pronto
	dr_I2C_Write(0, 0x69, 0x38, 0b00000001);
	//DESLIGA GYRO	//data = 0b00000111;
	dr_I2C_Write(0, 0x69, 0x6C, 0b00000000);
	//configura Gyro em  ± 250 °/s
	dr_I2C_Write(0, 0x69, 0x1B, 0b00000000);
	//Divisor de clock - amostragem //OR = 1Khz / (1+data)
	dr_I2C_Write(0, 0x69, 0x19, 24);
	//ACORDA
	dr_I2C_Write(0, 0x69, 0x6B, 0b00000000);

	dr_I2C_Read(0, 0x69, 0x75, &RXData[1]);

	while (1)
	{
		dr_Leds_alterar(contador);
		dr_Delay_s(1);
	}
}

void EUSCIA0_IRQHandler(void)
{
	uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
	if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
	{
		contador = UART_receiveData(EUSCI_A0_BASE) - 48;
		printf("\n\n\r-------Atualizando o valor para os Leds em: %d ---------",
				contador);
		if (contador == 2)
		{
			int status_erro;
			dr_Tick_start();
			status_erro = dr_I2C_ReadRaw(0, 0x69, 0x3B, 14, leitura);
			dr_Tick_stop();
			printf("\n\rRetorno do Status erro: %d", status_erro);
		}

		int j;
		for (j = 0; j < 2; j++)
			printf("\n\rRxData[%d]:%x", j, RXData[j]);
		for (j = 0; j < 14; j++)
			printf("\n\rleitura[%d]:%x", j, leitura[j]);
	}

}
