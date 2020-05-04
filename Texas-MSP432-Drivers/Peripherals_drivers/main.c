/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
//#include <driverlib.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <Drivers_Control.h>
int contador = 1;

void T32_interrupt_0()
{
	Timer32_clearInterruptFlag(TIMER32_0_BASE);
	printf("\n\r interruption Timer 32_0");
}
void T32_interrupt_1()
{
	Timer32_clearInterruptFlag(TIMER32_1_BASE);
	printf("\n\r interruption Timer 32_1");
}

int main(void)
{
	WDT_A_holdTimer();
	/*Inicialização dos periféricos e pinos */
	dr_Leds_sw_init();
	dr_Uart_init();
	dr_T32_init_seg(0, 1);
	dr_T32_init_Hz(1, 10);

	/* Inicialização das Interrupções */
	dr_Uart_interrupt_receive();
	dr_T32_interrupt_init(0, T32_interrupt_0);
	dr_T32_interrupt_init(1, T32_interrupt_1);
	/* Start no Timer */
	dr_T32_start(0);
	dr_T32_start(1);
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
		dr_Tick_start();
		contador = UART_receiveData(EUSCI_A0_BASE) - 48;
		printf("\n\rAtualizando o valor para os Leds em: %d ->>>>>><<<<---",
				contador);
		dr_Tick_stop();
		printf("\n\rPeriodo T32_0: %.2fseg", dr_T32_getPeriod_seg(0));
		printf("\n\rFrequencia T32_1: %.2fHz", dr_T32_get_freq(1));
	}
}

