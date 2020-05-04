/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <driverlib.h>

#include <stdint.h>
#include <stdbool.h>
#include <Drivers_Control.h>

int main(void)
{
	WDT_A_holdTimer();
	// Selecionar Função dos Pinos
	dr_Leds_sw_init();
	dr_Uart_init();
	while (1)
	{
		dr_Leds_alterar(1);
	}

}