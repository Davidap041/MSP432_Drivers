/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
//#include <driverlib.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <Drivers_Control.h>
int contador = 1;

int main(void)
{

    WDT_A_holdTimer();
    // Selecionar Função dos Pinos
    dr_Pin_config_1();
    dr_Config_uart();
    printf("\r\nPrintf support for the launchpad\r\n");

    printf("Decimal(10) :%d\r\n", 10);
    printf("Hex(10)     :%x\r\n", 10);
    printf("float       :%f\r\n", 4.32);

    dr_Interrupt_uart();
    dr_Interrupt_on();
    while (1)
    {
        dr_Alterar_LEDS(contador);
        dr_delay_s(2);
    }
}
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        dr_Start_tick();
        contador = UART_receiveData(EUSCI_A0_BASE) - 48;
        printf("\n\rAtualizando o valor para os Leds em: %d ->>>>>><<<<---",
               contador);
        dr_Stop_tick();
    }
}

