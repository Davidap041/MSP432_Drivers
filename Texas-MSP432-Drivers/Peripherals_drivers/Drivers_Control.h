/*
* Drivers_Control.h
*
*  Created on: 21 de mar de 2020
*      Author: davia
* Biblioteca para auxiliar o uso da SDK com alguns exemplos
*/
#ifndef DRIVERS_CONTROL_H_
#define DRIVERS_CONTROL_H_


//*****************************************************************************
//
//! \addtogroup gpio_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

// **************************************************************************************************************************************************************
// Includes necess�rios
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h> // usar essa defini��o para o CodeComposer
//#include <driverlib.h>                                   // Usar essa defini��o para o Visual Studio

/*
*************************"gpio.h"*****************************************
* GPIO_clearInterruptFlag - Limpa Flga de Interrup��o
* GPIO_disableInterrupt- Desabilita a Interrup��o
* GPIO_enableInterrupt - Habilita a Interrup��o -
* GPIO_getEnabledInterruptStatus - Ver o estado de "Habilita��o" da Flag
* GPIO_getInputPinValue - Ler o valor de Entrada do Pino
* GPIO_getInterruptStatus - Ler o estado de interrup��o do pino selecionado -
* GPIO_interruptEdgeSelect - Seleciona a Borda da Interrup�ao do pino -
* GPIO_registerInterrupt - Seleciona a rotina de Interrup��o da porta que ser� executada  -
* GPIO_unregisterInterrupt - Desabilita a rotina de interrup��o da porta configurada anteriormente -
* GPIO_setAsInputPin - Seleciona como pino de Entrada -
* GPIO_setAsInputPinWithPullDownResistor - Seleciona como pino de entrada e Resistor de Pull Down -
* GPIO_setAsInputPinWithPullUpResistor - Seleciona como pino de entrada e Resistor de Pull Up -
* GPIO_setAsOutputPin - Seleciona como pino de Sa�da -
*** Pinos podem ser para os m�dulos (p�g 35 Tabela 2.8 PxSel) sen�o por default ser�o simplie I/O (GPIO)
* GPIO_setAsPeripheralModuleFunctionInputPin - Seleciona o m�dulo funcional 1�,2� ou 3� como entrada -
* GPIO_setAsPeripheralModuleFunctionOutputPin - Seleciona o m�dulo funcional 1�,2� ou 3� como sa�da -
*** Set Drive Strength to selected port **??** (Procurar no DataSheet)
* GPIO_setDriveStrengthHigh - drivestrenght to High -
* GPIO_setDriveStrengthLow - drivestrenght to Low -
*** Opera��es com a sa�da dos pinos
* GPIO_setOutputHighOnPin - Seta o valor de sa�da HIGH para o Pino -
* GPIO_setOutputLowOnPin - Seta o valor de sa�da LOW para o Pino -
* GPIO_toggleOutputOnPin - Alterna o valor do pino de sa�da -
*/

/*
*************************"uart.h"*****************************************
* UART_initModule(uint32_t moduleInstance, const eUSCI_UART_Config *config) - Configura o m�dulo uart -
* UART_enableModule(uint32_t moduleInstance) - Habilita inicializando o m�dulo uart configurado -
* UART_disableModule(uint32_t moduleInstance) - Desabilita o m�dulo uart configurado -
* UART_transmitData(uint32_t moduleInstance, uint_fast8_t transmitData) - Transmite um byte pelo m�dulo uart -
* UART_receiveData(uint32_t moduleInstance) - Recebe um byte do m�dulo uart selecionado - ;
* UART_queryStatusFlags(uint32_t moduleInstance, uint_fast8_t mask) - Ler o estado atual da flag selecionada do m�dulo - ;
* UART_setDormant(uint32_t moduleInstance) - P�e o m�dulo em sleep mode, ( Only characters that are preceded by an idle-line or with address bit set UCRXIFG) - ;
* UART_resetDormant(uint32_t moduleInstance) - Desabilita o sleep mode (Not dormant. All received characters set UCRXIFG) -
* UART_transmitAddress(uint32_t moduleInstance, uint_fast8_t transmitAddress) - (Transmits the next byte to be transmitted marked as address depending on selected multiprocessor mode) -
* UART_transmitBreak(uint32_t moduleInstance) - (Transmit break. Transmits a break with the next write to the transmit buffer) -
* UART_getReceiveBufferAddressForDMA(uint32_t moduleInstance) - Retorna o endere�o do buffer rx do m�ulo DMA -
* UART_getTransmitBufferAddressForDMA(uint32_t moduleInstance) - (Returns the address of the TX Buffer of the UART for the DMA module) -
* UART_selectDeglitchTime(uint32_t moduleInstance, uint32_t deglitchTime) - (Sets the deglitch time, 2,50,100 ou 200ns) -
* UART_enableInterrupt(uint32_t moduleInstance, uint_fast8_t mask) - Habilita a interrup��o da UART para a mascara escolhida -
* UART_disableInterrupt(uint32_t moduleInstance, uint_fast8_t mask) - Desabilita a interrup��o da UART para a mascara escolhida - ;
* UART_getInterruptStatus(uint32_t moduleInstance, uint8_t mask) - Pega o estado de interrup��o da flag escolhida - ;
* UART_getEnabledInterruptStatus(uint32_t moduleInstance) - (Gets the current UART interrupt status masked with the enabled interrupts) - ;
* UART_clearInterruptFlag - Limpa a flag selecionda -
* UART_registerInterrupt UART_registerInterrupt(uint32_t moduleInstance, void (*intHandler)(void)) - Direciona a interrup��o para a rotina escolhida -
* UART_unregisterInterrupt(uint32_t moduleInstance) - (Unregisters the interrupt handler for the UART module)
**/
/*
*************************"systick.h"*****************************************
* SysTick_enableModule(void) - Habilita e inicia a contagem do m�dulo
* SysTick_disableModule(void) - Para a contagem do m�dulo
* SysTick_registerInterrupt(void (*intHandler)(void)) - Aponta para a rotina de interrup��o
* SysTick_unregisterInterrupt(void) - "Desaponta" a rotina de interrup��o do m�dulo
* SysTick_enableInterrupt(void) - Habilita a interrup��o de estouro da contagem
* SysTick_disableInterrupt(void) - Desabilita a interrup��o de estouro da contagem
* SysTick_setPeriod(uint32_t period) - Configura o n�mero de ticks m�ximo que come�a a contagem
* SysTick_getPeriod(void) - Retorna o valor do tick de in�cio de contagem programado
* SysTick_getValue(void) - Retonana o valor da contagem
* lembrando que � contagem decrescente em que o m�ximo � o valor do periodo
* SysTick->VAL = 0 - Para reiniciar a contagem de fato � preciso zerar esse registrador
**/







// **************************************************************************************************************************************************************
// Algumas defini��es para uso dos LEDs
#define dr_set_led1      GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0)
#define dr_set_RGB_red   GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0)
#define dr_set_RGB_green GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1)
#define dr_set_RGB_blue  GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2)
#define dr_clc_led1      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0)
#define dr_clc_RGB_red   GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0)
#define dr_clc_RGB_green GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1)
#define dr_clc_RGB_blue  GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2)
#define dr_read_SW1      !GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
#define dr_read_SW2      !GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4);

//*****************************************************************************
/*
* \brief Essa fun��o tem o prop�sito de testar as combina��es entre os dois leds
* \param contador define qual combina��o ser� usada
*      Valied Values are :
*      1
*      2
*      3
*      4
*      5
*      6
*      7
* \return None
*/
extern void dr_Alterar_LEDS(uint16_t contador);
//*****************************************************************************
/*
* \brief Essa fun��o tem como pr�sito fornecer um atraso de delay multiplicador por mil 
* 
* \param delay_k define qual valor de delay ser� usado
*      Tipic Values are : 
*      1
*      1.5
*      2
* \return None  
*/
extern void dr_delay_k(double maximo);
//*****************************************************************************
/*
* \brief Essa fun��o tem como pr�sito fornecer um atraso de n msegundos
*
* \param i define qual valor de delay em ms ser� usado
*      Tipic Values are :
*      1
*      2
*      ...
* \return None
*/
extern void dr_delay_ms(int n);
//*****************************************************************************
/*
* \brief Essa fun��o tem como pr�sito fornecer um atraso de n segundos
*
* \param i define qual valor de delay em seg ser� usado
*      Tipic Values are :
*      1
*      2
*      ...
* \return None
*/
extern void dr_delay_s(int n);
//*****************************************************************************
/*
* \brief Essa fun��o tem como pr�sito alterar os leds atrav�s das chaves
* deve ser posterior a fun��o PinConfig
* \param None
* \return None
*/
extern void dr_Alterar_LEDS_pela_SW();
//*****************************************************************************
/*
* \brief Essa fun��o tem como pr�sito configurar:
*   LEDS como sa�da ("posteriormente inicializando")
*   Chaves como entradas Pull Up
* deve ser posterior a fun��o PinConfig
* \param None
* \return None
*/
extern void dr_Pin_config_1();
//*****************************************************************************
/*
* \brief Essa fun��o tem como pr�sito enviar pela serial UART0 (PC) O PRINTF
* Deve ser usada em conjunto com a fputs
*/
extern int fputc(int _c, register FILE *_fp);
//*****************************************************************************
/*
* \brief Essa fun��o tem como pr�sito enviar pela serial UART0 (PC) O PRINTF
* Deve ser usada em conjunto com a fputc
*/
extern int fputs(const char *_ptr, register FILE *_fp);

//#define dr_BAUD_RATE_9600_Kbps
#define dr_BAUD_RATE_115200_Kbps

#ifdef dr_BAUD_RATE_9600_Kbps
#define clockPrescalar 78
#define firstModReg 2
#define secondModReg 0
#endif // dr_BAUD_RATE_9600_Kbps

#ifdef dr_BAUD_RATE_115200_Kbps
#define clockPrescalar 6
#define firstModReg 8
#define secondModReg 0
#endif // dr_BAUD_RATE_115200_Kbps

//*****************************************************************************
/*
* Fun��o de configura��o da UART:
* Configura��o dos pinos P1.2 e P1.3 como modo UART
* Configura��o do clock DCO em 12MHz
* Configura��o da UART0 (EUSCI)
*                 // set BAUD_RATE_9600_Kbps    //set BAUD_RATE_115200_Kbps
*                 // SMCLK Clock Source         //SMCLK Clock Source
*(clockPrescalar) // BRDIV = 78                 // BRDIV  = 6
*(firstModReg)    // UCxBRF = 2                 // UCxBRF = 8
*(secondModReg)   // UCxBRS = 0                 // UCxBRS = 0
* *** // No Parity  // LSB First    // One stop bit // UART mode ****
*(overSampling)   // Oversampling               // Oversampling
*     // 8 bit data length
* Posteriormente Habilta e inicializa o m�dulo
*
* UART Configuration Parameter. These are the configuration parameters to
* make the eUSCI A UART module to operate with a 115200 baud rate. These
* values were calculated using the online calculator that TI provides
* at:
* http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html

*/
extern void dr_Config_uart();
//*****************************************************************************
/*
* \brief Habilita a Interrup��o da UART0 (EUSCI_A0_BASE)
* para monitorar a flag de "recebimento" (EUSCI_A_UART_RECEIVE_INTERRUPT)
* de modo que quando recebe algo ela executa
* a rotina de interrup��o (default):void EUSCIA0_IRQHandler(void)
*/

extern void dr_Interrupt_uart();
//*****************************************************************************
/*
* \brief Habilita as Interrup��es que foram programadas at� ent�o
*/
extern void dr_Interrupt_on();
//*****************************************************************************
/*
* \brief Essa fun��o tem o prop�sito Iniciar uma contagem a partir do clock principal
* de 15MHz com o objetivo de calcular o tempo de execu��o de instru��es entre o
* Start_tick e o Stop_tick
* obs1: A contagem � parada quando � chamada a fun��o Stop_tick();
* obs2: O tempo m�ximo de entre as duas fun��es deve ser menor que 10 segundos
* ou seja, essa fun��o mede o tempo de processamento at� 10 segundo, caso esse valor seja
* ultrapassado (e a interrupt_on estiver habilitada) ser� mandado um printf indicando
* o estouro do tempo medido
*
* \param None
* \return Valor atual do Tick (Valor do in�cio da contagem)
* O valor atual do Tick � (valor m�ximo de tick programado - n�mero de ticks at� o get)
*/
extern uint32_t dr_Start_tick();
//*****************************************************************************
/*
* \brief Essa fun��o tem o prop�sito Parar a contagem e calcular o tempo entre o Start
* e o Stop
* Quando a fun��o Stop_tick � chamada � calculado o tempo gasto entre o Start e Stop em
* milisegundos e � "printado" na UART o valor correspondente
* \param None
* \return Valor de Ticks entre o Start e o Stop
*/
extern uint32_t dr_Stop_tick();
//*****************************************************************************
/*
* \brief Essa � a rotina de interrup��o pr� configurada que � chamado caso haja um estouro
* da contagem
*/
extern void SysTick_Handler(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif
#endif /* DRIVERS_CONTROL_H_ */

