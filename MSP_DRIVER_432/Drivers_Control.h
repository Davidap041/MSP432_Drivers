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
	* Limpa Flga de Interrup��o -   GPIO_clearInterruptFlag
	* Desabilita a Interrup��o -    GPIO_disableInterrupt
	* Habilita a Interrup��o -      GPIO_enableInterrupt
	* Ver o estado de "Habilita��o" da Flag - GPIO_getEnabledInterruptStatus
	* Ler o valor de Entrada do Pino -        GPIO_getInputPinValue
	* Ler o estado de interrup��o do pino selecionado - GPIO_getInterruptStatus
	* Seleciona a Borda da Interrup�ao do pino -    GPIO_interruptEdgeSelect
	* Seleciona a rotina de Interrup��o da porta que ser� executada  - GPIO_registerInterrupt
	* Desabilita a rotina de interrup��o da porta configurada anteriormente - GPIO_unregisterInterrupt
	* Seleciona como pino de Entrada -  GPIO_setAsInputPin
	* Seleciona como pino de entrada e Resistor de Pull Down -  GPIO_setAsInputPinWithPullDownResistor
	* Seleciona como pino de entrada e Resistor de Pull Up -    GPIO_setAsInputPinWithPullUpResistor
	* Seleciona como pino de Sa�da -    GPIO_setAsOutputPin
	*** Pinos podem ser para os m�dulos (p�g 35 Tabela 2.8 PxSel) sen�o por default ser�o simplie I/O (GPIO)
	* Seleciona o m�dulo funcional 1�,2� ou 3� como entrada -   GPIO_setAsPeripheralModuleFunctionInputPin
	* Seleciona o m�dulo funcional 1�,2� ou 3� como sa�da -     GPIO_setAsPeripheralModuleFunctionOutputPin
	*** Set Drive Strength to selected port **??** (Procurar no DataSheet)
	* drivestrenght to High -   GPIO_setDriveStrengthHigh
	* drivestrenght to Low -    GPIO_setDriveStrengthLow
	*** Opera��es com a sa�da dos pinos
	* Seta o valor de sa�da HIGH para o Pino -  GPIO_setOutputHighOnPin
	* Seta o valor de sa�da LOW para o Pino -   GPIO_setOutputLowOnPin
	* Alterna o valor do pino de sa�da -        GPIO_toggleOutputOnPin
	*/

	/*
	*************************"uart.h"*****************************************
	* Configura o m�dulo uart - UART UART_initModule(uint32_t moduleInstance, const eUSCI_UART_Config *config);
	* Habilita inicializando o m�dulo uart configurado -  UART_enableModule(uint32_t moduleInstance);
	* Desabilita o m�dulo uart configurado -  UART_disableModule(uint32_t moduleInstance);
	* Transmite um byte pelo m�dulo uart - UART_transmitData(uint32_t moduleInstance, uint_fast8_t transmitData);
	* Recebe um byte do m�dulo uart selecionado - UART_receiveData(uint32_t moduleInstance);
	* Ler o estado atual da flag selecionada do m�dulo - UART_queryStatusFlags(uint32_t moduleInstance, uint_fast8_t mask);
	* P�e o m�dulo em sleep mode, ( Only characters that are preceded by an idle-line or with address bit set UCRXIFG) - UART_setDormant(uint32_t moduleInstance);
	* Desabilita o sleep mode (Not dormant. All received characters set UCRXIFG) - UART_resetDormant(uint32_t moduleInstance);
	* (Transmits the next byte to be transmitted marked as address depending on selected multiprocessor mode) -  UART_transmitAddress(uint32_t moduleInstance, uint_fast8_t transmitAddress);
	* (Transmit break. Transmits a break with the next write to the transmit buffer) - UART_transmitBreak(uint32_t moduleInstance);
	* Retorna o endere�o do buffer rx do m�ulo DMA - UART_getReceiveBufferAddressForDMA(uint32_t moduleInstance);
	* (Returns the address of the TX Buffer of the UART for the DMA module) - UART_getTransmitBufferAddressForDMA(uint32_t moduleInstance);
	* (Sets the deglitch time, 2,50,100 ou 200ns) - UART_selectDeglitchTime(uint32_t moduleInstance, uint32_t deglitchTime);
	* Habilita a interrup��o da UART para a mascara escolhida - UART_enableInterrupt(uint32_t moduleInstance, uint_fast8_t mask);
	* Desabilita a interrup��o da UART para a mascara escolhida - UART_disableInterrupt(uint32_t moduleInstance, uint_fast8_t mask);
	* Pega o estado de interrup��o da flag escolhida - UART_getInterruptStatus(uint32_t moduleInstance, uint8_t mask);
	* (Gets the current UART interrupt status masked with the enabled interrupts) - UART_getEnabledInterruptStatus(uint32_t moduleInstance);
	* Limpa a flag selecionda - UART_clearInterruptFlag
	* Direciona a interrup��o para a rotina escolhida - UART_registerInterrupt UART_registerInterrupt(uint32_t moduleInstance, void (*intHandler)(void));
	* (Unregisters the interrupt handler for the UART module) - UART_unregisterInterrupt(uint32_t moduleInstance);
	**/







	// **************************************************************************************************************************************************************
	// Algumas defini��es para uso dos LEDs
#define set_led1      GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0)
#define set_RGB_red   GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0)
#define set_RGB_green GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1)
#define set_RGB_blue  GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2)
#define clc_led1      GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0)
#define clc_RGB_red   GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0)
#define clc_RGB_green GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1)
#define clc_RGB_blue  GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2)
#define read_SW1      !GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
#define read_SW2      !GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4);



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
	extern void Alterar_LEDS(uint16_t contador);
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
	extern void delay_k(double maximo);
	//*****************************************************************************
	/*
	* \brief Essa fun��o tem como pr�sito alterar os leds atrav�s das chaves
	* deve ser posterior a fun��o PinConfig
	* \param None
	* \return None
	*/
	extern void Alterar_LEDS_pela_SW();
	//*****************************************************************************
	/*
	* \brief Essa fun��o tem como pr�sito configurar:
	*   LEDS como sa�da ("posteriormente inicializando")
	*   Chaves como entradas Pull Up
	* deve ser posterior a fun��o PinConfig
	* \param None
	* \return None
	*/
	extern void Pin_config_1();
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

	//#define BAUD_RATE_9600_Kbps
#define BAUD_RATE_115200_Kbps

#ifdef BAUD_RATE_9600_Kbps
#define clockPrescalar 78
#define firstModReg 2
#define secondModReg 0
#endif // BAUD_RATE_9600_Kbps

#ifdef BAUD_RATE_115200_Kbps
#define clockPrescalar 6
#define firstModReg 8
#define secondModReg 0
#endif // BAUD_RATE_115200_Kbps

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
	        * UART Configuration Parameter. These are the configuration parameters to
	        * make the eUSCI A UART module to operate with a 115200 baud rate. These
	        * values were calculated using the online calculator that TI provides
	        * at:
	        * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
	        */
	        extern void config_uart();

	extern void interrupt_uart();






	//*****************************************************************************
	//
	// Mark the end of the C bindings section for C++ compilers.
	//
	//*****************************************************************************
#ifdef __cplusplus
}
#endif
#endif /* DRIVERS_CONTROL_H_ */

