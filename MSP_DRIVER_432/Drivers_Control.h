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
	// Includes necessários
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h> // usar essa definição para o CodeComposer
	//#include <driverlib.h>                                   // Usar essa definição para o Visual Studio

	/*
	*************************"gpio.h"*****************************************
	* Limpa Flga de Interrupção -   GPIO_clearInterruptFlag
	* Desabilita a Interrupção -    GPIO_disableInterrupt
	* Habilita a Interrupção -      GPIO_enableInterrupt
	* Ver o estado de "Habilitação" da Flag - GPIO_getEnabledInterruptStatus
	* Ler o valor de Entrada do Pino -        GPIO_getInputPinValue
	* Ler o estado de interrupção do pino selecionado - GPIO_getInterruptStatus
	* Seleciona a Borda da Interrupçao do pino -    GPIO_interruptEdgeSelect
	* Seleciona a rotina de Interrupção da porta que será executada  - GPIO_registerInterrupt
	* Desabilita a rotina de interrupção da porta configurada anteriormente - GPIO_unregisterInterrupt
	* Seleciona como pino de Entrada -  GPIO_setAsInputPin
	* Seleciona como pino de entrada e Resistor de Pull Down -  GPIO_setAsInputPinWithPullDownResistor
	* Seleciona como pino de entrada e Resistor de Pull Up -    GPIO_setAsInputPinWithPullUpResistor
	* Seleciona como pino de Saída -    GPIO_setAsOutputPin
	*** Pinos podem ser para os módulos (pág 35 Tabela 2.8 PxSel) senão por default serão simplie I/O (GPIO)
	* Seleciona o módulo funcional 1°,2° ou 3° como entrada -   GPIO_setAsPeripheralModuleFunctionInputPin
	* Seleciona o módulo funcional 1°,2° ou 3° como saída -     GPIO_setAsPeripheralModuleFunctionOutputPin
	*** Set Drive Strength to selected port **??** (Procurar no DataSheet)
	* drivestrenght to High -   GPIO_setDriveStrengthHigh
	* drivestrenght to Low -    GPIO_setDriveStrengthLow
	*** Operações com a saída dos pinos
	* Seta o valor de saída HIGH para o Pino -  GPIO_setOutputHighOnPin
	* Seta o valor de saída LOW para o Pino -   GPIO_setOutputLowOnPin
	* Alterna o valor do pino de saída -        GPIO_toggleOutputOnPin
	*/

	/*
	*************************"uart.h"*****************************************
	* Configura o módulo uart - UART UART_initModule(uint32_t moduleInstance, const eUSCI_UART_Config *config);
	* Habilita inicializando o módulo uart configurado -  UART_enableModule(uint32_t moduleInstance);
	* Desabilita o módulo uart configurado -  UART_disableModule(uint32_t moduleInstance);
	* Transmite um byte pelo módulo uart - UART_transmitData(uint32_t moduleInstance, uint_fast8_t transmitData);
	* Recebe um byte do módulo uart selecionado - UART_receiveData(uint32_t moduleInstance);
	* Ler o estado atual da flag selecionada do módulo - UART_queryStatusFlags(uint32_t moduleInstance, uint_fast8_t mask);
	* Põe o módulo em sleep mode, ( Only characters that are preceded by an idle-line or with address bit set UCRXIFG) - UART_setDormant(uint32_t moduleInstance);
	* Desabilita o sleep mode (Not dormant. All received characters set UCRXIFG) - UART_resetDormant(uint32_t moduleInstance);
	* (Transmits the next byte to be transmitted marked as address depending on selected multiprocessor mode) -  UART_transmitAddress(uint32_t moduleInstance, uint_fast8_t transmitAddress);
	* (Transmit break. Transmits a break with the next write to the transmit buffer) - UART_transmitBreak(uint32_t moduleInstance);
	* Retorna o endereço do buffer rx do móulo DMA - UART_getReceiveBufferAddressForDMA(uint32_t moduleInstance);
	* (Returns the address of the TX Buffer of the UART for the DMA module) - UART_getTransmitBufferAddressForDMA(uint32_t moduleInstance);
	* (Sets the deglitch time, 2,50,100 ou 200ns) - UART_selectDeglitchTime(uint32_t moduleInstance, uint32_t deglitchTime);
	* Habilita a interrupção da UART para a mascara escolhida - UART_enableInterrupt(uint32_t moduleInstance, uint_fast8_t mask);
	* Desabilita a interrupção da UART para a mascara escolhida - UART_disableInterrupt(uint32_t moduleInstance, uint_fast8_t mask);
	* Pega o estado de interrupção da flag escolhida - UART_getInterruptStatus(uint32_t moduleInstance, uint8_t mask);
	* (Gets the current UART interrupt status masked with the enabled interrupts) - UART_getEnabledInterruptStatus(uint32_t moduleInstance);
	* Limpa a flag selecionda - UART_clearInterruptFlag
	* Direciona a interrupção para a rotina escolhida - UART_registerInterrupt UART_registerInterrupt(uint32_t moduleInstance, void (*intHandler)(void));
	* (Unregisters the interrupt handler for the UART module) - UART_unregisterInterrupt(uint32_t moduleInstance);
	**/







	// **************************************************************************************************************************************************************
	// Algumas definições para uso dos LEDs
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
	* \brief Essa função tem o propósito de testar as combinações entre os dois leds
	* \param contador define qual combinação será usada
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
	* \brief Essa função tem como prósito fornecer um atraso de delay multiplicador por mil 
	* 
	* \param delay_k define qual valor de delay será usado
	*      Tipic Values are : 
	*      1
	*      1.5
	*      2
	* \return None  
	*/
	extern void delay_k(double maximo);
	//*****************************************************************************
	/*
	* \brief Essa função tem como prósito alterar os leds através das chaves
	* deve ser posterior a função PinConfig
	* \param None
	* \return None
	*/
	extern void Alterar_LEDS_pela_SW();
	//*****************************************************************************
	/*
	* \brief Essa função tem como prósito configurar:
	*   LEDS como saída ("posteriormente inicializando")
	*   Chaves como entradas Pull Up
	* deve ser posterior a função PinConfig
	* \param None
	* \return None
	*/
	extern void Pin_config_1();
	//*****************************************************************************
	/*
	* \brief Essa função tem como prósito enviar pela serial UART0 (PC) O PRINTF
	* Deve ser usada em conjunto com a fputs
	*/
	extern int fputc(int _c, register FILE *_fp);
	//*****************************************************************************
	/*
	* \brief Essa função tem como prósito enviar pela serial UART0 (PC) O PRINTF
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
	        * Função de configuração da UART:
	        * Configuração dos pinos P1.2 e P1.3 como modo UART
	        * Configuração do clock DCO em 12MHz
	        * Configuração da UART0 (EUSCI)
	        *                 // set BAUD_RATE_9600_Kbps    //set BAUD_RATE_115200_Kbps
	        *                 // SMCLK Clock Source         //SMCLK Clock Source
	        *(clockPrescalar) // BRDIV = 78                 // BRDIV  = 6
	        *(firstModReg)    // UCxBRF = 2                 // UCxBRF = 8
	        *(secondModReg)   // UCxBRS = 0                 // UCxBRS = 0
	        * *** // No Parity  // LSB First    // One stop bit // UART mode ****
	        *(overSampling)   // Oversampling               // Oversampling
	        *     // 8 bit data length
	        * Posteriormente Habilta e inicializa o módulo
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

