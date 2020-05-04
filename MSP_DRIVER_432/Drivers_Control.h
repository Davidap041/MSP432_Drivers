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
	*******************************************"gpio.h"*************************************************************
	* GPIO_clearInterruptFlag - Limpa Flga de Interrupção
	* GPIO_disableInterrupt- Desabilita a Interrupção
	* GPIO_enableInterrupt - Habilita a Interrupção -
	* GPIO_getEnabledInterruptStatus - Ver o estado de "Habilitação" da Flag
	* GPIO_getInputPinValue - Ler o valor de Entrada do Pino
	* GPIO_getInterruptStatus - Ler o estado de interrupção do pino selecionado -
	* GPIO_interruptEdgeSelect - Seleciona a Borda da Interrupçao do pino -
	* GPIO_registerInterrupt - Seleciona a rotina de Interrupção da porta que será executada  -
	* GPIO_unregisterInterrupt - Desabilita a rotina de interrupção da porta configurada anteriormente -
	* GPIO_setAsInputPin - Seleciona como pino de Entrada -
	* GPIO_setAsInputPinWithPullDownResistor - Seleciona como pino de entrada e Resistor de Pull Down -
	* GPIO_setAsInputPinWithPullUpResistor - Seleciona como pino de entrada e Resistor de Pull Up -
	* GPIO_setAsOutputPin - Seleciona como pino de Saída -
	*** Pinos podem ser para os módulos (pág 35 Tabela 2.8 PxSel) senão por default serão simplie I/O (GPIO)
	* GPIO_setAsPeripheralModuleFunctionInputPin - Seleciona o módulo funcional 1°,2° ou 3° como entrada -
	* GPIO_setAsPeripheralModuleFunctionOutputPin - Seleciona o módulo funcional 1°,2° ou 3° como saída -
	*** Set Drive Strength to selected port **??** (Procurar no DataSheet)
	* GPIO_setDriveStrengthHigh - drivestrenght to High -
	* GPIO_setDriveStrengthLow - drivestrenght to Low -
	*** Operações com a saída dos pinos
	* GPIO_setOutputHighOnPin - Seta o valor de saída HIGH para o Pino -
	* GPIO_setOutputLowOnPin - Seta o valor de saída LOW para o Pino -
	* GPIO_toggleOutputOnPin - Alterna o valor do pino de saída -
	*/
	/*
	*************************"uart.h"*****************************************
	* UART_initModule(uint32_t moduleInstance, const eUSCI_UART_Config *config) - Configura o módulo uart -
	* UART_enableModule(uint32_t moduleInstance) - Habilita inicializando o módulo uart configurado -
	* UART_disableModule(uint32_t moduleInstance) - Desabilita o módulo uart configurado -
	* UART_transmitData(uint32_t moduleInstance, uint_fast8_t transmitData) - Transmite um byte pelo módulo uart -
	* UART_receiveData(uint32_t moduleInstance) - Recebe um byte do módulo uart selecionado - ;
	* UART_queryStatusFlags(uint32_t moduleInstance, uint_fast8_t mask) - Ler o estado atual da flag selecionada do módulo - ;
	* UART_setDormant(uint32_t moduleInstance) - Põe o módulo em sleep mode, ( Only characters that are preceded by an idle-line or with address bit set UCRXIFG) - ;
	* UART_resetDormant(uint32_t moduleInstance) - Desabilita o sleep mode (Not dormant. All received characters set UCRXIFG) -
	* UART_transmitAddress(uint32_t moduleInstance, uint_fast8_t transmitAddress) - (Transmits the next byte to be transmitted marked as address depending on selected multiprocessor mode) -
	* UART_transmitBreak(uint32_t moduleInstance) - (Transmit break. Transmits a break with the next write to the transmit buffer) -
	* UART_getReceiveBufferAddressForDMA(uint32_t moduleInstance) - Retorna o endereço do buffer rx do móulo DMA -
	* UART_getTransmitBufferAddressForDMA(uint32_t moduleInstance) - (Returns the address of the TX Buffer of the UART for the DMA module) -
	* UART_selectDeglitchTime(uint32_t moduleInstance, uint32_t deglitchTime) - (Sets the deglitch time, 2,50,100 ou 200ns) -
	* UART_enableInterrupt(uint32_t moduleInstance, uint_fast8_t mask) - Habilita a interrupção da UART para a mascara escolhida -
	* UART_disableInterrupt(uint32_t moduleInstance, uint_fast8_t mask) - Desabilita a interrupção da UART para a mascara escolhida - ;
	* UART_getInterruptStatus(uint32_t moduleInstance, uint8_t mask) - Pega o estado de interrupção da flag escolhida - ;
	* UART_getEnabledInterruptStatus(uint32_t moduleInstance) - (Gets the current UART interrupt status masked with the enabled interrupts) - ;
	* UART_clearInterruptFlag - Limpa a flag selecionda -
	* UART_registerInterrupt UART_registerInterrupt(uint32_t moduleInstance, void (*intHandler)(void)) - Direciona a interrupção para a rotina escolhida -
	* UART_unregisterInterrupt(uint32_t moduleInstance) - (Unregisters the interrupt handler for the UART module)
	**/
	/*
	*************************"systick.h"*****************************************
	* SysTick_enableModule(void) - Habilita e inicia a contagem do módulo
	* SysTick_disableModule(void) - Para a contagem do módulo
	* SysTick_registerInterrupt(void (*intHandler)(void)) - Aponta para a rotina de interrupção
	* SysTick_unregisterInterrupt(void) - "Desaponta" a rotina de interrupção do módulo
	* SysTick_enableInterrupt(void) - Habilita a interrupção de estouro da contagem
	* SysTick_disableInterrupt(void) - Desabilita a interrupção de estouro da contagem
	* SysTick_setPeriod(uint32_t period) - Configura o número de ticks máximo que começa a contagem
	* SysTick_getPeriod(void) - Retorna o valor do tick de início de contagem programado
	* SysTick_getValue(void) - Retonana o valor da contagem
	* lembrando que é contagem decrescente em que o máximo é o valor do periodo
	* SysTick->VAL = 0 - Para reiniciar a contagem de fato é preciso zerar esse registrador
	**/
	/*
	*************************"timer32.h"*****************************************
	* Timer32_initModule - instancia(0,1); prescaler(1,15,256);ativa resolução e modo de operação
	* Timer32_setCount(uint32_t timer, uint32_t count) - se no modo periódico ele muda o periodo se no free run
	* ( Sets the count of the timer and resets the current value to the value passed. This value is set on the next rising edge of the clock provided to the timer module)
	* Timer32_setCountInBackground - seta o valor do current valor sem resetar
	* Timer32_getValue - (Timer32_getValue)
	* Timer32_startTimer - dar início ao timer pre configurado
	* Timer32_haltTimer - Para a contagem preservando os parâmetros
	* Timer32_enableInterrupt - Habilita a interrupção do Timer
	* Timer32_disableInterrupt - desabilita a interrupção do timer
	* Timer32_clearInterruptFlag - Limpa a flag de interrupção do timer
	* Timer32_getInterruptStatus - pega o status da interrupção do timer
	* Timer32_registerInterrupt - Registra uma chamada de interrupção para o timer selecionado
	* Timer32_unregisterInterrupt - "Desregistra" a chamada de interrupção para o timer selecionado
	*/
	/*
	 **************************"cs.h"*********************************************************
	 * CS_setExternalClockSourceFrequency - Seta a frequencia para o uso de cristais externos do LFXT e HFXT
	 * CS_initClockSignal - Inicializa os clocks do sistema, escolhendo prescalers e osciladores
	 * CS_startHFXT - Inicializa o HFXT oscilador previamente configurado com a frequência dita
	 * CS_startHFXTWithTimeout - This function has a timeout associated with stabilizing the oscillator.
	 * CS_startLFXT - Inicializa o LFXT oscilador previamente configurado com a frequência dita
	 * CS_startLFXTWithTimeout - This function has a timeout associated with stabilizing the oscillator.
	 * CS_setReferenceOscillatorFrequency - Seta a frequência do oscilador interno [REFO] em 32kHz ou 128kHz
	 * CS_enableClockRequest - Habilita a requisição condicional para uso dos clocks[MCLK,ACLK,HSMCLK,SMCLK]
	 * CS_disableClockRequest - Desabilita a requisição condicional para uso dos clocks[MCLK,ACLK,HSMCLK,SMCLK]
	 * CS_getACLK - Get the current ACLK frequency.
	 * CS_getSMCLK - Get the current SMCLK frequency.
	 * CS_getMCLK - Get the current MCLK frequency.
	 * CS_getBCLK - Get the current BCLK frequency.
	 * CS_getHSMCLK - Get the current HSMCLK frequency.
	 * CS_setDCOCenteredFrequency - Sets the centered frequency of DCO operation. Each frequency represents
	 the centred frequency of a particular frequency range. Further tuning can be achieved by using the CS_tuneDCOFrequency function.
	  * CS_setDCOFrequency - Automatically sets/tunes the DCO to the given frequency.
	  * CS_tuneDCOFrequency - Tunes the DCO to a specific frequency.
	  * CS_enableDCOExternalResistor -  Enables the external resistor for DCO operation
	  * CS_disableDCOExternalResistor -  Disables the external resistor for DCO operation
	  * CS_setDCOExternalResistorCalibration - ets the calibration value for the DCO when using the external resistor
	  * CS_getDCOFrequency -  Gets the current tuned DCO frequency.
	  * CS_enableFaultCounter - Enables the fault counter for the CS module. [HFXT , LFXT]
	  * CS_disableFaultCounter -  Disables the fault counter for the CS module. [HFXT , LFXT]
	  * CS_resetFaultCounter -  Resets the fault counter for the CS module. [HFXT , LFXT]
	  * CS_startFaultCounter - Sets the count for the start value of the fault counter.
	  * CS_enableInterrupt - Enables individual clock control interrupt sources.[HFXT, LFXT, DCO]
	  * CS_disableInterrupt - Disables individual clock control interrupt sources.[HFXT, LFXT, DCO]
	  * CS_getEnabledInterruptStatus - Gets the current interrupt status masked with the enabled interrupts.
	  * CS_getInterruptStatus -  Gets the current interrupt status.
	  * CS_clearInterruptFlag - Clears clock system interrupt sources.
	  * CS_registerInterrupt - Registers an interrupt handler for the clock system interrupt.
	  * CS_unregisterInterrupt - Unregisters the interrupt handler for the clock system.
	  **/
	  /*
	   **************************timer_a.h*************************************************
	   * Timer_A_startCounter - Inicializa o contador, e o modo de operação.
	   * Timer_A_configureContinuousMode - (em modo contínuo) Configura a fonte de clk, interrupção, prescaler.
	   * Timer_A_configureUpMode -  (em modo rampa) Configura a fonte de clk, interrupção, prescaler.
	   * Timer_A_configureUpDownMode - (em modo triangular) Configura a fonte de clk, interrupção, prescaler.
	   * Timer_A_initCapture - (em modo capture) Configura a fonte de clk, interrupção, prescaler.
	   * Timer_A_initCompare - (em modo compare) Configura a fonte de clk, interrupção, prescaler.
	   * Timer_A_clearTimer - Reset/Clear the timer clock divider, direction e count
	   * Timer_A_getSynchronizedCaptureCompareInput - Get synchronized capture compare input
	   * Timer_A_getOutputForOutputModeOutBitValue - Get output bit for output mode
	   * Timer_A_getCaptureCompareCount - Get current capture compare count
	   * Timer_A_setOutputForOutputModeOutBitValue - Set ouput bit for output mode
	   * Timer_A_generatePWM - Configura o pwm -
	   * Timer_A_stopTimer - Stops the timer
	   * Timer_A_setCompareValue - Sets the value of the capture-compare register
	   * Timer_A_getCounterValue - Returns the current value of the specified timer.
	   * Timer_A_clearInterruptFlag - Clears the Timer TAIFG interrupt flag
	   * Timer_A_clearCaptureCompareInterrupt - Clears the capture-compare interrupt flag
	   * Timer_A_enableInterrupt - Enable timer interrupt
	   * Timer_A_disableInterrupt - Disable timer interrupt
	   * Timer_A_getInterruptStatus - Get timer interrupt status
	   * Timer_A_getEnabledInterruptStatus - Get timer interrupt status masked with the enabled interrupts.
	   * Timer_A_enableCaptureCompareInterrupt - Enable capture compare interrupt
	   * Timer_A_disableCaptureCompareInterrupt - Disable capture compare interrupt
	   * Timer_A_getCaptureCompareInterruptStatus - Return capture compare interrupt status
	   * Timer_A_getCaptureCompareEnabledInterruptStatus - Return capture compare interrupt status masked with the enabled interrupts.
	   * Timer_A_registerInterrupt - Registers an interrupt handler for the timer capture compare interrupt.
	   * Timer_A_unregisterInterrupt - Unregisters the interrupt handler for the timer
	   *
	   *********************************** "i2c.h" *************************************************************************
	   * I2C_enableModule - Enables the I2C block
	   * I2C_enableMultiMasterMode - Enables Multi Master Mode
	   * I2C_disableMultiMasterMode - Disables Multi Master Mode
	   * I2C_setSlaveAddress - Sets the address that the I2C Master will place on the bus.
	   * I2C_setMode - Sets the mode of the I2C device [EUSCI_B_I2C_TRANSMIT_MODE,EUSCI_B_I2C_RECEIVE_MODE]
	   * I2C_initMaster - Initializes the I2C Master block and configure [clk source, data rate, CounterThreshold, autoSTOPGeneration]

	   * I2C_disableModule - Disables the I2C block.
	   * I2C_getMode - Gets the mode of the I2C device return[0(receive mode),1?(transimit mode )]
	   * I2C_isBusBusy - Indicates whether or not the I2C bus is busy return [0(EUSCI_B_I2C_BUS_NOT_BUSY),1(EUSCI_B_I2C_BUS_BUSY]
	   * I2C_masterIsStopSent - indica se o stop foi enviado
	   * I2C_masterIsStartSent - indica de o start foi enviado \return Returns true if the START has been sent, false if it is sending

	   * I2C_masterSendStart - This function is used by the Master module to initiate START
	   * I2C_masterSendSingleByte - Does single byte transmission from Master to Slave [Sends START, Transmits the byte to the Slave, Sends STOP] \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
	   * I2C_masterSendMultiByteStart - Starts multi-byte transmission from Master to Slave
	   * I2C_masterSendMultiByteNext - Continues multi-byte transmission from Master to Slave
	   * I2C_masterSendMultiByteFinish - Finishes multi-byte transmission from Master to Slave This function Transmits the last data byte of a multi-byte transmission to the Slave and Sends STOP
	   * I2C_masterSendMultiByteStop - Send STOP byte at the end of a multi-byte transmission from Master to Slave

	   * I2C_masterReceiveStart -  Starts reception at the Master end This function is used by the Master module initiate reception of a single byte. This function Sends START
	   * I2C_masterReceiveSingleByte - Does single byte reception from the slave This function is used by the Master module to receive a single byte. This function: Sends START and STOP Waits for data reception Receives one byte from the Slave
	   * I2C_masterReceiveSingle - Receives a byte that has been sent to the I2C Master Module. This function reads a byte of data from the I2C receive data Register.
	   * I2C_masterReceiveMultiByteNext - Starts multi-byte reception at the Master end one byte at a time This function is used by the Master module to receive each byte of a multi-byte reception This function reads currently received byte
	   * I2C_masterReceiveMultiByteFinish - Finishes multi-byte reception at the Master end - Receives the current byte and initiates the STOP from Master to Slave
	   * I2C_masterReceiveMultiByteStop - Sends the STOP at the end of a multi-byte reception at the Master end - This function is used by the Master module to initiate STOP

	   * I2C_getReceiveBufferAddressForDMA - Returns the address of the RX Buffer of the I2C for the DMA module.Returns the address of the I2C RX Buffer. This can be used in conjunction with the DMA to store the received data directly to memory.
	   * I2C_getTransmitBufferAddressForDMA - Returns the address of the TX Buffer of the I2C for the DMA module. Returns the address of the I2C TX Buffer. This can be used in conjunction with the DMA to obtain transmitted data directly from memory.

	   * I2C_enableInterrupt - [EUSCI_B_I2C_STOP_INTERRUPT, EUSCI_B_I2C_START_INTERRUPT,EUSCI_B_I2C_TRANSMIT_INTERRUPT0, EUSCI_B_I2C_RECEIVE_INTERRUPT0 , EUSCI_B_I2C_NAK_INTERRUPT]
	   * I2C_disableInterrupt - [EUSCI_B_I2C_STOP_INTERRUPT, EUSCI_B_I2C_START_INTERRUPT,EUSCI_B_I2C_TRANSMIT_INTERRUPT0, EUSCI_B_I2C_RECEIVE_INTERRUPT0 , EUSCI_B_I2C_NAK_INTERRUPT]
	   * I2C_clearInterruptFlag - Clears I2C interrupt sources - Modified register is \b UCBxIFG.
	   * I2C_getInterruptStatus - Gets the current I2C interrupt status.
	   * I2C_getEnabledInterruptStatus - Gets the current I2C interrupt status masked with the enabled interrupts. This function is useful to call in ISRs to get a list of pending interrupts that are actually enabled and could have caused the ISR.
	   * I2C_registerInterrupt - Registers an interrupt handler for I2C interrupts.
	   * I2C_unregisterInterrupt
	   *
	   * I2C_initSlave - Initializes the I2C Slave block and configure param
	   * I2C_slaveSendNAK - This function is used by the slave to send a NAK out over the I2C line
	   * I2C_slavePutData - Transmits a byte from the I2C Module [Modified register is \b UCBxTXBUF register]
	   * I2C_slaveGetData - Receives a byte that has been sent to the I2C Module return [UCBxRXBUF register]
	   *
	   * I2C_masterSendSingleByteWithTimeout - Does single byte transmission from Master to Slave with timeout \return 0x01 or 0x00URE of the transmission process.
	   * I2C_masterSendMultiByteStartWithTimeout - Starts multi-byte transmission from Master to Slave with timeout \return 0x01 or 0x00URE of the transmission process.
	   * I2C_masterSendMultiByteNextWithTimeout - Continues multi-byte transmission from Master to Slave with timeout \return 0x01 or 0x00URE of the transmission process.
	   * I2C_masterSendMultiByteFinishWithTimeout - Finishes multi-byte transmission from Master to Slave with timeout \return 0x01 or 0x00URE of the transmission process.
	   * I2C_masterSendMultiByteStopWithTimeout - Send STOP byte at the end of a multi-byte transmission from Master to Slave with timeout
	   * I2C_masterReceiveMultiByteFinishWithTimeout - Finishes multi-byte reception at the Master end with timeout \return 0x01 or 0x00URE of the transmission process.
	   */
	   /******************************************* interrupt.h *****************************************************************************************************
	    * Interrupt_enableMaster - Enables the processor interrupt. \return Returns \b true if interrupts were disabled when the function was called or \b false if they were initially enabled.
	    * Interrupt_disableMaster - Disables the processor interrupt. \return Returns \b true if interrupts were already disabled when the function was called or \b false if they were initially enabled.
	    * Interrupt_registerInterrupt - Registers a function to be called when an interrupt occurs. See \link Interrupt_enableInterrupt \endlink for details about the interrupt parameter
	    * Interrupt_unregisterInterrupt - Unregisters the function to be called when an interrupt occurs.
	    * Interrupt_setPriorityGrouping - Sets the priority grouping of the interrupt controller.
	    * Interrupt_getPriorityGrouping - Gets the priority grouping of the interrupt controller.
	    * Interrupt_setPriority - Sets the priority of an interrupt.
	    * Interrupt_getPriority - Gets the priority of an interrupt.
	    * Interrupt_enableInterrupt - Enables an interrupt. \param interruptNumber specifies the interrupt to be enabled.
	    * Interrupt_disableInterrupt - Disables an interrupt.
	    * Interrupt_isEnabled - Returns if a peripheral interrupt is enabled. \return A non-zero value if the interrupt is enabled.
	    * Interrupt_pendInterrupt - Pends an interrupt. Pending an interrupt causes the interrupt controller to execute the corresponding interrupt handler at the next available time, based on the current interrupt state priorities.
	    * Interrupt_unpendInterrupt - Un-pends an interrupt.
	    * Interrupt_setPriorityMask - Sets the priority masking level
	    * Interrupt_getPriorityMask - Gets the priority masking level
	    * Interrupt_setVectorTableAddress - Sets the address of the vector table. This function is for advanced users who might want to switch between multiple instances of vector tables (perhaps between flash/ram).
	    * Interrupt_getVectorTableAddress - Returns the address of the interrupt vector table.
	    * Interrupt_enableSleepOnIsrExit - Enables the processor to sleep when exiting an ISR. For low power operation,
	    * Interrupt_disableSleepOnIsrExit - Disables the processor to sleep when exiting an ISR.
	     **/

	     /****************************** "interrupt.h" ***********************************************************************
	      *
	      *
	      *
	      *
	      **/

	      // **************************************************************************************************************************************************************
	      // Algumas definições para uso dos LEDs
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
	      extern void dr_Leds_alterar(uint16_t contador);
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
	extern void dr_Delay_k(double maximo);
	//*****************************************************************************
	/*
	* \brief Essa função tem como prósito fornecer um atraso de n msegundos
	*
	* \param i define qual valor de delay em ms será usado
	*      Tipic Values are :
	*      1
	*      2
	*      ...
	* \return None
	*/
	extern void dr_Delay_ms(int n);
	//*****************************************************************************
	/*
	* \brief Essa função tem como prósito fornecer um atraso de n segundos
	*
	* \param i define qual valor de delay em seg será usado
	*      Tipic Values are :
	*      1
	*      2
	*      ...
	* \return None
	*/
	extern void dr_Delay_s(int n);
	//*****************************************************************************
	/*
	* \brief Essa função tem como prósito alterar os leds através das chaves
	* deve ser posterior a função PinConfig
	* \param None
	* \return None
	*/
	extern void dr_Leds_alterar_pela_sw();
	//*****************************************************************************
	/*
	* \brief Essa função tem como prósito configurar:
	*   LEDS como saída ("posteriormente inicializando")
	*   Chaves como entradas Pull Up
	* deve ser posterior a função PinConfig
	* \param None
	* \return None
	*/
	extern void dr_Leds_sw_init();
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
	*
	* UART Configuration Parameter. These are the configuration parameters to
	* make the eUSCI A UART module to operate with a 115200 baud rate. These
	* values were calculated using the online calculator that TI provides
	* at:
	* http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html

	*/
	extern void dr_Uart_init();
	//*****************************************************************************
	/*
	* \brief Habilita a Interrupção da UART0 (EUSCI_A0_BASE)
	* para monitorar a flag de "recebimento" (EUSCI_A_UART_RECEIVE_INTERRUPT)
	* de modo que quando recebe algo ela executa
	* a rotina de interrupção (default):void EUSCIA0_IRQHandler(void)
	*/

	extern void dr_Uart_interrupt_receive();
	//*****************************************************************************
	/*
	* \brief Habilita as Interrupções que foram programadas até então
	*/
	extern void dr_Interrupt_on();
	//*****************************************************************************
	/*
	 * \brief Habilita as Interrupções que foram programadas até então
	 * Faz a leitura da frequência dos osciladores naquele momento e exibe-os
	 * através de um "printf".
	 * MCLK_timer
	 * SMCLK_timer
	 * HSMCLK_timer
	 * DCO_timer
	 * ACLK_timer
	 * BCLK_timer
	 *
	 */
	 extern void dr_Clk_print();




	//*****************************************************************************
	/*
	* \brief Essa função tem o propósito Iniciar uma contagem a partir do clock principal
	* da CPU com objetivo de calcular o tempo de execução de instruções entre o
	* Start_tick e o Stop_tick
	* obs1: A contagem é parada quando é chamada a função Stop_tick();
	* obs2: O tempo máximo de entre as duas funções deve ser menor que 10 segundos
	* ou seja, essa função mede o tempo de processamento até 10 segundo, caso esse valor seja
	* ultrapassado (e a interrupt_on estiver habilitada) será mandado um printf indicando
	* o estouro do tempo medido
	*
	* \param None
	* \return Valor atual do Tick (Valor do início da contagem)
	* O valor atual do Tick é (valor máximo de tick programado - número de ticks até o get)
	*/
	extern uint32_t dr_Tick_start();
	//*****************************************************************************
	/*
	* \brief Essa função tem o propósito Parar a contagem e calcular o tempo entre o Start
	* e o Stop
	* Quando a função Stop_tick é chamada é calculado o tempo gasto entre o Start e Stop em
	* milisegundos e é "printado" na UART o valor correspondente
	* \param None
	* \return Valor de Ticks entre o Start e o Stop
	*/
	extern uint32_t dr_Tick_stop();
	//*****************************************************************************
	/*
	* \brief Essa é a rotina de interrupção pré configurada que é chamado caso haja um estouro
	* da contagem
	*/
	extern void SysTick_Handler(void);
	//*****************************************************************************
	/*
	* \brief Essa função tem o propósito incializar o T32 e configurar com o periodo entregue
	* por parametro "tempo_seg". Esse timer é aconselhado para itervalos grandes maior que 1 seg
	* o limite superior ainda não foi definido, e o arredondamento também não foi
	* mas o periodo da interrupção pode ser calculada através de outra função dr_T32_getPeriod_seg()
	* obs1: o T32 só começa acontar apartir da função T32_start()
	* obs2: deve ser inicializada a interrupção através da função dr_T32_interrupt_init()
	* \param tempo_seg
	*           define o tempo em segundos que a função será chamada.
	* \return noce
	*/
	extern void dr_T32_init_seg(uint32_t timer, float tempo_seg);
	//*****************************************************************************
	/*
	* \brief Essa função dá o start no T32 para começar a contagem
	* obs1: o T32 deve ser incializado apartir das funções dr_T32_init_x antes do Start
	* obs2: aconselha-se configurar a interrupção antes do Start
	* \param none
	* \return none
	*/
extern void dr_T32_init_Hz(uint32_t timer, float freq_Hz);
	extern void dr_T32_start(uint32_t timer);
	//*****************************************************************************
	/*
	* \brief Essa função registra e habilita a interrupção que será usada pelo T32
	* obs1: Não esquecer de habilitar a interrupção geral em dr_interrupt_on
	* \param void rotina(void)
	*       recebe a rotina de interrupção que será chamada quando houver o estouro do
	*       T32 pré configurado pelas funções dr_T32_init_x
	* \return none
	*/
	extern void dr_T32_interrupt_init(uint32_t timer, void rotina(void));
	//*****************************************************************************
	/*
	* \brief Essa função retorna o valor exato que está configurado a interrupção
	* obs1: O principal uso dessa função é para calcular o period configurado sem arredondamentos
	* e o periodo que foi realmente registrado com todos os truncamentos;
	* \param none
	* \return period
	*       retorna o valor do periodo registrado nos registrados internos do T32
	*/
	extern double dr_T32_getPeriod_seg(uint32_t timer);
	//*****************************************************************************
	/*
	* \brief Essa função retorna o valor exato que está configurado a interrupção
	* obs1: O principal uso dessa função é para calcular o period configurado sem arredondamentos
	* e o periodo que foi realmente registrado com todos os truncamentos;
	* \param none
	* \return period
	*       retorna o valor de Frequência registrado nos registrados internos do T32
	*/
	extern double dr_T32_get_freq(uint32_t timer);


	//*****************************************************************************
	//
	// Mark the end of the C bindings section for C++ compilers.
	//
	//*****************************************************************************
#ifdef __cplusplus
}
#endif
#endif /* DRIVERS_CONTROL_H_ */

