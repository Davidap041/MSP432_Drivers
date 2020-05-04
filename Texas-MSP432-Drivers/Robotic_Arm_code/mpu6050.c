#include "mpu6050.h"

int dr_MPU6050_atualizar(mpu_data_t *sensor)
{
	uint8_t leitura[14] = { 0 };
	uint8_t ready = 0;
	int8_t status_erro;
	erro_watch[1] = 100;
	/* interrupção dado Pronto */
	while (((ready && 1) != 1) && erro_watch[1] > 0)
	{
		status_erro = dr_MPU6050_Read(sensor->I2C, sensor->address, 0x3A,
										&ready);
		if (status_erro != 1)
		{
			dr_MPU6050_ligar(100);
			dr_MPU6050_init(sensor);
			return status_erro; // erro nos cabos de comunicação

		}
		erro_watch[1]--;
	}
	if (erro_watch[1] == 0)
	{
		dr_MPU6050_ligar(100);
		dr_MPU6050_init(sensor);
		return -1; // erro nos cabos de alimentação
	}

	dr_I2C_ReadRaw(sensor->I2C, sensor->address, 0x3B, 14, leitura);
	sensor->ax = (leitura[0] << 8) | (leitura[1]);
	sensor->ay = (leitura[2] << 8) | (leitura[3]);
	sensor->az = (leitura[4] << 8) | (leitura[5]);
	sensor->temp = (leitura[6] << 8) | (leitura[7]);
	sensor->gx = (leitura[8] << 8) | (leitura[9]);
	sensor->gy = (leitura[10] << 8) | (leitura[11]);
	sensor->gz = (leitura[12] << 8) | (leitura[13]);
	return 1;
}

int dr_MPU6050_init(mpu_data_t *sensor)
{
	uint8_t data = 0;
	uint16_t status_erro;
	status_erro = dr_MPU6050_Read(sensor->I2C, sensor->address, 0x75, &data)
			- 10;
	if (data != 0x68)
	{
		return status_erro;
	}
	//Acelerometro mede a resolução +-2g
	dr_I2C_Write(sensor->I2C, sensor->address, 0x1C, 0b00000000);
	//pequeno filtro digital
	dr_I2C_Write(sensor->I2C, sensor->address, 0x1A, 0b00000001);
	//liga interrupção dado pronto
	dr_I2C_Write(sensor->I2C, sensor->address, 0x38, 0b00000001);
	//DESLIGA GYRO	//data = 0b00000111;
	dr_I2C_Write(sensor->I2C, sensor->address, 0x6C, 0b00000000);
	//configura Gyro em  ± 250 °/s
	dr_I2C_Write(sensor->I2C, sensor->address, 0x1B, 0b00000000);
	//Divisor de clock - amostragem //OR = 1Khz / (1+data)
	dr_I2C_Write(sensor->I2C, sensor->address, 0x19, 24);
	//ACORDA
	dr_I2C_Write(sensor->I2C, sensor->address, 0x6B, 0b00000000);
	return 1;
}

void dr_MPU6050_ligar(uint16_t tempo_ms)
{
	/* Rotina para ligar o sensor, ou sensores */
	GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN1);
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN1);
	dr_Delay_ms(tempo_ms);  // mínmo 50
	GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN1);
	dr_Delay_ms(tempo_ms);
}

int dr_MPU6050_Read(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
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
	erro_watch[2] = 500;
	erro_watch[3] = 500;

	moduleI2C->I2CSA = slaveAddr; /* setup slave address */
	moduleI2C->CTLW0 |= 0x0010; /* enable transmitter */
	moduleI2C->CTLW0 |= 0x0002; /* generate START and send slave address */
	/* wait until slave address is sent *//*Essa flag não estar esperando*/
	while ((moduleI2C->CTLW0 & 2) && erro_watch[2] > 0)
	{
		erro_watch[2]--;
	}
	if (erro_watch[2] == 0)
		return -2;

	moduleI2C->TXBUF = memAddr; /* send memory address to slave */
	/* wait till it's ready to transmit */
	while (!(moduleI2C->IFG & 2) && erro_watch[3] > 0)
	{
		erro_watch[3]--;
	}

	if (erro_watch[3] == 0)
		return -3;

	moduleI2C->CTLW0 &= ~0x0010; /* enable receiver */
	moduleI2C->CTLW0 |= 0x0002; /* generate RESTART and send slave address */
	while (moduleI2C->CTLW0 & 2)
		; /* wait till restart is finished */
	moduleI2C->CTLW0 |= 0x0004; /* setup to send STOP after the byte is received */
	while (!(moduleI2C->IFG & 1))
		; /* wait till data is received */ // flag de erro aqui ?
	*data = moduleI2C->RXBUF; /* read the received data */
	while (moduleI2C->CTLW0 & 4)
		; /* wait until STOP is sent */
	// dr_I2C_read() end
	return 1;
}
int dr_MPU6050_ReadRaw(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
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

