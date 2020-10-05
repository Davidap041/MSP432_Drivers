#include "mpu6050.h"
void Update_Servo_Motor(uint16_t position_table, bool prbs_on);

int DR_mpu6050_atualizar(dr_mpu_data_t *sensor)
{
    uint8_t leitura[14] = { 0 };
    uint8_t ready = 0;
    int8_t status_erro;
    erro_watch[1] = 100;
    /* interrup��o dado Pronto */
    while (((ready && 1) != 1) && erro_watch[1] > 0)
    {
        status_erro = DR_mpu6050_read(sensor->I2C, sensor->address, 0x3A,
                                      &ready);
        if (status_erro != 1)
        {
            DR_mpu6050_ligar(100);
            DR_mpu6050_init(sensor);
            diagnostic_erro[0]++;	// Comunication Error
            return status_erro; // erro nos cabos de comunica��o

        }
        erro_watch[1]--;
    }
    if (erro_watch[1] == 0)
    {
        DR_mpu6050_ligar(100);
        DR_mpu6050_init(sensor);
        diagnostic_erro[1]++; // Supply Error
        return -1; // erro nos cabos de alimenta��o
    }

    DR_mpu6050_readraw(sensor->I2C, sensor->address, 0x3B, 14, leitura); // Ta dando erro aqui dentro
    sensor->ax = (leitura[0] << 8) | (leitura[1]);
    sensor->ay = (leitura[2] << 8) | (leitura[3]);
    sensor->az = (leitura[4] << 8) | (leitura[5]);
    sensor->temp = (leitura[6] << 8) | (leitura[7]);
    sensor->gx = (leitura[8] << 8) | (leitura[9]);
    sensor->gy = (leitura[10] << 8) | (leitura[11]);
    sensor->gz = (leitura[12] << 8) | (leitura[13]);
    return 1;
}

int DR_mpu6050_init(dr_mpu_data_t *sensor)
{
    uint8_t data = 0;
    uint16_t status_erro;
    status_erro = DR_mpu6050_read(sensor->I2C, sensor->address, 0x75, &data)
            - 10;
    if (data != 0x68)
    {
        return status_erro;
    }
    //Acelerometro mede a resolu��o +-2g
    DR_i2c_write(sensor->I2C, sensor->address, 0x1C, 0b00000000);
    //pequeno filtro digital
    DR_i2c_write(sensor->I2C, sensor->address, 0x1A, 0b00000001);
    //liga interrup��o dado pronto
    DR_i2c_write(sensor->I2C, sensor->address, 0x38, 0b00000001);
    //DESLIGA GYRO	//data = 0b00000111;
    DR_i2c_write(sensor->I2C, sensor->address, 0x6C, 0b00000000);
    //configura Gyro em  � 250 �/s
    DR_i2c_write(sensor->I2C, sensor->address, 0x1B, 0b00000000);
    //Divisor de clock - amostragem //OR = 1Khz / (1+data)
    DR_i2c_write(sensor->I2C, sensor->address, 0x19, 24);
    //ACORDA
    DR_i2c_write(sensor->I2C, sensor->address, 0x6B, 0b00000000);
    return 1;
}

void DR_mpu6050_ligar(uint16_t tempo_ms)
{
    /* Rotina para ligar o sensor, ou sensores */
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN1);
    DR_delay_ms(tempo_ms);  // m�nmo 50
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN1);
    DR_delay_ms(tempo_ms);
}

int DR_mpu6050_read(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
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
    erro_watch[4] = 500;

    moduleI2C->I2CSA = slaveAddr; /* setup slave address */
    moduleI2C->CTLW0 |= 0x0010; /* enable transmitter */
    moduleI2C->CTLW0 |= 0x0002; /* generate START and send slave address */
    /* wait until slave address is sent *//*Essa flag n�o estar esperando*/
    while ((moduleI2C->CTLW0 & 2) && erro_watch[2] > 0)
    {
        erro_watch[2]--;
        diagnostic_erro[2]++; /*[2]Erro de ??*/
    }
    if (erro_watch[2] == 0)
        return -2;

    moduleI2C->TXBUF = memAddr; /* send memory address to slave */
    /* wait till it's ready to transmit */
    while (!(moduleI2C->IFG & 2) && erro_watch[3] > 0)
    {
        erro_watch[3]--;
        diagnostic_erro[3]++; /*[3] Erro de ?*/
    }

    if (erro_watch[3] == 0)
        return -3;

    moduleI2C->CTLW0 &= ~0x0010; /* enable receiver */
    moduleI2C->CTLW0 |= 0x0002; /* generate RESTART and send slave address */
    while (moduleI2C->CTLW0 & 2)
        ; /* wait till restart is finished */
    moduleI2C->CTLW0 |= 0x0004; /* setup to send STOP after the byte is received */

    /* wait till data is received */ // flag de erro aqui simm kk
    while (!(moduleI2C->IFG & 1) && erro_watch[4] > 0)
    {
        erro_watch[4]--;
        diagnostic_erro[4]++;
    }
    if (erro_watch[4] == 0)
        return -4;

    *data = moduleI2C->RXBUF; /* read the received data */
    while (moduleI2C->CTLW0 & 4)
        ; /* wait until STOP is sent */
    // dr_I2C_read() end
    return 1;
}
int DR_mpu6050_readraw(uint8_t n_I2C, uint8_t slaveAddr, uint8_t memAddr,
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
            // Error Aqui !!!
            ; /* wait till data is received */
        *data++ = moduleI2C->RXBUF; /* read the received data */
        byteCount--;
    }
    while (byteCount);

    while (moduleI2C->CTLW0 & 4)
        ; /* wait until STOP is sent */

    return 0; /* no error */
}

void DR_Gyroscope_calibrate(dr_mpu_data_t *sensor)
{
    int i = 100;
    int Max_Value[3];
    int Min_Value[3];

    DR_mpu9250_atualizar(sensor);

    Max_Value[0] = sensor->gx;
    Min_Value[0] = sensor->gx;
    Max_Value[1] = sensor->gy;
    Min_Value[1] = sensor->gy;
    Max_Value[2] = sensor->gz;
    Min_Value[2] = sensor->gz;

    while (i > 0)
    {
        DR_mpu9250_atualizar(sensor);
        // Higher and lower X-axis Value
        if (sensor->gx > Max_Value[0])
        {
            Max_Value[0] = sensor->gx;
        }
        if (sensor->gx < Min_Value[0])
        {
            Min_Value[0] = sensor->gx;
        }
        // Higher and lower Y-axis Value
        if (sensor->gy > Max_Value[1])
        {
            Max_Value[1] = sensor->gy;
        }
        if (sensor->gy < Min_Value[1])
        {
            Min_Value[1] = sensor->gy;
        }
        // Higher and lower Z-axis Value
        if (sensor->gz > Max_Value[2])
        {
            Max_Value[2] = sensor->gz;
        }
        if (sensor->gz < Min_Value[2])
        {
            Min_Value[2] = sensor->gz;
        }
        DR_delay_k(1);
        i--;
    }
    sensor->gyro_offset_x = (Max_Value[0] + Min_Value[0]) / 2; // Offset  x-axis
    sensor->gyro_offset_y = (Max_Value[1] + Min_Value[1]) / 2; // Offset  y-axis
    sensor->gyro_offset_z = (Max_Value[2] + Min_Value[2]) / 2; // Offset  z-axis
}

void DR_magnetometer_calibrate(dr_mpu_data_t *sensor)
{
    int i = 1000;
    int Max_Value[3];
    int Min_Value[3];
    uint8_t leitura[14];
    uint16_t div_motor = 62;    // tamanho dos pontos de calibacao dos motores
    float int_motor;
    float cont_motor = 0;

    int_motor = i / div_motor;

    DR_i2c_readraw(0, 0x0C, 0x03, 7, leitura);
    sensor->ax = (leitura[1] << 8) | (leitura[0]); // HXL HXH
    sensor->ay = (leitura[3] << 8) | (leitura[2]);
    sensor->az = (leitura[4] << 8) | (leitura[4]);

    Max_Value[0] = sensor->ax;
    Min_Value[0] = sensor->ax;
    Max_Value[1] = sensor->ay;
    Min_Value[1] = sensor->ay;
    Max_Value[2] = sensor->az;
    Min_Value[2] = sensor->az;

    while (i > 0)
    {
        DR_i2c_readraw(0, 0x0C, 0x03, 7, leitura);
        sensor->ax = (leitura[1] << 8) | (leitura[0]); // HXL HXH
        sensor->ay = (leitura[3] << 8) | (leitura[2]);
        sensor->az = (leitura[4] << 8) | (leitura[4]);
        // Higher and lower X-axis Value
        if (sensor->ax > Max_Value[0])
        {
            Max_Value[0] = sensor->ax;
        }
        if (sensor->ax < Min_Value[0])
        {
            Min_Value[0] = sensor->ax;
        }
        // Higher and lower Y-axis Value
        if (sensor->ay > Max_Value[1])
        {
            Max_Value[1] = sensor->ay;
        }
        if (sensor->ay < Min_Value[1])
        {
            Min_Value[1] = sensor->ay;
        }
        // Higher and lower Z-axis Value
        if (sensor->az > Max_Value[2])
        {
            Max_Value[2] = sensor->az;
        }
        if (sensor->az < Min_Value[2])
        {
            Min_Value[2] = sensor->az;
        }
        DR_delay_k(1);
        float param_a, param_b;
        param_a = (div_motor - cont_motor) * int_motor;
        param_b = (div_motor - cont_motor - 1) * int_motor;
        if ((i > param_b) && (i < param_a))
        {
            Update_Servo_Motor(cont_motor, 0);
            cont_motor++;
        }

        i--;
    }
    sensor->mag_offset_x = (Max_Value[0] + Min_Value[0]) / 2; // Offset  x-axis
    sensor->mag_offset_y = (Max_Value[1] + Min_Value[1]) / 2; // Offset  y-axis
    sensor->mag_offset_z = (Max_Value[2] + Min_Value[2]) / 2; // Offset  z-axis
}

void initAK8963(float *destination)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    DR_i2c_write(0, 0x0C, 0x0A, 0x00); // Power down magnetometer
    DR_delay_k(1);
    DR_i2c_write(0, 0x0C, 0x0A, 0x0F); // Enter Fuse ROM access mode
    DR_delay_k(1);
    DR_i2c_readraw(0, 0x0C, 0X10, 3, &rawData[0]); // Read the x-, y-, and z-axis calibration values
    destination[0] = (float) (rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
    destination[1] = (float) (rawData[1] - 128) / 256. + 1.;
    destination[2] = (float) (rawData[2] - 128) / 256. + 1.;
    DR_i2c_write(0, 0x0C, 0x0A, 0x00); // Power down magnetometer
    DR_delay_k(1);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    DR_i2c_write(0, 0x0C, 0x0A, 1 << 4 | 0x02); // Set magnetometer data resolution and sample ODR
    DR_delay_k(1);
}

int DR_mpu9250_init(dr_mpu_data_t *sensor)
{
    // Set accelerometers low pass filter at 5Hz (ACCEL_CONFIG 2)
    DR_i2c_write(sensor->I2C, sensor->address, 29, 0x06);
    // Set gyroscope low pass filter at 5Hz (CONFIG)
    DR_i2c_write(sensor->I2C, sensor->address, 26, 0x06);

    // Configure gyroscope range(GYRO_FS_SEL)
    DR_i2c_write(sensor->I2C, sensor->address, 27, 0x10);
    // Configure accelerometers range (ACCEL_FS_SEL)
    DR_i2c_write(sensor->I2C, sensor->address, 28, 0x08);
    // Set by pass mode for the magnetometers
    DR_i2c_write(sensor->I2C, sensor->address, 0x37, 0x02);
    // Request continuous magnetometer measurements in 16 bits !!!
    DR_i2c_write(sensor->I2C, 0x0C, 0x0A, 0x16);
    return 1;
}
int DR_mpu9250_atualizar(dr_mpu_data_t *sensor)
{
    // Leitura do Acelerêometro e Giroscopio
    uint8_t leitura[14];
    DR_i2c_readraw(0, 0x68, 0x3B, 14, leitura);
    sensor->ax = (leitura[0] << 8) | (leitura[1]); // AccXH XL
    sensor->ay = (leitura[2] << 8) | (leitura[3]);
    sensor->az = (leitura[4] << 8) | (leitura[5]);
    sensor->temp = (leitura[6] << 8) | (leitura[7]);
    sensor->gx = (leitura[8] << 8) | (leitura[9]);
    sensor->gy = (leitura[10] << 8) | (leitura[11]);
    sensor->gz = (leitura[12] << 8) | (leitura[13]);
    // Leitura do Magnetometro
    DR_i2c_readraw(0, 0x0C, 0x03, 7, leitura);
    sensor->mx = (leitura[1] << 8) | (leitura[0]); // HXL HXH
    sensor->my = (leitura[3] << 8) | (leitura[2]);
    sensor->mz = (leitura[4] << 8) | (leitura[4]);
    //leitura[7];	// Dado pronto
    return 1;
}

