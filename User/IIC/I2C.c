#include "i2c.h"

#define SCL_H()         GPIOC->BSRR = GPIO_Pin_12
#define SCL_L()         GPIOC->BRR  = GPIO_Pin_12 

#define SDA_H()         GPIOC->BSRR = GPIO_Pin_11 
#define SDA_L()         GPIOC->BRR  = GPIO_Pin_11 

#define SDA_read()      GPIOC->IDR  & GPIO_Pin_11 

void I2C_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static void I2C_delay(void)
{
	volatile int i = 7;
	while (i)
	i--;
}

static bool I2C_Start(void)
{
	SDA_H();
	SCL_H();
	I2C_delay();
	if (!SDA_read())
			return false;
	SDA_L();
	I2C_delay();
	if (SDA_read())
			return false;
	SDA_L();
	I2C_delay();
	return true;
}

static void I2C_Stop(void)
{
	SCL_L();
	I2C_delay();
	SDA_L();
	I2C_delay();
	SCL_H();
	I2C_delay();
	SDA_H();
	I2C_delay();
}

static void I2C_Ack(void)
{
	SCL_L();
	I2C_delay();
	SDA_L();
	I2C_delay();
	SCL_H();
	I2C_delay();
	SCL_L();
	I2C_delay();
}

static void I2C_NoAck(void)
{
	SCL_L();
	I2C_delay();
	SDA_H();
	I2C_delay();
	SCL_H();
	I2C_delay();
	SCL_L();
	I2C_delay();
}

static bool I2C_WaitAck(void)
{
	SCL_L();
	I2C_delay();
	SDA_H();
	I2C_delay();
	SCL_H();
	I2C_delay();
	if (SDA_read()) 
	{
		SCL_L();
		return false;
	}
	SCL_L();
	return true;
}

static void I2C_SendByte(uint8_t byte)
{
	uint8_t i = 8;
	while (i--) 
	{
		SCL_L();
		I2C_delay();
		if (byte & 0x80)
				SDA_H();
		else
				SDA_L();
		byte <<= 1;
		I2C_delay();
		SCL_H();
		I2C_delay();
	}
	SCL_L();
}

static uint8_t I2C_ReceiveByte(void)
{
	uint8_t i = 8;
	uint8_t byte = 0;

	SDA_H();
	while (i--) 
	{
		byte <<= 1;
		SCL_L();
		I2C_delay();
		SCL_H();
		I2C_delay();
		if (SDA_read())
		{
			byte |= 0x01;
		}
	}
	SCL_L();
	return byte;
}

bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	int i;
	if (!I2C_Start())
		return false;
	I2C_SendByte(addr << 1 | I2C_DirectionTransmitter);
	if (!I2C_WaitAck()) 
	{
		I2C_Stop();
		return false;
	}
	I2C_SendByte(reg);
	I2C_WaitAck();
	for (i = 0; i < len; i++) 
	{
		I2C_SendByte(data[i]);
		if (!I2C_WaitAck()) 
		{
			I2C_Stop();
			return false;
		}
	}
	I2C_Stop();
	return true;
}

int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	if(i2cWriteBuffer(addr,reg,len,data))
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if(i2cRead(addr,reg,len,buf))
	{
		return 0;
	}
	else
	{
		return -1;
	}
}
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
	if (!I2C_Start())
		return false;
	I2C_SendByte(addr << 1 | I2C_DirectionTransmitter);
	if (!I2C_WaitAck()) {
		I2C_Stop();
		return false;
	}
	I2C_SendByte(reg);
	I2C_WaitAck();
	I2C_SendByte(data);
	I2C_WaitAck();
	I2C_Stop();
	return true;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	if (!I2C_Start())
		return false;
	I2C_SendByte(addr << 1 | I2C_DirectionTransmitter);
	if (!I2C_WaitAck()) 
	{
		I2C_Stop();
		return false;
	}
	I2C_SendByte(reg);
	I2C_WaitAck();
	I2C_Start();
	I2C_SendByte(addr << 1 | I2C_DirectionReceiver);
	I2C_WaitAck();
	while (len) 
	{
		*buf = I2C_ReceiveByte();
		if (len == 1)
			I2C_NoAck();
		else
			I2C_Ack();
		buf++;
		len--;
	}
	I2C_Stop();
	return true;
}

uint16_t i2cGetErrorCounter(void)
{
	return 0;
}



