/**
  ******************************************************************************
  * @file    bsp_i2c_ee.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   i2c mpu6050Ӧ�ú���bsp
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 


#include "bsp_i2c.h"
#include "bsp_SysTick.h"



#define Delay mdelay

/* STM32 I2C ����ģʽ */
#define I2C_Speed              400000  //*

/* �����ַֻҪ��STM32��ҵ�I2C������ַ��һ������ */
#define I2Cx_OWN_ADDRESS7      0X0A   


static __IO uint32_t  I2CTimeout = I2CT_LONG_TIMEOUT;    



/********************************* Prototypes *********************************/
unsigned long ST_Hard_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);
unsigned long ST_Hard_Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
/*******************************  Function ************************************/

//
#define Soft_I2C_SDA_STATE   	GPIO_ReadInputDataBit(Soft_I2C_PORT, Soft_I2C_SDA)
#define Soft_I2C_DELAY 				Soft_I2C_Delay(100000)
#define Soft_I2C_NOP					Soft_I2C_Delay(10) 
//
#define Soft_I2C_READY		0x00
#define Soft_I2C_BUS_BUSY	0x01	
#define Soft_I2C_BUS_ERROR	0x02
//
#define Soft_I2C_NACK	  0x00 
#define Soft_I2C_ACK		0x01


//
static void Soft_I2C_Configuration(void);
static void Soft_I2C_Delay(uint32_t dly);
static uint8_t Soft_I2C_START(void);
static void Soft_I2C_STOP(void);
static void Soft_I2C_SendACK(void);
static void Soft_I2C_SendNACK(void);
static uint8_t Soft_I2C_SendByte(uint8_t anbt_i2c_data);
static uint8_t Soft_I2C_ReceiveByte_WithACK(void);
static uint8_t Soft_I2C_ReceiveByte(void);
static uint8_t Soft_DMP_I2C_Write(uint8_t soft_dev_addr, uint8_t soft_reg_addr, uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf);
static uint8_t Soft_DMP_I2C_Read(uint8_t soft_dev_addr, uint8_t soft_reg_addr, uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf);


#define ST_Sensors_I2C_WriteRegister  Soft_DMP_I2C_Write
#define ST_Sensors_I2C_ReadRegister Soft_DMP_I2C_Read


/**
  * @brief  I2C �����ʼ��
  * @param  ��
  * @retval ��
  */
void I2C_Bus_Init(void)
{	
	
	Set_I2C_Retry(5);
	
	#ifdef HARD_IIC
	MPU_DEBUG("hard iic");
	
  I2C_GPIO_Config(); 
 
  I2C_Mode_Configu();
	
	#else
	
  MPU_DEBUG("soft iic");
	Soft_I2C_Configuration();
	
	#endif

}






/**
  * @brief  ��IIC�豸�ļĴ�������д�����ݣ�����ʱ�������ã���mpu�ӿڵ���
  * @param  Address: IIC�豸��ַ
  * @param  RegisterAddr: �Ĵ�����ַ
  * @param  RegisterLen: Ҫд�����ݵĳ���
  * @param  RegisterValue: Ҫָ��д�����ݵ�ָ��
  * @retval 0��������0�쳣
  */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len,
                                        const unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryWriteAgain:
  ret = 0;
  ret = ST_Sensors_I2C_WriteRegister( slave_addr, reg_addr, len, ( unsigned char *)data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;

    mdelay(retry_in_mlsec);
    goto tryWriteAgain;
  }
  return ret;
}


/**
  * @brief  ��IIC�豸�ļĴ���������������,����ʱ�������ã���mpu�ӿڵ���
  * @param  Address: IIC�豸��ַ
  * @param  RegisterAddr: �Ĵ�����ַ
  * @param  RegisterLen: Ҫ��ȡ�����ݳ���
  * @param  RegisterValue: ָ��洢�������ݵ�ָ��
  * @retval 0��������0�쳣
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len,
                                       unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryReadAgain:
  ret = 0;
  ret = ST_Sensors_I2C_ReadRegister( slave_addr, reg_addr, len, ( unsigned char *)data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;

    mdelay(retry_in_mlsec);
    goto tryReadAgain;
  }
  return ret;
}


static unsigned short RETRY_IN_MLSEC  = 55;

/**
  * @brief  ����iic����ʱ��
  * @param  ml_sec�����Ե�ʱ�䣬��λ����
  * @retval ���Ե�ʱ�䣬��λ����
  */
void Set_I2C_Retry(unsigned short ml_sec)
{
  RETRY_IN_MLSEC = ml_sec;
}

/**
  * @brief  ��ȡ���õ�iic����ʱ��
  * @param  none
  * @retval none
  */
unsigned short Get_I2C_Retry(void)
{
  return RETRY_IN_MLSEC;
}


/************************���IIC��������****************************************/

#ifndef HARD_IIC

static void Soft_I2C_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); 
	//
	//
  GPIO_InitStructure.GPIO_Pin = Soft_I2C_SCL | Soft_I2C_SDA;					//����ʹ�õ�I2C��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   //����I2C�������������ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	  //����I2CΪ��©���
  GPIO_Init(Soft_I2C_PORT, &GPIO_InitStructure); 
	//
	//
	Soft_I2C_SCL_1; 
	Soft_I2C_SDA_1;
	Soft_I2C_DELAY; 
}

static void Soft_I2C_Delay(uint32_t dly) 
{
	while(--dly);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
}

static uint8_t Soft_I2C_START(void)
{ 
	Soft_I2C_SDA_1; 
 	Soft_I2C_NOP;
  // 
 	Soft_I2C_SCL_1; 
 	Soft_I2C_NOP;    
	//
 	if(!Soft_I2C_SDA_STATE) return Soft_I2C_BUS_BUSY;
	//
 	Soft_I2C_SDA_0;
 	Soft_I2C_NOP;
  //
 	Soft_I2C_SCL_0;  
 	Soft_I2C_NOP; 
	//
 	if(Soft_I2C_SDA_STATE) return Soft_I2C_BUS_ERROR;
	//
 	return Soft_I2C_READY;
}

static void Soft_I2C_STOP(void)
{
 	Soft_I2C_SDA_0; 
 	Soft_I2C_NOP;
  // 
 	Soft_I2C_SCL_1; 
 	Soft_I2C_NOP;    
	//
 	Soft_I2C_SDA_1;
 	Soft_I2C_NOP;
}

static void Soft_I2C_SendACK(void)
{
 	Soft_I2C_SDA_0;
 	Soft_I2C_NOP;
 	Soft_I2C_SCL_1;
 	Soft_I2C_NOP;
 	Soft_I2C_SCL_0; 
 	Soft_I2C_NOP;  
}

static void Soft_I2C_SendNACK(void)
{
	Soft_I2C_SDA_1;
	Soft_I2C_NOP;
	Soft_I2C_SCL_1;
	Soft_I2C_NOP;
	Soft_I2C_SCL_0; 
	Soft_I2C_NOP;  
}




/**
  * @brief  �ȴ�Ӧ���źŵ���
  * @retval ����ֵ��1������Ӧ��ʧ��
	*									0������Ӧ��ɹ�
  */
uint8_t Soft_I2C_Wait_Ack(void)
{
	uint8_t ucErrTime=0;

	Soft_I2C_SDA_1;
	Soft_I2C_NOP;	   
	Soft_I2C_SCL_1;
	Soft_I2C_NOP;	 
	
	while(Soft_I2C_SDA_STATE)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			Soft_I2C_STOP();
			return Soft_I2C_BUS_ERROR;
		}
	}
	Soft_I2C_SCL_0;//ʱ�����0 	   
	return 0;  
} 

static uint8_t Soft_I2C_SendByte(uint8_t soft_i2c_data)
{
 	uint8_t i;
 	
	Soft_I2C_SCL_0;
 	for(i=0;i<8;i++)
 	{  
  		if(soft_i2c_data&0x80) Soft_I2C_SDA_1;
   		else Soft_I2C_SDA_0;
			//
  		soft_i2c_data<<=1;
  		Soft_I2C_NOP;
			//
  		Soft_I2C_SCL_1;
  		Soft_I2C_NOP;
  		Soft_I2C_SCL_0;
  		Soft_I2C_NOP; 
 	}
	//
// 	Soft_I2C_SDA_1; 
// 	Soft_I2C_NOP;
// 	Soft_I2C_SCL_1;
// 	Soft_I2C_NOP;   
// 	if(Soft_I2C_SDA_STATE)
// 	{
//  		Soft_I2C_SCL_0;
//  		return Soft_I2C_NACK;
// 	}
// 	else
// 	{
//  		Soft_I2C_SCL_0;
//  		return Soft_I2C_ACK;  
// 	}  
	return Soft_I2C_Wait_Ack();  
}

static uint8_t Soft_I2C_ReceiveByte(void)
{
	uint8_t i,soft_i2c_data;
	//
 	Soft_I2C_SDA_1;
 	Soft_I2C_SCL_0; 
 	soft_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		Soft_I2C_SCL_1;
  		Soft_I2C_NOP; 
  		soft_i2c_data<<=1;
			//
  		if(Soft_I2C_SDA_STATE)	soft_i2c_data|=0x01; 
  
  		Soft_I2C_SCL_0;  
  		Soft_I2C_NOP;         
 	}
	Soft_I2C_SendNACK();
 	return soft_i2c_data;
}

static uint8_t Soft_I2C_ReceiveByte_WithACK(void)
{
	uint8_t i,soft_i2c_data;
	//
 	Soft_I2C_SDA_1;
 	Soft_I2C_SCL_0; 
 	soft_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		Soft_I2C_SCL_1;
  		Soft_I2C_NOP; 
  		soft_i2c_data<<=1;
			//
  		if(Soft_I2C_SDA_STATE)	soft_i2c_data|=0x01; 
  
  		Soft_I2C_SCL_0;  
  		Soft_I2C_NOP;         
 	}
	Soft_I2C_SendACK();
 	return soft_i2c_data;
}


//static void Soft_DMP_Delay_us(uint32_t dly)
//{
//	uint8_t i;
//	while(dly--) for(i=0;i<10;i++);
//}
////
//static void Soft_DMP_Delay_ms(uint32_t dly)
//{
//	while(dly--) Soft_DMP_Delay_us(1000);
//}
//

static uint8_t Soft_DMP_I2C_Write(uint8_t soft_dev_addr, uint8_t soft_reg_addr, uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf)
{		
		uint8_t i, result=0;
		Soft_I2C_START();
		result  = Soft_I2C_SendByte(soft_dev_addr << 1 | I2C_Direction_Transmitter);	
		if(result != 0) return result;
	
		result = Soft_I2C_SendByte(soft_reg_addr);  
		if(result != 0) return result;
	
		for (i=0;i<soft_i2c_len;i++) 
		{
			result = Soft_I2C_SendByte(soft_i2c_data_buf[i]); 
			if (result != 0) return result;
		}
		Soft_I2C_STOP();
		return 0x00;
}

static uint8_t Soft_DMP_I2C_Read(uint8_t soft_dev_addr, uint8_t soft_reg_addr, uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf)
{
		uint8_t result;
	
		Soft_I2C_START();
		result  = Soft_I2C_SendByte(soft_dev_addr << 1 | I2C_Direction_Transmitter);			
		if(result != 0) return result;

		result = Soft_I2C_SendByte(soft_reg_addr); 
		if(result != 0) return result;

		Soft_I2C_START();
		result = Soft_I2C_SendByte(soft_dev_addr << 1 | I2C_Direction_Receiver);
		if(result != 0) return result;
	
		//
    while (soft_i2c_len)
		{
			if (soft_i2c_len==1) *soft_i2c_data_buf =Soft_I2C_ReceiveByte();  
      else *soft_i2c_data_buf =Soft_I2C_ReceiveByte_WithACK();
      soft_i2c_data_buf++;
      soft_i2c_len--;
    }
		Soft_I2C_STOP();
    return 0x00;
}

#endif //endof #ifndef HARD_IIC
/*********************************************END OF FILE**********************/

