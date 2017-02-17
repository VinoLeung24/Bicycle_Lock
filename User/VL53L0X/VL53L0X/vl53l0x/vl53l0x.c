#include "vl53l0x.h"


u8 VL53L0X_Write_Byte(u8 reg,u8 data) 				 
{ 
  IIC_Start(); 
	IIC_Send_Byte(0X52);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}

u8 VL53L0X_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte(0x52);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte(0x53 );//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
    IIC_Stop();			//产生一个停止条件 
	return res;		
}

u8 VL53L0X_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte(0x52);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte(0x53);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//产生一个停止条件 
	return 0;	
}

uint16_t makeuint16(int lsb, int msb)
{
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void vl530l0_iic_init(void)
{
   IIC_Init();
}

//uint16_t vl530l0_get_distance(void)
//{
//    u16 i=0;
//	uint16_t dis;
//	u8 datatemp[16];
//	uint16_t temp_val = 0;
//	
//	  //写入刷新数据的指令  
//	VL53L0X_Write_Byte(VL53L0X_REG_SYSRANGE_START,0x01);
//		//滤波
//	for(i=0;i<10;i++)
//	{
//		VL53L0X_Read_Len(address,VL53L0X_REG_RESULT_RANGE_STATUS,12,datatemp);
//		
//		dis = makeuint16(datatemp[11], datatemp[10]);
//		
//		temp_val+=dis;
//		delay_ms(5);
//	}				
//	dis = temp_val/10;
//	temp_val = 0;
//	
//	return dis;
//}

float vl530l0_get_distance(void)
{
    u16 i=0;
	float dis;
	u8 datatemp[16];
	uint16_t temp_val = 0;
	
	  //写入刷新数据的指令  
	VL53L0X_Write_Byte(VL53L0X_REG_SYSRANGE_START,0x01);
		//滤波

		VL53L0X_Read_Len(address,VL53L0X_REG_RESULT_RANGE_STATUS,12,datatemp);
		
		dis = (float)makeuint16(datatemp[11], datatemp[10]) / 10;

	return dis;
}

























