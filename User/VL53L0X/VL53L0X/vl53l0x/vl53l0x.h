#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "myiic.h" 

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define address 0x29

void vl530l0_iic_init(void);
u8 VL53L0X_Write_Byte(u8 reg,u8 data) ;
u8 VL53L0X_Read_Byte(u8 reg);
u8 VL53L0X_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
uint16_t makeuint16(int lsb, int msb);
//uint16_t vl530l0_get_distance(void);
float vl530l0_get_distance(void);
#endif
















