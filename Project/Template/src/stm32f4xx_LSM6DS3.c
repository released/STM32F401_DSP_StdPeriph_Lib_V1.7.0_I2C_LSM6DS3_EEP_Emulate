/**
 ******************************************************************************
 * @file    LSM6DS3.c
 * @author  MEMS Application Team
 * @version V1.0.0
 * @date    30-July-2014
 * @brief   This file provides a set of functions needed to manage the LSM6DS3.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_LSM6DS3.h"

#include <stdio.h>
#include <math.h>

__IO uint32_t  LSM6DS3_Timeout = LSM6DS3_LONG_TIMEOUT; 

static int16_t LSM6DS3_ACCx,LSM6DS3_ACCy,LSM6DS3_ACCz;
static int16_t LSM6DS3_GYROx,LSM6DS3_GYROy,LSM6DS3_GYROz;

static int16_t cLSM6DS3_ACCx =0;
static int16_t cLSM6DS3_ACCy =0;
static int16_t cLSM6DS3_ACCz =0;

static int16_t cLSM6DS3_GYROx =0;
static int16_t cLSM6DS3_GYROy =0;
static int16_t cLSM6DS3_GYROz =0;

void drvLSM6DS3_Delay(uint16_t nCount)
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}

int16_t drvLSM6DS3_I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint8_t ack) {
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	LSM6DS3_Timeout = LSM6DS3_LONG_TIMEOUT;
	while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) && LSM6DS3_Timeout) 
	{
		if (--LSM6DS3_Timeout == 0x00) 
		{
			return 1;
		}
	}

	if (ack) 
	{
		I2C_AcknowledgeConfig(I2Cx, ENABLE);
	}
	
	I2C_Send7bitAddress(I2Cx, address, direction);

	if (direction == I2C_Direction_Transmitter) 
	{
		LSM6DS3_Timeout = LSM6DS3_LONG_TIMEOUT;
		while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR) && LSM6DS3_Timeout) 
		{
			if (--LSM6DS3_Timeout == 0x00) 
			{
				return 1;
			}
		}
	} else if (direction == I2C_Direction_Receiver) 
	{
		LSM6DS3_Timeout = LSM6DS3_LONG_TIMEOUT;
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && LSM6DS3_Timeout) 
		{
			if (--LSM6DS3_Timeout == 0x00) 
			{
				return 1;
			}
		}
	}
	I2Cx->SR2;
	
	return 0;
}

void drvLSM6DS3_I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data) {
	LSM6DS3_Timeout = LSM6DS3_LONG_TIMEOUT;
	while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) && LSM6DS3_Timeout) 
	{
		LSM6DS3_Timeout--;
	}
	I2C_SendData(I2Cx, data);
}

uint8_t drvLSM6DS3_I2C_ReadAck(I2C_TypeDef* I2Cx) {
	uint8_t data;
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	
	LSM6DS3_Timeout = LSM6DS3_LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) && LSM6DS3_Timeout) 
	{
		LSM6DS3_Timeout--;
	}

	data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t drvLSM6DS3_I2C_ReadNack(I2C_TypeDef* I2Cx) {
	uint8_t data;
	
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	LSM6DS3_Timeout = LSM6DS3_LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) && LSM6DS3_Timeout) 
	{
		LSM6DS3_Timeout--;
	}

	data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t drvLSM6DS3_I2C_Stop(I2C_TypeDef* I2Cx) {	
	LSM6DS3_Timeout = LSM6DS3_LONG_TIMEOUT;
	while (((!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE)) || (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF))) && LSM6DS3_Timeout) 
	{
		if (--LSM6DS3_Timeout == 0x00) 
		{
			return 1;
		}
	}
	I2C_GenerateSTOP(I2Cx, ENABLE);
	
	return 0;
}

uint8_t drvLSM6DS3_I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg) {
	uint8_t received_data;
	drvLSM6DS3_I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 0);
	drvLSM6DS3_I2C_WriteData(I2Cx, reg);
	drvLSM6DS3_I2C_Stop(I2Cx);
	drvLSM6DS3_I2C_Start(I2Cx, address, I2C_Direction_Receiver, 0);
	received_data = drvLSM6DS3_I2C_ReadNack(I2Cx);
	return received_data;
	
}

void drvLSM6DS3_I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data) 
{
	drvLSM6DS3_I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 0);
	drvLSM6DS3_I2C_WriteData(I2Cx, reg);
	drvLSM6DS3_I2C_WriteData(I2Cx, data);
	drvLSM6DS3_I2C_Stop(I2Cx);
	
}

void drvLSM6DS3_I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) 
{
	uint16_t i;
	drvLSM6DS3_I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 1);
	drvLSM6DS3_I2C_WriteData(I2Cx, reg);
	drvLSM6DS3_I2C_Stop(I2Cx);
	drvLSM6DS3_I2C_Start(I2Cx, address, I2C_Direction_Receiver, 1);
	for (i = 0; i < count; i++) 
	{
		if (i == (count - 1)) 
		{
			//Last byte
			data[i] = drvLSM6DS3_I2C_ReadNack(I2Cx);
		} else 
		{
			data[i] = drvLSM6DS3_I2C_ReadAck(I2Cx);
		}
	}
}

void drvLSM6DS3_I2C_ReadMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint16_t count) 
{
	uint16_t i;
	drvLSM6DS3_I2C_Start(I2Cx, address, I2C_Direction_Receiver, 1);
	for (i = 0; i < count; i++) 
	{
		if (i == (count - 1)) 
		{
			//Last byte
			data[i] = drvLSM6DS3_I2C_ReadNack(I2Cx);
		} else 
		{
			data[i] = drvLSM6DS3_I2C_ReadAck(I2Cx);
		}
	}
}

void drvLSM6DS3_I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) 
{
	uint16_t i;
	drvLSM6DS3_I2C_Start(I2Cx, address, I2C_Direction_Transmitter, 0);
	drvLSM6DS3_I2C_WriteData(I2Cx, reg);
	for (i = 0; i < count; i++) 
	{
		drvLSM6DS3_I2C_WriteData(I2Cx, data[i]);
	}
	drvLSM6DS3_I2C_Stop(I2Cx);
	
}

void appLSM6DS3_Read(uint8_t DeviceAddr, uint8_t RegisterAddr,
                              uint16_t NumByteToRead,
                              uint8_t* pBuffer)
{
	drvLSM6DS3_I2C_ReadMulti(LSM6DS3_I2C,DeviceAddr,RegisterAddr,pBuffer,NumByteToRead);
	
}
void appLSM6DS3_Write(uint8_t DeviceAddr, uint8_t RegisterAddr,
                               uint16_t NumByteToWrite,
                               uint8_t* pBuffer)
{
	drvLSM6DS3_I2C_WriteMulti(LSM6DS3_I2C,DeviceAddr,RegisterAddr,pBuffer,NumByteToWrite);
	
}

void appLSM6DS3_SetAccCalData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			cLSM6DS3_ACCx = data; 
			break;

		case AXIS_Y:
			cLSM6DS3_ACCy = data; 
			break;

		case AXIS_Z:
			cLSM6DS3_ACCz = data; 
			break;				
	}
}

void appLSM6DS3_SetGyroCalData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			cLSM6DS3_GYROx = data; 
			break;

		case AXIS_Y:
			cLSM6DS3_GYROy = data; 
			break;

		case AXIS_Z:
			cLSM6DS3_GYROz = data; 
			break;				
	}
}

void appLSM6DS3_SetAccData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			LSM6DS3_ACCx = data; 
			break;

		case AXIS_Y:
			LSM6DS3_ACCy = data; 
			break;

		case AXIS_Z:
			LSM6DS3_ACCz = data; 
			break;				
	}
}

void appLSM6DS3_SetGyroData(AXIS_TypeDef axis,int16_t data)
{
	switch(axis)
	{
		case AXIS_X:
			LSM6DS3_GYROx = data; 
			break;

		case AXIS_Y:
			LSM6DS3_GYROy = data; 
			break;

		case AXIS_Z:
			LSM6DS3_GYROz = data; 
			break;				
	}
}


int16_t appLSM6DS3_GetAccData(AXIS_TypeDef axis)
{
	int16_t data = 0;
	
	switch(axis)
	{
		case AXIS_X:
			data = LSM6DS3_ACCx + cLSM6DS3_ACCx; 
			break;

		case AXIS_Y:
			data = LSM6DS3_ACCy + cLSM6DS3_ACCy; 
			break;

		case AXIS_Z:
			data = LSM6DS3_ACCz + cLSM6DS3_ACCz; 
			break;				
	}
	
	return data ;
}

int16_t appLSM6DS3_GetGyroData(AXIS_TypeDef axis)
{
	int16_t data = 0;
	
	switch(axis)
	{
		case AXIS_X:
			data = LSM6DS3_GYROx + cLSM6DS3_GYROx; 
			break;

		case AXIS_Y:
			data = LSM6DS3_GYROy + cLSM6DS3_GYROy; 
			break;

		case AXIS_Z:
			data = LSM6DS3_GYROz + cLSM6DS3_GYROz; 
			break;				
	}
	
	return data ;
}


void appLSM6DS3_GetAcc(void)
{
	uint8_t tmpxl, tmpxh, tmpyl, tmpyh, tmpzl, tmpzh, tmp;
	int16_t ax_s,ay_s,az_s;
	uint8_t u8WaitCnt=0;
    float sensitivity = 0.061;	//default
	
	do{
		appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_STATUS_REG, 1,&tmp);
		if (u8WaitCnt++>30)
			break;
	}while(!(tmp&BIT(0)));

	#if 1	//calculate linear acceleration in mg
	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1,&tmp);
    tmp &= LSM6DS3_XL_FS_MASK;
//	printf("tmp(A) = 0x%2X\r\n",tmp);//debug
    switch(tmp)
    {
      case LSM6DS3_XL_FS_2G:
        sensitivity = 0.061;
        break;
      case LSM6DS3_XL_FS_4G:
        sensitivity = 0.122;
        break;
      case LSM6DS3_XL_FS_8G:
        sensitivity = 0.244;
        break;
      case LSM6DS3_XL_FS_16G:
        sensitivity = 0.488;
        break;
    }
	#endif	

	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_X_H_XL, 1, &tmpxh);
	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_X_L_XL, 1, &tmpxl);
	ax_s=(((int16_t) ((tmpxh << 8) | tmpxl)));
//	printf("ax_s:%d,%d,%d\r\n",ax_s,tmpxh,tmpxl);	//debug

	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_Y_H_XL, 1, &tmpyh);
	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_Y_L_XL, 1, &tmpyl);
	ay_s=(((int16_t) ((tmpyh << 8) | tmpyl)));
//	printf("ay_s:%d,%d,%d\r\n",ay_s,tmpyh,tmpyl);	//debug	

	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_Z_H_XL, 1, &tmpzh);
	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_OUT_Z_L_XL, 1, &tmpzl);
	az_s=(((int16_t) ((tmpzh << 8) | tmpzl)));
//	printf("az_s:%d,%d,%d\r\n",az_s,tmpzh,tmpzl);	//debug	

	appLSM6DS3_SetAccData(AXIS_X,(int16_t)(ax_s*sensitivity));	
	appLSM6DS3_SetAccData(AXIS_Y,(int16_t)(ay_s*sensitivity));	
	appLSM6DS3_SetAccData(AXIS_Z,(int16_t)(az_s*sensitivity));	

//	drvLSM6DS3_Delay(5);
}

void appLSM6DS3_GetGyro(void)
{
	uint8_t tmpxl, tmpxh, tmpyl, tmpyh, tmpzl, tmpzh, tmp;
	int16_t gx_s,gy_s,gz_s;
	uint8_t u8WaitCnt=0;
    float sensitivity = 8.75;	//default
    
	do{
		appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_STATUS_REG, 1,&tmp);
		u8WaitCnt++;
		if (u8WaitCnt>30)
			break;
	}while(!(tmp&BIT(1)));

	#if 1	//calculate angular rate in mdps
	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_CTRL2_G, 1,&tmp);
    tmp &= LSM6DS3_G_FS_MASK;
//	printf("tmp(G) = 0x%2X\r\n",tmp);//debug
    switch(tmp)
    {
      case LSM6DS3_G_FS_125:
        sensitivity = 4.375;
        break;
      case LSM6DS3_G_FS_245:
        sensitivity = 8.75;
        break;
      case LSM6DS3_G_FS_500:
        sensitivity = 17.50;
        break;
      case LSM6DS3_G_FS_1000:
        sensitivity = 35;
        break;
      case LSM6DS3_G_FS_2000:
        sensitivity = 70;
        break;
    }
	#endif	

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_X_H_G, 1, &tmpxh);
	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_X_L_G, 1, &tmpxl);
	gx_s=( (((int16_t)(tmpxh << 8)) | ((int16_t)tmpxl)));

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_Y_H_G, 1, &tmpyh);
	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_Y_L_G, 1, &tmpyl);
	gy_s=( (((int16_t)(tmpyh << 8)) | ((int16_t)tmpyl)));	

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_Z_H_G, 1, &tmpzh);
	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_OUT_Z_L_G, 1, &tmpzl);
	gz_s=( (((int16_t)(tmpzh << 8)) | ((int16_t)tmpzl)));	

	appLSM6DS3_SetGyroData(AXIS_X,(int16_t)(gx_s*sensitivity)/1000);	
	appLSM6DS3_SetGyroData(AXIS_Y,(int16_t)(gy_s*sensitivity)/1000);	
	appLSM6DS3_SetGyroData(AXIS_Z,(int16_t)(gz_s*sensitivity)/1000);	

//	drvLSM6DS3_Delay(5);
}

void appLSM6DS3_SetACC(void)
{
	uint8_t data;

	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1,&data);
//	drvLSM6DS3_Delay(5);	
	
    /* Output Data Rate selection */
    data &= ~(LSM6DS3_XL_ODR_MASK);
    data |= LSM6DS3_XL_ODR_1K66HZ;

    /* Full scale selection */
    data &= ~(LSM6DS3_XL_FS_MASK);
    data |= LSM6DS3_XL_FS_2G;
	
	appLSM6DS3_Write(ACC_ADDRESS, LSM6DS3_XG_CTRL1_XL, 1,&data);
//	drvLSM6DS3_Delay(5);

	appLSM6DS3_Read(ACC_ADDRESS, LSM6DS3_XG_CTRL9_XL, 1,&data);
//	drvLSM6DS3_Delay(5);	

    /* Enable X axis selection */
    data &= ~(LSM6DS3_XL_XEN_MASK);
    data |= LSM6DS3_XL_XEN_ENABLE;

    /* Enable Y axis selection */
    data &= ~(LSM6DS3_XL_YEN_MASK);
    data |= LSM6DS3_XL_YEN_ENABLE;

    /* Enable Z axis selection */
    data &= ~(LSM6DS3_XL_ZEN_MASK);
    data |= LSM6DS3_XL_ZEN_ENABLE;

	appLSM6DS3_Write(ACC_ADDRESS, LSM6DS3_XG_CTRL9_XL, 1,&data);
//	drvLSM6DS3_Delay(5);	
	
}

void appLSM6DS3_SetGyro(void)
{
	uint8_t data;

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_WHO_AM_I_ADDR, 1, &data);
//	drvLSM6DS3_Delay(5);

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_CTRL2_G, 1,&data);
//	drvLSM6DS3_Delay(5);	
	
    /* Output Data Rate selection */
    data &= ~(LSM6DS3_G_ODR_MASK);
    data |= LSM6DS3_G_ODR_1K66HZ;

    /* Full scale selection */
    data &= ~(LSM6DS3_G_FS_MASK);
    data |= LSM6DS3_G_FS_245;
	
	appLSM6DS3_Write(GYRO_ADDRESS, LSM6DS3_XG_CTRL2_G, 1,&data);
//	drvLSM6DS3_Delay(5);

	appLSM6DS3_Read(GYRO_ADDRESS, LSM6DS3_XG_CTRL10_C, 1,&data);
//	drvLSM6DS3_Delay(5);	

    /* Enable X axis selection */
    data &= ~(LSM6DS3_G_XEN_MASK);
    data |= LSM6DS3_G_XEN_ENABLE;

    /* Enable Y axis selection */
    data &= ~(LSM6DS3_G_YEN_MASK);
    data |= LSM6DS3_G_YEN_ENABLE;

    /* Enable Z axis selection */
    data &= ~(LSM6DS3_G_ZEN_MASK);
    data |= LSM6DS3_G_ZEN_ENABLE;

	appLSM6DS3_Write(GYRO_ADDRESS, LSM6DS3_XG_CTRL10_C, 1,&data);
//	drvLSM6DS3_Delay(5);	
	
}

void appLSM6DS3_CommonInit(void)
{
	uint8_t data;


	appLSM6DS3_Read(LSM6DS3_ADDRESS_HIGH, LSM6DS3_XG_CTRL3_C, 1,&data);
//	drvLSM6DS3_Delay(5);	

    /* Enable register address automatically incremented during a multiple byte
       access with a serial interface (I2C or SPI) */
    data &= ~(LSM6DS3_XG_IF_INC_MASK);
    data |= LSM6DS3_XG_IF_INC;
	
	appLSM6DS3_Write(LSM6DS3_ADDRESS_HIGH, LSM6DS3_XG_CTRL3_C, 1,&data);
//	drvLSM6DS3_Delay(5);

	appLSM6DS3_Read(LSM6DS3_ADDRESS_HIGH, LSM6DS3_XG_FIFO_CTRL5, 1,&data);
//	drvLSM6DS3_Delay(5);	

    /* FIFO ODR selection */
    data &= ~(LSM6DS3_XG_FIFO_ODR_MASK);
    data |= LSM6DS3_XG_FIFO_ODR_NA;

    /* FIFO mode selection */
    data &= ~(LSM6DS3_XG_FIFO_MODE_MASK);
    data |= LSM6DS3_XG_FIFO_MODE_BYPASS;

	appLSM6DS3_Write(LSM6DS3_ADDRESS_HIGH, LSM6DS3_XG_FIFO_CTRL5, 1,&data);
//	drvLSM6DS3_Delay(5);	
	
}

void appLSM6DS3_Setup(void)
{
	appLSM6DS3_CommonInit();
	appLSM6DS3_SetGyro();
	appLSM6DS3_SetACC();
}	

void appLSM6DS3_SelfTest(void)
{
	appLSM6DS3_GetAcc();
	appLSM6DS3_GetGyro();

	#if 0	//debug
	printf("ACC:%6d,%6d,%6d, GYRO:%6d,%6d,%6d\r\n",
				appLSM6DS3_GetAccData(AXIS_X),appLSM6DS3_GetAccData(AXIS_Y),appLSM6DS3_GetAccData(AXIS_Z),
				appLSM6DS3_GetGyroData(AXIS_X),appLSM6DS3_GetGyroData(AXIS_Y),appLSM6DS3_GetGyroData(AXIS_Z));
	#endif
}	

void appLSM6DS3_LowLevel_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(LSM6DS3_I2C_SCL_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(LSM6DS3_I2C_SDA_GPIO_CLK, ENABLE);

	GPIO_PinAFConfig(LSM6DS3_I2C_SCL_GPIO_PORT, LSM6DS3_I2C_SCL_SOURCE, LSM6DS3_I2C_SCL_AF);
	GPIO_PinAFConfig(LSM6DS3_I2C_SDA_GPIO_PORT, LSM6DS3_I2C_SDA_SOURCE, LSM6DS3_I2C_SDA_AF);

	/* Configure SCL */
	GPIO_InitStructure.GPIO_Pin = LSM6DS3_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(LSM6DS3_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

	/* Configure SDA */
	GPIO_InitStructure.GPIO_Pin = LSM6DS3_I2C_SDA_PIN;
	GPIO_Init(LSM6DS3_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
	
}

void appLSM6DS3_I2C_Init(void)
{ 
	I2C_InitTypeDef I2C_InitStructure;

	appLSM6DS3_LowLevel_Init();

	I2C_SoftwareResetCmd(LSM6DS3_I2C, ENABLE);

	I2C_DeInit(LSM6DS3_I2C);
	I2C_Cmd(LSM6DS3_I2C, DISABLE);

	/* LSM6DS3_I2C Periph clock enable */
	RCC_APB1PeriphClockCmd(LSM6DS3_I2C_CLK, ENABLE);

	/* Configure LSM6DS3_I2C for communication */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x33;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = LSM6DS3_I2C_SPEED;

	I2C_Cmd(LSM6DS3_I2C, ENABLE);
	I2C_Init(LSM6DS3_I2C, &I2C_InitStructure);
  
}

void appLSM6DS3_I2C_DeInit(void)
{
	/* sEE_I2C Peripheral Disable */
	I2C_Cmd(LSM6DS3_I2C,DISABLE);
	/* sEE_I2C DeInit */
	I2C_DeInit(LSM6DS3_I2C);
	/* sEE_I2C Periph clock disable */
	RCC_APB1PeriphClockCmd(LSM6DS3_I2C_CLK, DISABLE);
	
}	

#ifdef LSM6DS3_USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LSM6DS3_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
	appLSM6DS3_I2C_DeInit();
	appLSM6DS3_I2C_Init();
	return LSM6DS3_FAIL;
	
}
#endif /* LSM6DS3_USE_DEFAULT_TIMEOUT_CALLBACK */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
