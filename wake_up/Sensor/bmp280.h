#ifndef __BMP280_H
#define __BMP280_H


#include "stdbool.h"
#include <math.h>
#include "sys.h"
#include "delay.h"
#define BMP280_ADDR						(0xEC)
#define BMP280_DEFAULT_CHIP_ID			(0x58)

#define BMP280_CHIP_ID					(0xD0)         /* Chip ID Register */
#define BMP280_RST_REG					(0xE0)         /* Softreset Register */
#define BMP280_STAT_REG					(0xF3)         /* Status Register */
#define BMP280_CTRL_MEAS_REG			(0xF4)         /* Ctrl Measure Register */
#define BMP280_CONFIG_REG				(0xF5)         /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG			(0xF7)         /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG			(0xF8)         /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG		(0xF9)         /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG		(0xFA)         /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG		(0xFB)         /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG		(0xFC)         /* Temperature XLSB Reg */

#define BMP280_SLEEP_MODE				(0x00)
#define BMP280_FORCED_MODE				(0x01)
#define BMP280_NORMAL_MODE				(0x03)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_FRAME_SIZE			(6)

#define BMP280_OVERSAMP_SKIPPED			(0x00)
#define BMP280_OVERSAMP_1X				(0x01)
#define BMP280_OVERSAMP_2X				(0x02)
#define BMP280_OVERSAMP_4X				(0x03)
#define BMP280_OVERSAMP_8X				(0x04)
#define BMP280_OVERSAMP_16X				(0x05)

#define u8 unsigned char

bool bmp280Init(void);
void bmp280GetData(float* pressure, float* temperature, float* asl);



//IO��������
//#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
//#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO��������	 
//#define IIC_SCL    PBout(12)                                                   //SCL
//#define IIC_SDA    PBout(11)                                                   //SDA	 
//#define READ_SDA   PBin(11)                                                    //����SDA 

//IIC���в�������
void BMP_IIC_Init(void);                                                           //��ʼ��IIC��IO��				 
void BMP_IIC_Start(void);				                                           //����IIC��ʼ�ź�
void BMP_IIC_Stop(void);	  			                                           //����IICֹͣ�ź�
void BMP_IIC_Send_Byte(u8 txd);			                                           //IIC����һ���ֽ�
u8   BMP_IIC_Read_Byte(unsigned char ack);                                           //IIC��ȡһ���ֽ�
u8   BMP_IIC_Wait_Ack(void); 				                                           //IIC�ȴ�ACK�ź�
void BMP_IIC_Ack(void);					                                           //IIC����ACK�ź�
void BMP_IIC_NAck(void);				                                           //IIC������ACK�ź�

void BMP_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8   BMP_IIC_Read_One_Byte(u8 daddr,u8 addr);	
			 
u8   BMP_iicDevReadByte(u8 devaddr,u8 addr);	                                       /*��һ�ֽ�*/
void BMP_iicDevWriteByte(u8 devaddr,u8 addr,u8 data);	                           /*дһ�ֽ�*/
void BMP_iicDevRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf);                           /*������ȡ����ֽ�*/
void BMP_iicDevWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf);                          /*����д�����ֽ�*/

void SDA_IN(void);
void SDA_OUT(void);


#endif
