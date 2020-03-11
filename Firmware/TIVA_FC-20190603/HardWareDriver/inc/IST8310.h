#ifndef _IST8310_H
#define _IST8310_H

#include <stdint.h>
#include "time.h"

/* If I2C SEEPROM is tested, make sure FAST_MODE_PLUS is 0.
For board to board test, this flag can be turned on. */

#define IST8310_Addr 0x0C

#define ADDR_WAI                0x00		/* WAI means 'Who Am I'*/
# define WAI_EXPECTED_VALUE     0x10

#define ADDR_STAT1              0x02
# define STAT1_DRDY             (1 << 0)
# define STAT1_DRO              (1 << 1)

#define ADDR_DATA_OUT_X_LSB     0x03
#define ADDR_DATA_OUT_X_MSB     0x04
#define ADDR_DATA_OUT_Y_LSB     0x05
#define ADDR_DATA_OUT_Y_MSB     0x06
#define ADDR_DATA_OUT_Z_LSB     0x07
#define ADDR_DATA_OUT_Z_MSB     0x08

#define ADDR_STAT2              0x09
# define STAT2_INT              (1 << 3)

#define ADDR_CTRL1              0x0A
# define CTRL1_MODE_STDBY       (0 << 0)  //休眠模式
# define CTRL1_MODE_SINGLE      (1 << 0)  //测量模式

#define ADDR_CTRL2              0x0B
# define CTRL2_SRST             (1 << 0) //复位
# define CTRL2_DRP              (1 << 2) //DRDY引脚电平模式
# define CTRL2_DREN             (1 << 3) //数据准备好使能控制

#define ADDR_CTRL3				    0x41  /* Average 平均 滤波寄存器*/
# define CTRL3_SAMPLEAVG_16		0x24	/* Sample Averaging 16 */
# define CTRL3_SAMPLEAVG_8		0x1b	/* Sample Averaging 8 */
# define CTRL3_SAMPLEAVG_4		0x12	/* Sample Averaging 4 */
# define CTRL3_SAMPLEAVG_2		0x09	/* Sample Averaging 2 */

#define ADDR_CTRL4				0x42
# define CTRL4_SRPD				0xC0	/* Set Reset Pulse Duration 设定*/

#define ADDR_STR                0x0c
# define STR_SELF_TEST_ON       (1 << 6)
# define STR_SELF_TEST_OFF      (0 << 6)

#define ADDR_TEMPL              0x1c
#define ADDR_TEMPH              0x1d

#define ADDR_Y11_Low			0x9c
#define ADDR_Y11_High			0x9d
#define ADDR_Y12_Low			0x9e
#define ADDR_Y12_High			0x9f
#define ADDR_Y13_Low			0xa0
#define ADDR_Y13_High			0xa1
#define ADDR_Y21_Low			0xa2
#define ADDR_Y21_High			0xa3
#define ADDR_Y22_Low			0xa4
#define ADDR_Y22_High			0xa5
#define ADDR_Y23_Low			0xa6
#define ADDR_Y23_High			0xa7
#define ADDR_Y31_Low			0xa8
#define ADDR_Y31_High			0xa9
#define ADDR_Y32_Low			0xaa
#define ADDR_Y32_High			0xab
#define ADDR_Y33_Low			0xac
#define ADDR_Y33_High			0xad

#define I2CONSET_I2EN       (0x1<<6)  /* I2C Control Set Register */
#define I2CONSET_AA         (0x1<<2)
#define I2CONSET_SI         (0x1<<3)
#define I2CONSET_STO        (0x1<<4)
#define I2CONSET_STA        (0x1<<5)

#define I2CONCLR_AAC        (0x1<<2)  /* I2C Control clear Register */
#define I2CONCLR_SIC        (0x1<<3)
#define I2CONCLR_STAC       (0x1<<5)
#define I2CONCLR_I2ENC      (0x1<<6)

#define I2DAT_I2C           0x00000000  /* I2C Data Reg */
#define I2ADR_I2C           0x00000000  /* I2C Slave Address Reg */
#define I2SCLH_SCLH         0x00000180  /* I2C SCL Duty Cycle High Reg */
#define I2SCLL_SCLL         0x00000180  /* I2C SCL Duty Cycle Low Reg */
#define I2SCLH_HS_SCLH		  0x00000015  /* Fast Plus I2C SCL Duty Cycle High Reg */
#define I2SCLL_HS_SCLL		  0x00000015  /* Fast Plus I2C SCL Duty Cycle Low Reg */

#define IST8310_O_CONFIG_A     0x00 //Configuration Register A 
#define IST8310_O_CONFIG_B     0x01 //Configuration Register B
#define IST8310_O_MODE         0x02 //Mode Register
#define IST8310_O_XOUT_H       0x03 //Data Output X MSB Register
#define IST8310_O_XOUT_L       0x04 //Data Output X LSB Register
#define IST8310_O_ZOUT_H       0x05 //Data Output Z MSB Register
#define IST8310_O_ZOUT_L       0x06 //Data Output Z LSB Register
#define IST8310_O_YOUT_H       0x07 //Data Output Y MSB Register
#define IST8310_O_YOUT_L       0x08 //Data Output Y LSB Register
#define IST8310_O_STATUS       0x09 //Status Register
#define IST8310_O_ID_A         0x0A //Identification Register A
#define IST8310_O_ID_B         0x0B //Identification Register B
#define IST8310_O_ID_C         0x0C //Identification Register C 
#define IST8310_O_TS_H         0x31 //Temperature Output MSB Register
#define IST8310_O_TS_L         0x32 //Temperature Output LSB Register

typedef struct 
{
	xyz_s16_t Mag_Adc;			//采样值
	xyz_f_t   Mag_Offset;		//偏移值
	xyz_f_t 	Mag_Gain;			//比例缩放	
  xyz_f_t 	Mag_Val;			//纠正后的值
}ist8310_t;

extern ist8310_t ist8310;
extern u8 Mag_CALIBRATED;
extern u8 ist8310_ok;
/* IST8310相关 */
void IST8310_Init(void);
void IST8310_Task(void);
unsigned char	Write_IST8310(unsigned char reg,unsigned char *buf,unsigned char len);
unsigned char	Read_IST8310(unsigned char reg,unsigned char *buf,unsigned char len);
unsigned char IST8310_Read_Byte(unsigned char ack);

#endif

