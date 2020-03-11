#include "MY_I2C.h"
#include "i2c.h"
#include "time.h"
#include "stdio.h"
/*�����ⲿIIC0���߳�ʼ��*/
void IIC0_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	GPIOPinTypeI2C(GPIO_PORTB_BASE,GPIO_PIN_3);
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE,GPIO_PIN_2);

	// �������һ�������������趨���ݴ������ʵġ�
	// false��ʾ����������100kbps��true����ζ�Ŵ���������400kbps��
	//�˴�ʹ�õ���100kbps�Ĵ�������
	I2CMasterInitExpClk(I2C0_BASE,SysCtlClockGet(), true);
	I2CMasterEnable(I2C0_BASE);
}
/*�ڲ�IIC1  ������ ��ѹ�� ��������*/
void IIC1_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	GPIOPinTypeI2C(GPIO_PORTA_BASE,GPIO_PIN_7);
	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE,GPIO_PIN_6);

	// �������һ�������������趨���ݴ������ʵġ�
	// false��ʾ����������100kbps��true����ζ�Ŵ���������400kbps��
	//�˴�ʹ�õ���100kbps�Ĵ�������
	I2CMasterInitExpClk(I2C1_BASE,SysCtlClockGet(), true);
	I2CMasterEnable(I2C1_BASE);
}
void I2C0_WriteByte(uint8_t Slave_Addr,uint8_t Reg_Addr,uint8_t Data)
{
	I2CMasterSlaveAddrSet(I2C0_BASE, Slave_Addr, false);
	I2CMasterDataPut(I2C0_BASE, Reg_Addr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE));
	I2CMasterDataPut(I2C0_BASE, Data);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
	while(I2CMasterBusy(I2C0_BASE));
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
	while(I2CMasterBusy(I2C0_BASE));
}
void I2C1_WriteByte(uint8_t Slave_Addr,uint8_t Reg_Addr,uint8_t Data)
{
	I2CMasterSlaveAddrSet(I2C1_BASE, Slave_Addr, false);
	I2CMasterDataPut(I2C1_BASE, Reg_Addr);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C1_BASE));
	I2CMasterDataPut(I2C1_BASE, Data);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
	while(I2CMasterBusy(I2C1_BASE));
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
	while(I2CMasterBusy(I2C1_BASE));
}

uint8_t I2C0_ReadByte(uint8_t Slave_Addr,uint8_t Reg_Addr)
{
	uint8_t data;
	I2CMasterSlaveAddrSet(I2C0_BASE, Slave_Addr, false);
	I2CMasterDataPut(I2C0_BASE, Reg_Addr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(I2C0_BASE));
	I2CMasterSlaveAddrSet(I2C0_BASE, Slave_Addr, true);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusy(I2C0_BASE));
	data = I2CMasterDataGet(I2C0_BASE);
	return data;
}
uint8_t I2C1_ReadByte(uint8_t Slave_Addr,uint8_t Reg_Addr)
{
	uint8_t data;
	I2CMasterSlaveAddrSet(I2C1_BASE, Slave_Addr, false);
	I2CMasterDataPut(I2C1_BASE, Reg_Addr);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(I2C1_BASE));
	I2CMasterSlaveAddrSet(I2C1_BASE, Slave_Addr, true);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusy(I2C1_BASE));
	data = I2CMasterDataGet(I2C1_BASE);
	return data;
}

uint8_t I2C0_Write(unsigned char slave_addr,unsigned char reg_addr,unsigned char length,unsigned char const *data)
{
	//����I2C1��ģ�齫Ҫ���������ϵĴ�ģ���ַ
	//false������ģ�鷢�ͣ���ģ�����
	unsigned char i;
	unsigned char status=1;
	unsigned int j;
	if(0 == length)
	{
		return 0;
	}
	I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
	I2CMasterDataPut(I2C0_BASE, reg_addr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	j=8000;while(I2CMasterBusy(I2C0_BASE)){j--;if(j==0){status=0;break;}}
	// ��ģ�鿪ʼ���ͼĴ�����ַ
	for(i=0; i<length; i++)
  {
		I2CMasterDataPut(I2C0_BASE, data[i]);
		// ��ģ�鿪ʼ��������
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
		j=8000;while(I2CMasterBusy(I2C0_BASE)){j--;if(j==0){status=0;break;}}
  }
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
	j=8000;while(I2CMasterBusy(I2C0_BASE)){j--;if(j==0){status=0;break;}}
	return status;
}
uint8_t I2C1_Write(unsigned char slave_addr,unsigned char reg_addr,unsigned char length,unsigned char const *data)
{
	//����I2C1��ģ�齫Ҫ���������ϵĴ�ģ���ַ
	//false������ģ�鷢�ͣ���ģ�����
	unsigned char status=1;
	unsigned int j;
	unsigned char i;
	if(0 == length)
	{
		return 0;
	}
	I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);
	I2CMasterDataPut(I2C1_BASE, reg_addr);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	j=8000;while(I2CMasterBusy(I2C1_BASE)){j--;if(j==0){status=0;break;}}
	// ��ģ�鿪ʼ���ͼĴ�����ַ
	for(i=0; i<length; i++)
  {
		I2CMasterDataPut(I2C1_BASE, data[i]);
		// ��ģ�鿪ʼ��������
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
		j=8000;while(I2CMasterBusy(I2C1_BASE)){j--;if(j==0){status=0;break;}}
  }
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
	j=8000;while(I2CMasterBusy(I2C1_BASE)){j--;if(j==0){status=0;break;}}
	return status;
}
uint8_t I2C0_Read(unsigned char slave_addr,unsigned char reg_addr,unsigned char length,unsigned char *data)
{
	unsigned char status=1;
	unsigned int j;
	if(0 == length)
		 return 0;
	if(1 == length)
	{
		I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
		I2CMasterDataPut(I2C0_BASE, reg_addr);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		j=8000;while(I2CMasterBusy(I2C0_BASE)){j--;if(j==0){status=0;break;}}
		I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
		j=8000;while(I2CMasterBusy(I2C0_BASE)){j--;if(j==0){status=0;break;}}
		data[0] = I2CMasterDataGet(I2C0_BASE);
	}
	else
	{
		unsigned char i;
		I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
		I2CMasterDataPut(I2C0_BASE, reg_addr);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		j=8000;while(I2CMasterBusy(I2C0_BASE)){j--;if(j==0){status=0;break;}}
		I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
		j=8000;while(I2CMasterBusy(I2C0_BASE)){j--;if(j==0){status=0;break;}}
		data[0] = I2CMasterDataGet(I2C0_BASE);
		for(i=1; i<length-1; i++)
		{
			I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
			j=8000;while(I2CMasterBusy(I2C0_BASE)){j--;if(j==0){status=0;break;}}
			data[i] = I2CMasterDataGet(I2C0_BASE);
		}
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		j=8000;while(I2CMasterBusy(I2C0_BASE)){j--;if(j==0){status=0;break;}}
		data[i] = I2CMasterDataGet(I2C0_BASE);
	}
	return status;
}
uint8_t I2C1_Read(unsigned char slave_addr,unsigned char reg_addr,unsigned char length,unsigned char *data)
{
	unsigned char status=1;
	unsigned int j;
	if(0 == length)
		 return -0;
	if(1 == length)
	{
		I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);
		I2CMasterDataPut(I2C1_BASE, reg_addr);
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		j=8000;while(I2CMasterBusy(I2C1_BASE)){j--;if(j==0){status=0;break;}}
		I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, true);
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
		j=8000;while(I2CMasterBusy(I2C1_BASE)){j--;if(j==0){status=0;break;}}
		data[0] = I2CMasterDataGet(I2C1_BASE);
	}
	else
	{
		unsigned char i;
		I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);
		I2CMasterDataPut(I2C1_BASE, reg_addr);
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		j=8000;while(I2CMasterBusy(I2C1_BASE)){j--;if(j==0){status=0;break;}}
		I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, true);
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
		j=8000;while(I2CMasterBusy(I2C1_BASE)){j--;if(j==0){status=0;break;}}
		data[0] = I2CMasterDataGet(I2C1_BASE);
		for(i=1; i<length-1; i++)
		{
			I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
			j=8000;while(I2CMasterBusy(I2C1_BASE)){j--;if(j==0){status=0;break;}}
			data[i] = I2CMasterDataGet(I2C1_BASE);
		}
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		j=8000;while(I2CMasterBusy(I2C1_BASE)){j--;if(j==0){status=0;break;}}
		data[i] = I2CMasterDataGet(I2C1_BASE);
	}
	return status;
}

//��ʼ��MPU9250
bool MPU_Init(void)
{
    IIC1_Init();

    I2C1_WriteByte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU9250
    Delay_ms(50);
    I2C1_WriteByte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU9250
    Delay_ms(50);
    I2C1_WriteByte(MPU6050_ADDR,27,0x18);   //�����Ǵ�����,��2000dps
    I2C1_WriteByte(MPU6050_ADDR,28,0x18);   //���ٶȴ�����,��16g
    I2C1_WriteByte(MPU6050_ADDR,MPU_CFG_REG,0X03);   //���ֵ�ͨ�˲� ��ʱA4.9ms G4.8ms
    I2C1_WriteByte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,0X03);  //���ò�����250Hz
    I2C1_WriteByte(MPU6050_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
    I2C1_WriteByte(MPU6050_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
    I2C1_WriteByte(MPU6050_ADDR,MPU_FIFO_EN_REG,0X00);  //�ر�FIFO
    I2C1_WriteByte(MPU6050_ADDR,MPU_INTBP_CFG_REG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
    if(I2C1_ReadByte(MPU6050_ADDR,WHO_AM_I)!=0x68)
    {
			printf("ID:0x%x\r\n",I2C1_ReadByte(MPU6050_ADDR,WHO_AM_I));
      return false;
    }
    else 
    {   
			printf("ID:0x%x\r\n",I2C1_ReadByte(MPU6050_ADDR,WHO_AM_I));
      return true;
    }
}

//**************************************
//�ϳ�����=====>>�������κμĴ���
//**************************************
int GetData(unsigned char REG_Address)
{
	char H,L;
	H=I2C1_ReadByte(MPU6050_ADDR,REG_Address);
	L=I2C1_ReadByte(MPU6050_ADDR,REG_Address+1);
	return (H<<8)+L;
}
//MPU6050��ȡ�¶�
double get_temp(void)
{
	double Temperature;
	int Temp_l,Temp_h;
	Temp_h=I2C1_ReadByte(MPU6050_ADDR,MPU_TEMP_OUTH_REG);
	Temp_l=I2C1_ReadByte(MPU6050_ADDR,MPU_TEMP_OUTL_REG);
	Temperature=Temp_h<<8|Temp_l;
	Temperature = 36.53f+(Temperature)/340;
	return Temperature;
}



