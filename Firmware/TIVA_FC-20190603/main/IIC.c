#include "IIC.h"

void IIC_INIT(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    GPIOPinTypeI2C(GPIO_PORTA_BASE,GPIO_PIN_7);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE,GPIO_PIN_6);

   // 语句的最后一个参数是用来设定数据传输速率的。
   // false表示传输速率是100kbps，true则意味着传输速率是400kbps。
   //此处使用的是100kbps的传输速率
   I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true);
   I2CMasterEnable(I2C1_BASE);
}
int i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data)
{
    //设置I2C1主模块将要放在总线上的从模块地址
    //false代表主模块发送，从模块接收
	unsigned char i;
    if(0 == length)
    {
    	return -1;
    }
	I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);
	I2CMasterDataPut(I2C1_BASE, reg_addr);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C1_BASE))
	{
	}
	// 主模块开始发送寄存器地址
	for(i=0; i<length; i++)
    {
		I2CMasterDataPut(I2C1_BASE, data[i]);
		// 主模块开始发送数据
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
		while(I2CMasterBusy(I2C1_BASE))
		{
		}
    }
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);
	while(I2CMasterBusy(I2C1_BASE))
	{
	}
	return 0;
}

int i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data)
{
	if(0 == length)
		 return -1;
	if(1 == length)
	{
    	I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);
    	I2CMasterDataPut(I2C1_BASE, reg_addr);
    	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    	while(I2CMasterBusy(I2C1_BASE))
        {
    	}
    	I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, true);
    	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    	while(I2CMasterBusy(I2C1_BASE))
        {
    	}
    	data[0] = I2CMasterDataGet(I2C1_BASE);
	}
	else
	{
    	unsigned char i;
    	I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, false);
    	I2CMasterDataPut(I2C1_BASE, reg_addr);
    	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    	while(I2CMasterBusy(I2C1_BASE))
    	{
    	}
    	I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, true);
    	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    	while(I2CMasterBusy(I2C1_BASE))
    	{
    	}
    	data[0] = I2CMasterDataGet(I2C1_BASE);
    	for(i=1; i<length-1; i++)
		{

    		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    		while(I2CMasterBusy(I2C1_BASE))
    		{
    		}
    		data[i] = I2CMasterDataGet(I2C1_BASE);
		}
    	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    	while(I2CMasterBusy(I2C1_BASE))
        {
        }
    	data[i] = I2CMasterDataGet(I2C1_BASE);
	}
	return 0;
}

void delay_ms(unsigned int n)
{
	SysCtlDelay(n*SysCtlClockGet()/3000);
}
void get_ms(unsigned long *time)
{

}
