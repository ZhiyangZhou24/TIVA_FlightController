#include "MY_PWM.h"
#include "include.h"

void PWM_Init(void)
{
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);// PWM时钟配置  @500HZ
	//使能
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

	//配置PWM功能
	GPIOPinConfigure(GPIO_PB4_M0PWM2);
	GPIOPinConfigure(GPIO_PB5_M0PWM3);
	
	GPIOPinConfigure(GPIO_PE4_M0PWM4);
	GPIOPinConfigure(GPIO_PE5_M0PWM5);
	GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_4|GPIO_PIN_5);
	GPIOPinTypePWM(GPIO_PORTE_BASE,GPIO_PIN_4|GPIO_PIN_5);

	//配置PWM发生器0：加减计数，不同步
	PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN|PWM_GEN_MODE_NO_SYNC);
  //设置PWM发生器1的频率，时钟频率/PWM分频数/n，80M/64/5000=250HZ
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 5000);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 5000);
	//设置输出的脉冲宽度
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1250);  //初始值1ms
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 1250);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 1250);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 1250);
	//使能PWM发生器
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	PWMGenEnable(PWM0_BASE, PWM_GEN_2);
	//使能输出
	PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
}
//125hz 0<PWM<1250
void PWM_Go(unsigned int ch1,unsigned int ch2,unsigned int ch3,unsigned int ch4)
{
	if(ch1>1250) ch1 = 1250;
	if(ch2>1250) ch2 = 1250;
	if(ch3>1250) ch3 = 1250;
	if(ch4>1250) ch4 = 1250;
	//设置输出的脉冲宽度
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ch2+1250);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, ch1+1250);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ch3+1250);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, ch4+1250);
}
