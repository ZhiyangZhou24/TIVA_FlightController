#include "scheduler_fc.h"
#include "include.h"
#include "time.h"
#include "rc.h"
#include "imu.h"
#include "ctrl.h"
#include "parameter.h"
#include "height_ctrl.h"
#include "ultrasonic.h"
#include "led_task.h"
#include "app_radiolink.h"
#include "data_transfer.h"
#include "ms5611.h"
#include "ist8310.h"
#include "nrf24l01.h"
#include "my_i2c.h"
s16 loop_cnt;

loop_t loop;
extern unsigned char RF_Reciver;
unsigned char ESC_Calibration=0;
unsigned long Cali_time1=0,Cali_time2=0;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
}
//LED控制
unsigned char LED_Mode=0;
extern unsigned int Now_PackNum;    //1s时间内收包数
void Duty_1ms()
{
	Get_Cycle_T(1);
	LED_Display(height_ctrl_mode,fly_ready);								//20级led渐变显示
	ANO_DT_Data_Exchange();
}
/*内环PID*/
float test[5];
void Duty_2ms()
{
	float inner_loop_time;
	
	inner_loop_time = Get_Cycle_T(0); 						//获取内环准确的执行周期
	
	test[0] = GetSysTime_us()/1000000.0f;

	icm20602_Data_Prepare( inner_loop_time );			//ICM6轴传感器数据读取，处理

	CTRL_1( inner_loop_time ); 										//内环角速度控制。输入：执行周期，期望角速度，测量角速度，角度前馈；输出：电机PWM占空比。<函数未封装>
	
  if(RF_Reciver == DJI_DR16){RC_Duty( inner_loop_time , DBUS_Link );}
	else {RC_Duty( inner_loop_time , NRF_Link );}
	
	if(Cali_time1<200&&CH_filter[2]>450)
	{
		ESC_Calibration = 1;
	}
	if(ESC_Calibration)
	{
		PWM_Go(CH_filter[2]+500,CH_filter[2]+500,CH_filter[2]+500,CH_filter[2]+500);
	}
	if(CH_filter[2]<-450)
	{
		Cali_time2++;
		if(Cali_time2>200){ESC_Calibration=0;}
	}
	Cali_time1++;
	test[1] = GetSysTime_us()/1000000.0f;
}
/*外环PID控制*/
void Duty_5ms()
{
	float outer_loop_time;
	outer_loop_time = Get_Cycle_T(2);								//获取外环准确的执行周期
	test[2] = GetSysTime_us()/1000000.0f;
	/*AHRS更新姿态。输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿态角*/
 	AHRS_update(0.5f *outer_loop_time,
							icm20602.Gyro_deg.x,icm20602.Gyro_deg.y,icm20602.Gyro_deg.z,
							icm20602.Acc.x,icm20602.Acc.y,icm20602.Acc.z,
							ist8310.Mag_Val.x,ist8310.Mag_Val.y,ist8310.Mag_Val.z,
							&Roll,&Pitch,&Yaw);
 	CTRL_2( outer_loop_time ); 											// 外环角度控制。输入：执行周期，期望角度（摇杆量），姿态角度；输出：期望角速度。<函数未封装>
	test[3] = GetSysTime_us()/1000000.0f;
}
//更新ms5611气压计数据
void Duty_10ms()
{
	if( MS5611_Update() ) 				//更新ms5611气压计数据
	{	
		baro_ctrl_start = 1;  //20ms
	}
		
}

//保存参数任务
void Duty_20ms()
{
	Parameter_Save();
	if(RF_Reciver == NRF24L01)
	NRF_Task();
	IST8310_Task(); //获取电子罗盘数据
}
//接收机失联判断
unsigned char RF_Lost=0,Lost_Time=0;;
void Duty_50ms()
{
	if(RF_Reciver == DJI_DR16){Mode(DBUS_Link);}  //模式切换
	else {Mode(NRF_Link);}
	
	Ultra_Duty();  //超声波测距触发
	/*接收机失联，上锁飞控*/
	if(Now_PackNum<1){Lost_Time++;}
	if(Now_PackNum>20){Lost_Time = 0;}
	if(Lost_Time>5){fly_ready = 0;}
}

void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{
	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//周期1ms的任务,LED任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		loop.check_flag = 0;		//循环运行完毕标志
	}
}
