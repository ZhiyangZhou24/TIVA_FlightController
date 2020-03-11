#include "include.h"
#include "rc.h"
#include "mymath.h"
#include "icm_20602.h"
#include "FC_Config.h"


s8 CH_in_Mapping[CH_NUM] = {0,1,2,3,4,5};    //通道映射

void CH_Mapping_Fun(u16 *in,u16 *Mapped_CH)
{
	u8 i;
	for( i = 0 ; i < CH_NUM ; i++ )
	{
		*( Mapped_CH + i ) = *( in + CH_in_Mapping[i] );
	}
}

s16 CH[CH_NUM];

float CH_Old[CH_NUM];
float CH_filter[CH_NUM];
float CH_filter_Old[CH_NUM];
float CH_filter_D[CH_NUM];
u8 NS,CH_Error[CH_NUM];
u16 NS_cnt,CLR_CH_Error[CH_NUM];

s16 MAX_CH[CH_NUM]  = {1684 ,1684 ,1684 ,1684 ,1684 ,1684 };	//摇杆最大
s16 MIN_CH[CH_NUM]  = {364  ,364  ,364  ,364  ,364  ,364  };	//摇杆最小
char CH_DIR[CH_NUM] = {0    ,0    ,0    ,0    ,0    ,0    };  //摇杆方向
#define CH_OFFSET 0
//LEDmode 
extern unsigned char LED_Mode;
extern s16 DBUS_Link[CH_NUM];
extern s16 NRF_Link[CH_NUM];
extern unsigned int Pack_Num;    //1s时间内收包数
float filter_A;

void RC_Duty( float T , u16 tmp16_CH[CH_NUM] )
{
	u8 i;
	s16 CH_TMP[CH_NUM];
	
	for(i=0;i<4;i++)
	{
		
		if( !CH_DIR[i] )
		{
			CH[i] =   LIMIT ( (s16)( ( tmp16_CH[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - 500 ), -500, 500); //归一化，输出+-500
		}
		else
		{
			CH[i] = - LIMIT ( (s16)( ( tmp16_CH[i] - MIN_CH[i] )/(float)( MAX_CH[i] - MIN_CH[i] ) *1000 - 500 ), -500, 500); //归一化，输出+-500
		}
		if(i==1)CH[i] = -CH[i];//Pitch取反
//=================== filter ===================================
//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
//=================== filter =================================== 		
			
		filter_A = 3.14f *20 *T;
		
		if( ABS(CH_TMP[i] - CH_filter[i]) <100 )
		{
			CH_filter[i] += filter_A *(CH[i] - CH_filter[i]) ;
		}
		else
		{
			CH_filter[i] += 0.5f *filter_A *( CH[i] - CH_filter[i]) ;
		}
		CH_filter_D[i] 	= ( CH_filter[i] - CH_filter_Old[i] );
		CH_filter_Old[i] = CH_filter[i];
		CH_Old[i] 		= CH[i];
	}
	CH_filter[AUX1] = tmp16_CH[AUX1];
	CH_filter[AUX2] = tmp16_CH[AUX2];
	//======================================================================
	Fly_Ready(T);		//解锁判断
	//======================================================================
	if(++NS_cnt>200)  // 400ms  未插信号线。
	{
		NS_cnt = 0;
		NS = 0;
	}
}

u8 fly_ready = 0;
s16 ready_cnt=0;

void Fly_Ready(float T)
{
	if( CH_filter[2] < -400 )  							//油门小于10%
	{
		if( fly_ready && ready_cnt != -1 ) //解锁完成，且已退出解锁上锁过程
		{
			//ready_cnt += 1000 *T;
		}
#if(USE_TOE_IN_UNLOCK)		
		if( CH_filter[3] < -400 )							
		{
			if( CH_filter[1] > 400 )
			{
				if( CH_filter[0] > 400 )
				{
					if( ready_cnt != -1 )				   //外八满足且退出解锁上锁过程
					{
						ready_cnt += 3 *1000 *T;
					}
				}
			}
		}
#else
		if( CH_filter[3] < -400 )					      //左下满足		
		{
			if( ready_cnt != -1 && fly_ready )	//判断已经退出解锁上锁过程且已经解锁
			{
				ready_cnt += 1000 *T;
			}
		}
		else if( CH_filter[3] > 400 )      			//右下满足
		{
			if( ready_cnt != -1 && !fly_ready )	//判断已经退出解锁上锁过程且已经上锁
			{
				ready_cnt += 1000 *T;
			}
		}
#endif		
		else if( ready_cnt == -1 )						//4通道(CH[3])回位
		{
			ready_cnt=0;
		}
	}
	else
	{
		ready_cnt=0;
	}

	if( ready_cnt > 2000 ) // 1000ms 
	{
		ready_cnt = -1;
		//fly_ready = ( fly_ready==1 ) ? 0 : 1 ;
		if( !fly_ready )
		{
			fly_ready = 1;
			icm20602.Gyro_CALIBRATE = 2;
		}
		else
		{
			fly_ready = 0;
		}
	}
	
}

void Feed_Rc_Dog(u8 ch_mode) //400ms内必须调用一次
{
	NS = ch_mode;
	NS_cnt = 0;
}

//=================== filter ===================================
//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
//=================== filter =================================== 	
u8 height_ctrl_mode = 0;
extern u8 ultra_ok;
void Mode(u16 tmp16_CH[CH_NUM])
{
	if( !fly_ready || CH_filter[THR]<-400 ) //只在上锁时 以及 油门 低于10% 的时候，允许切换模式，否则只能向模式0切换。
	{
		if(tmp16_CH[AUX2]==1)
		{
			//icm20602.Acc_CALIBRATE = 1;
		}
		if(tmp16_CH[AUX2]==2)
		{
			//icm20602.Gyro_CALIBRATE = 1;
		}
		if(tmp16_CH[AUX1] ==1)
		{
			height_ctrl_mode = 0;
		}
		else if( tmp16_CH[AUX1] ==2 )
		{
			height_ctrl_mode = 1;
		}
		else
		{
			if(ultra_ok == 1)
			{
				height_ctrl_mode = 2;
			}
			else
			{
				height_ctrl_mode = 1;
			}
		}
	}
	else
	{
		if( tmp16_CH[AUX1] == 1 )
		{
			height_ctrl_mode = 0;
		}
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

