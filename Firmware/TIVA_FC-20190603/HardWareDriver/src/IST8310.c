#include "include.h"
#include "IST8310.h"
#include "parameter.h"
#include "my_i2c.h"
#include "time.h"
#include "matrix.h"

/* ist8310 Slave Address Select : default address 0x0C
 *        CAD1  |  CAD0   |  Address
 *    ------------------------------
 *         VSS   |   VSS  |  0CH
 *         VSS   |   VDD  |  0DH
 *         VDD   |   VSS  |  0EH
 *         VDD   |   VDD  |  0FH
 * if CAD1 and CAD0 are floating, I2C address will be 0EH
 *
 *
 * CTRL_REGA: Control Register 1
 * Read Write
 * Default value: 0x0A
 * 7:4  0   Reserved.
 * 3:0  DO2-DO0: Operating mode setting
 *        DO3  |  DO2 |  DO1 |  DO0 |   mode
 *    ------------------------------------------------------
 *         0   |   0  |  0   |  0   |   Stand-By mode
 *         0   |   0  |  0   |  1   |   Single measurement mode
 *                                       Others: Reserved
 *
 * CTRL_REGB: Control Register 2
 * Read Write
 * Default value: 0x0B
 * 7:4  0   Reserved.
 * 3    DREN : Data ready enable control: 
 *      0: disable 
 *      1: enable
 * 2    DRP: DRDY pin polarity control
 *      0: active low
 *      1: active high
 * 1    0   Reserved.
 * 0    SRST: Soft reset, perform Power On Reset (POR) routine
 *      0: no action
 *      1: start immediately POR routine
 *      This bit will be set to zero after POR routine
 */


ist8310_t ist8310 = { {0,0,0},{-1,-1,-1},{1,0.8538,0.9389},{0,0,0} };
#define CALIBRATING_MAG_CYCLES              1000  //У׼ʱ�����20s

uint8_t  Magnetic_data[4];     //�����ʹų�����
uint8_t  Original_Magnetic[6]; //ԭʼ�ų�����

unsigned int   magadd,magnum;
unsigned char  Config_IST8310[4];    //���������ò���
unsigned char  Magnetic_data[4];     //�����ʹų�����
unsigned char  Original_Magnetic[6]; //ԭʼ�ų�����
float angle;
float Corr_matri_X[3][3];    //����������ԭʼ����

//IST8310У׼����
void calibration_parameters()
{
	unsigned int   i,j,k;
  unsigned char  Cross_Axis_data[18];//�����Ĵ�������
	struct _Matrix M1,N1,N2;  //��������
	float buff_M1[9],buff_N1[9],buff_N2[9];
	float data_buf[9];
	
	Read_IST8310(ADDR_Y11_Low,Cross_Axis_data,18);//��ȡ�����Ĵ�������
	
	//��ʼ������ 
	//matrix_init(&M1);
	matrix_set_m(&M1,3);
	matrix_set_n(&M1,3);
	M1.arr=(float*)buff_M1;
	
 // matrix_init(&N1);
	matrix_set_m(&N1,3);
	matrix_set_n(&N1,3);
	N1.arr=(float*)buff_N1;
	
  //matrix_init(&N2);
	matrix_set_m(&N2,3);
	matrix_set_n(&N2,3);
	N2.arr=(float*)buff_N2;
	
	//�������� ����
	matrix_write(&M1,0,0,50.0);matrix_write(&M1,0,1,0);   matrix_write(&M1,0,2,0);
	matrix_write(&M1,1,0,0);   matrix_write(&M1,1,1,50.0);matrix_write(&M1,1,2,0);
	matrix_write(&M1,2,0,0);   matrix_write(&M1,2,1,0);   matrix_write(&M1,2,2,50.0);
  
	//�Ĵ�������ת�� �������
	for (k=0,i = 0;i < 3;i++)
	{
		for (j = 0;j < 3;j++)
		{
		  Corr_matri_X[i][j]=(float)((short)((unsigned short)Cross_Axis_data[k*2+1]<<8|(unsigned short)Cross_Axis_data[k*2]))*(float)(3.0/20.0);
			data_buf[k]=Corr_matri_X[i][j];
			matrix_write(&N1,j,i,data_buf[k++]);
		}
	}
	//�������  N2=N1����
  matrix_inverse(&N1,&N2);    
  //�������  N1=M1*N2	
	matrix_multiply(&M1,&N2,&N1);   
	//װ�ص���ά���� ���ڳ������
	for(k=0,i=0;i<3;i++){for(j=0;j<3;j++){Corr_matri_X[i][j]=matrix_read(&N1,i,j);}}
	//�ͷ�ռ���ڴ�ռ�
	matrix_free(&M1);matrix_free(&N1);matrix_free(&N2);
}
unsigned char  ist8310_ok=0;	
void IST8310_Init(void)
{
	unsigned char  Cfg_data;
	unsigned char ID,dat;
	//��ȡ���� ID   Ӧ���� ID=0x10
  ID = I2C1_ReadByte(IST8310_Addr,ADDR_WAI);
	if(ID==0x10)
	{
		ist8310_ok = 1;
		Cfg_data=CTRL2_SRST;         Write_IST8310(ADDR_CTRL2,&Cfg_data,1);//�����λ
		while(dat&0x01)
		{
			dat = I2C1_ReadByte(IST8310_Addr,ADDR_CTRL2);//�ȴ���λ���
		}
		Cfg_data=CTRL1_MODE_STDBY;   Write_IST8310(ADDR_CTRL1,&Cfg_data,1);Delay_ms(5);//����ģʽ  �Ա��������
		Cfg_data=0x00;               Write_IST8310(ADDR_CTRL2,&Cfg_data,1);Delay_ms(5);//����������׼������
		Cfg_data=CTRL3_SAMPLEAVG_16; Write_IST8310(ADDR_CTRL3,&Cfg_data,1);Delay_ms(5);//���ò���ƽ����  ���� ��16�ڲ�ƽ���˲� ������
		Cfg_data=CTRL4_SRPD;         Write_IST8310(ADDR_CTRL4,&Cfg_data,1);Delay_ms(5);//Ĭ������
		Cfg_data=STR_SELF_TEST_OFF;  Write_IST8310(ADDR_STR,&Cfg_data,1);  Delay_ms(5);  //�ر��Բ�ģʽ
		Cfg_data=CTRL1_MODE_SINGLE;  Write_IST8310(ADDR_CTRL1,&Cfg_data,1);Delay_ms(5);//��ʼ����
	}
	else {ist8310_ok = 0;}
	calibration_parameters();
}

u8 Mag_CALIBRATED = 0;
//�������е����

void IST8310_CalOffset_Mag(void)
{
	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
	static uint16_t cnt_m=0;
	
	if(Mag_CALIBRATED)
	{	
		
		if(ABS(ist8310.Mag_Adc.x)<400&&ABS(ist8310.Mag_Adc.y)<400&&ABS(ist8310.Mag_Adc.z)<400)
		{
			MagMAX.x = MAX(ist8310.Mag_Adc.x, MagMAX.x);
			MagMAX.y = MAX(ist8310.Mag_Adc.y, MagMAX.y);
			MagMAX.z = MAX(ist8310.Mag_Adc.z, MagMAX.z);
			
			MagMIN.x = MIN(ist8310.Mag_Adc.x, MagMIN.x);
			MagMIN.y = MIN(ist8310.Mag_Adc.y, MagMIN.y);
			MagMIN.z = MIN(ist8310.Mag_Adc.z, MagMIN.z);		
			
			if(cnt_m == CALIBRATING_MAG_CYCLES)
			{
				ist8310.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				ist8310.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				ist8310.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
	
				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;
				
				ist8310.Mag_Gain.y = MagSum.x / MagSum.y;
				ist8310.Mag_Gain.z = MagSum.x / MagSum.z;
				
				Save_Mag_Calibration_Date(&ist8310.Mag_Offset);      //��������
				cnt_m = 0;
				Mag_CALIBRATED = 0;
			}
		}
		cnt_m++;
	}
	else
	{

	}
}


void IST8310_Task(void)
{
	unsigned char  Cfg_data;
  short   x=0,y=0,z=0,xx=0,yy=0,zz=0;
  unsigned char  getbuf[6]={0};
	//��ȡ����������
	if((Read_IST8310(ADDR_DATA_OUT_X_LSB,getbuf,6)==0x01)&&ist8310_ok)
	{
		magadd++;
		x=((((short)getbuf[1]) << 8) | ((short)getbuf[0]));
		y=((((short)getbuf[3]) << 8) | ((short)getbuf[2]));
		z=((((short)getbuf[5]) << 8) | ((short)getbuf[4]));

		xx=(short)(Corr_matri_X[0][0]*(float)x+Corr_matri_X[0][1]*(float)y+Corr_matri_X[0][2]*(float)z);
		yy=(short)(Corr_matri_X[1][0]*(float)x+Corr_matri_X[1][1]*(float)y+Corr_matri_X[1][2]*(float)z);
		zz=(short)(Corr_matri_X[2][0]*(float)x+Corr_matri_X[2][1]*(float)y+Corr_matri_X[2][2]*(float)z);

		angle= atan2((double)yy,(double)xx) * (180.0 / 3.14159265) + 180.0;

		ist8310.Mag_Adc.x = xx;
		ist8310.Mag_Adc.y = yy;
		ist8310.Mag_Adc.z = zz;

		ist8310.Mag_Val.x = (ist8310.Mag_Adc.x - ist8310.Mag_Offset.x) ;
		ist8310.Mag_Val.y = (ist8310.Mag_Adc.y - ist8310.Mag_Offset.y) ;
		ist8310.Mag_Val.z = (ist8310.Mag_Adc.z - ist8310.Mag_Offset.z) ;
		//�������е����	
		IST8310_CalOffset_Mag();

		//ÿ�ζ�Ҫ��������  ��������Ҫ�ȴ�6ms 16��ƽ���˲������ʱ��
		Cfg_data=CTRL1_MODE_SINGLE; Write_IST8310(ADDR_CTRL1,&Cfg_data,1);
	}
}

unsigned char	Write_IST8310(unsigned char reg,unsigned char *buf,unsigned char len)
{
	return I2C1_Write(IST8310_Addr,reg,len,buf);
}

unsigned char	Read_IST8310(unsigned char reg,unsigned char *buf,unsigned char len)
{
	
	return I2C1_Read(IST8310_Addr,reg,len,buf);
}

// end

