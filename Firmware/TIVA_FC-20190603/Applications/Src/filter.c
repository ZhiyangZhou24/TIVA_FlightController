#include "include.h"
#include "filter.h"
#include "mymath.h"

// #define WIDTH_NUM 101
// #define FIL_ITEM  10

 void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out)
{
	u16 width_num;
	
	width_num = len ;
	
	if( ++fil_cnt[0] > width_num )	
	{
		fil_cnt[0] = 0; //now
		fil_cnt[1] = 1; //old
	}
	else
	{
		fil_cnt[1] = (fil_cnt[0] == width_num)? 0 : (fil_cnt[0] + 1);
	}
	
	moavarray[ fil_cnt[0] ] = in;
	*out += ( in - ( moavarray[ fil_cnt[1] ]  ) )/(float)( width_num ) ;
	
}


// float filter_tmp[FIL_ITEM][WIDTH_NUM ];
// float filter_out[FIL_ITEM];

// u8 fil_cnt[FIL_ITEM],fil_cnt_old[FIL_ITEM];

// float Moving_Average(u8 item,u8 width_num,float in)
// {
// 	if(item >= FIL_ITEM || width_num >= WIDTH_NUM )
// 	{
// 		return 0;
// 	}
// 	else
// 	{
// 		if( ++fil_cnt[item] > width_num )	
// 		{
// 			fil_cnt[item] = 0;
// 			fil_cnt_old[item] = 1;
// 		}
// 		else
// 		{
// 			fil_cnt_old[item] = (fil_cnt[item] == width_num)? 0 : (fil_cnt[item] + 1);
// 		}
// 		
// 		filter_tmp[item][ fil_cnt[item] ] = in;
// 		filter_out[item] += ( in - ( filter_tmp[item][ fil_cnt_old[item] ]  ) )/(float)( width_num ) ;//+ 0.01 *filter_out[item]
// 								//test_array( array ); /////////////////
// 		return ( filter_out[item] );
// 	}

// 	
// }



#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  2

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];

u8 med_fil_cnt[MED_FIL_ITEM];

float Moving_Median(u8 item,u8 width_num,float in)
{
	u8 i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	
	if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++med_fil_cnt[item] >= width_num )	
		{
			med_fil_cnt[item] = 0;
		}
		
		med_filter_tmp[item][ med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}

		
		return ( tmp[(u16)width_num/2] );
	}
}


void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out) //小范围内正确。
{
	static s8 pn;
	static float h_tmp_x,h_tmp_y;
	
	h_tmp_x = my_sqrt(my_pow(ref->z) + my_pow(ref->y));
	h_tmp_y = my_sqrt(my_pow(ref->z) + my_pow(ref->x));
	
	pn = ref->z < 0? -1 : 1;
	
	  out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
		out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
	
// 	 out->x = h_tmp_x *in->x - ref->x *in->z;
// 	 out->y = ref->z *in->y - ref->y *in->z;
	
	out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;

}
