#ifndef _APP_OPTICALFLOW_H_
#define _APP_OPTICALFLOW_H_

#include "include.h"
typedef struct{
	uint8_t    head;          //0x54
	uint8_t    id;            //'f' flow 
	uint16_t   qual;    
	/*���ٶȣ���λ������*/
	float       flow_x;        
	float       flow_y;
	float      flow_raw_x;  
	float      flow_raw_y;
	/*��ǲ�����λ������*/
	float      flowPixelCompX;
	float      flowPixelCompY;
	/*��ͨ�˲�֮���λ�����ݣ���λ������*/
	float      flow_raw_x_lpf;		/**< speed over ground in meters, rotation-compensated �ٶ�*/
	float      flow_raw_y_lpf;		/**< speed over ground in meters, rotation-compensated */
	
	/*�Եظ߶ȣ���λ����*/
	float      ground_distance;
	uint16_t   sum;           //20
	uint8_t    rev;
	uint8_t    tail;          //0xc6
}CameraFram;


/**
 * Optical flow in NED body frame in SI units.
 *
 */
typedef struct {

	uint64_t timestamp;		/**< in microseconds since system start          */

	uint64_t flow_timestamp;		/**< timestamp from flow sensor */
	int16_t  flow_comp_raw_x;		/**< flow in pixels in X direction, not rotation-compensated ����֮���λ��*/
	int16_t  flow_comp_raw_y;		/**< flow in pixels in Y direction, not rotation-compensated */
	float    flow_comp_x_m;		/**< speed over ground in meters, rotation-compensated ����֮����ٶ�*/
	float    flow_comp_y_m;		/**< speed over ground in meters, rotation-compensated */

	float    ground_distance_m;	/**< Altitude / distance to ground in meters */
	float    Vz;
	uint8_t	 quality;		/**< Quality of the measurement, 0: bad quality, 255: maximum quality */
	uint8_t  sensor_id;		/**< id of the sensor emitting the flow value */
  
	bool     optical_valid;  /*refresh flag*/
}
OpticalFlow;

#endif
