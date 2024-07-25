#ifndef _MW_UORB_BUSSTRUCT_CONVERSION_H_
#define _MW_UORB_BUSSTRUCT_CONVERSION_H_

#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

typedef struct sensor_gyro_s  px4_Bus_sensor_gyro ;
typedef struct vehicle_attitude_s  px4_Bus_vehicle_attitude ;
typedef struct vehicle_local_position_s  px4_Bus_vehicle_local_position ;
typedef struct vehicle_local_position_setpoint_s  px4_Bus_vehicle_local_position_setpoint ;

#endif
