#ifndef __FOC_angle_H

#define __FOC_angle_H



#include "bsp_system.h" // 根据你的芯片型号修改



typedef enum

{

	Type_torque,//!< Torque control

	Type_velocity,//!< Velocity motion control

	Type_angle,//!< Position/angle motion control

	Type_velocity_openloop,

	Type_angle_openloop

} MotionControlType;



typedef enum

{

	Type_voltage, //!< Torque control using voltage

	Type_dc_current, //!< Torque control using DC current (one current magnitude)

	Type_foc_current //!< torque control using dq currents

} TorqueControlType;



extern TorqueControlType torque_controller;

extern MotionControlType controller;



float shaftAngle(void);

float shaftVelocity(void);



#endif



