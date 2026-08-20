#include "FOC_angle.h"



TorqueControlType torque_controller;

MotionControlType controller;





float sensor_offset=0;   //À∆∫ı√ª”√



float shaftAngle(void)

{

  return Dir1 * AS5600_I2C_Sensor_return(&as5600_encoder) - sensor_offset;

}



float shaftVelocity(void)

{

  return Dir1 * LPF_Update(&lpf1, AS5600_I2C_Sensor_getVelocity(&as5600_encoder));

}

