#include "BLCDHardwareController.h"

BLCDHardwareController::BLCDHardwareController(float max_speed,int min_duty, int max_duty,int can_id,int invert) : 
    max_speed_(max_speed), 
    min_duty_(min_duty), 
    max_duty_(max_duty),
    can_id_(can_id),
    invert_(invert)
{
    wheel_direction_ = FORWARD;
}

void BLCDHardwareController::setupDirection(Wheel_Direction direction) 
{
  switch(direction) 
  {
    case FORWARD:

        wheel_direction_ = FORWARD;
        break;

    case BACKWARD:

		wheel_direction_ = BACKWARD;
        break;
   }   
}

void BLCDHardwareController::velocity(float velocity)
{
    if (velocity < 0) 
    {
		setupDirection(BACKWARD);	
	} 
    else 
    {
  		setupDirection(FORWARD);
	}
    
    float duty = ( (max_duty_-min_duty_) * abs(velocity) ) / (max_speed_) + min_duty_;
    power(duty);
}

void BLCDHardwareController::power(float duty)
{
    bldc_interface_set_forward_can(can_id_);

    if ( wheel_direction_ == FORWARD )
    {
        bldc_interface_set_duty_cycle(duty * invert_);
    }
    else 
    {
        bldc_interface_set_duty_cycle(-duty * invert_);
    }
}