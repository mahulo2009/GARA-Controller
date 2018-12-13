#include "WheelBLCD.h"

WheelBLCD::WheelBLCD() 
{
}

void WheelBLCD::move(float velocity)
{
    this->targetVelocity_=velocity;
	if (pid_ != 0) 
	{
		pid_->setTarget(this->targetVelocity_);
	}
}

void WheelBLCD::stop()
{
    this->targetVelocity_=0;
	if (pid_ != 0) 
	{
		pid_->setTarget(this->targetVelocity_);
	}  
}

//TODO WHO IS CALLING THIS METHOD
void WheelBLCD::update_()
{
    this->currentVelocity_ = 0; //TODO READ VALUE FROM HARDWARE

	if (pid_ != 0) 
	{
		this->demandedVelocity_= 
			pid_->update(this->currentVelocity_,this->period_pid_controller_); 
	} else 
	{
		this->demandedVelocity_ = targetVelocity_;
	}  
  	this->controller_->velocity(demandedVelocity_);

	#ifdef WHEEL_BLCD
  	Serial.print("WheelBLCD::update_:");
	Serial.print("\t");
  	Serial.print(targetVelocity_);
  	Serial.print("\t");
  	Serial.print(currentVelocity_);
  	Serial.print("\t");
  	Serial.print(this->demandedVelocity_);
  	Serial.print("\n");
  	#endif
}