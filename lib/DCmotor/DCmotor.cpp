#include "DCmotor.hpp"

DCmotor::DCmotor(u8 dir_pin_, u8 pwm_pin_)
: dir_pin(dir_pin_), pwm_pin(pwm_pin_)
{
	pinMode(dir_pin, OUTPUT);
	digitalWrite(dir_pin, LOW);

	//analogWriteFreq(5000);
	analogWriteResolution(8);
	analogWrite(pwm_pin, 0);
}

void DCmotor::set_accel_time(ut new_accel_time)
{
	accel_time = new_accel_time; 
	//accel_dt = 1;
}

void DCmotor::set_speed(u8 new_speed)
{
	speed = new_speed;
	if(stop_flag == 0){
		active_speed = speed;
	}
	//active_speed = speed;
	//accel_dt = accel_time / speed;
}

void DCmotor::set_dir(bool dir)
{
	digitalWrite(dir_pin, dir);
}

void DCmotor::change_dir()
{
	digitalWrite(dir_pin, !digitalRead(dir_pin));
}

u8 DCmotor::get_current_speed()
{
	return curr_speed;
}

void DCmotor::start(){
	active_speed = speed;
	stop_flag = 0;
}

void DCmotor::stop(){
	stop_flag = 1;
	active_speed = 0;
}

void DCmotor::fast_stop()
{
	stop_flag = 1;
	curr_speed = 0;
	analogWrite(pwm_pin, curr_speed);
}

void DCmotor::accel(int8_t accel_dir)
{
	if (millis() > last_accel + accel_dt)
	{
		last_accel = millis();
		analogWrite(pwm_pin, curr_speed += accel_dir);
	}
}

void DCmotor::drive()
{
	if (curr_speed != active_speed)
	{
		if (curr_speed < active_speed)
		{
			accel(UP);
		}
		else
		{
			accel(DOWN);
		}
	}
}