#pragma once

#include <Arduino.h>

#define DIR_CW 0
#define DIR_CCW 1

#ifndef ut
#define ut unsigned long
#endif

typedef uint8_t u8;

class DCmotor
{
	u8 dir_pin;
	u8 pwm_pin;

	u8 speed = 50;
	u8 curr_speed = 0;
	u8 active_speed = 0;

	ut last_accel = 0;
	ut accel_time = 1000;
	ut accel_dt = 1; //accel_time / speed;

	bool stop_flag = 0;

	const int8_t UP = 1;
	const int8_t DOWN = -1;

public:
	DCmotor(u8 dir_pin, u8 pwm_pin);

	void set_accel_time(ut new_accel_time);
	void set_speed(u8 new_speed);
	void set_dir(bool dir);
	void change_dir();
	u8 get_current_speed();

	void start();
	void stop();
	void fast_stop();

	void accel(int8_t accel_dir);
	void drive();
};
