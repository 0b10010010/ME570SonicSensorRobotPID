#pragma once

class PID
{
	private:
		float _Kp, _Ki, _Kd;
		float _up, _ui, _ud;
		float _low_i_lim, _high_i_lim, _low_o_lim, _high_o_lim;
		float last_error, _output;
	public:
		PID(float Kp, float Ki, float Kd, float low_i_lim, float high_i_lim, float low_o_lim, float high_o_lim); // constructor
		float run(float error, float dt);
};