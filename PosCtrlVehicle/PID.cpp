#include "PID.h"

PID::PID(float Kp, float Ki, float Kd, float low_i_lim, float high_i_lim, float low_o_lim, float high_o_lim)
{
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;
	_low_i_lim = low_i_lim;
	_high_i_lim = high_i_lim;
	_low_o_lim = low_o_lim;
	_high_o_lim = high_o_lim;
}

float PID::run(float error, float dt)
{
	_up = error*_Kp;
	_ui += _Ki * error * dt;
	if (_ui < _low_i_lim) { _ui = _low_i_lim; }
	if (_ui > _high_i_lim){ _ui = _high_i_lim;}
	_ud = _Kd * ((error - last_error) / dt);
	last_error = error;
	
	if (dt == 0) { _output = _up + _ui; } // ignore division by zero
	else 		 { _output = _up + _ui + _ud; }
	
	if (_output < _low_o_lim) { _output = _low_o_lim; }
	if (_output > _high_o_lim){ _output = _high_o_lim;}

	return _output;
}