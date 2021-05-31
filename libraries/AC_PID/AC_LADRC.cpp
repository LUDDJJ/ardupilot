#include <AP_Math/AP_Math.h>
#include "AC_LADRC.h"

const AP_Param::GroupInfo AC_LADRC::var_info[] = {
	// @Param: Wo
	// @DisplayName: LADRC Observer bandwidth
	// @Description: none
	AP_GROUPINFO("Wo", 0, AC_LADRC, _wo, 1),

	// @Param: Wc
	// @DisplayName: LADRC Control bandwidth
	// @Description: none
	AP_GROUPINFO("Wc", 1, AC_LADRC, _wc, 1),

	// @Param: b
	// @DisplayName: LADRC Observer input coefficient
	// @Description: none
	AP_GROUPINFO("b", 2, AC_LADRC, _b, 1),

	// @Param: r
	// @DisplayName: Linear TD coefficient
	// @Description: none
	AP_GROUPINFO("r", 3, AC_LADRC, _r, 1),

	// @Param: FF
	// @DisplayName: FF FeedForward Gain
	// @Description: FF Gain which produces an output value that is proportional to the demanded input
	AP_GROUPINFO("FF", 4, AC_LADRC, _kff, 0),

	AP_GROUPEND
};

// Constructor
AC_LADRC::AC_LADRC(float initial_wc, float initial_wo, float initial_b, float initial_r, float initial_ff, float dt) :
	_dt(dt),
	_z3(0.0f)
{
	// load parameter values from eeprom
	AP_Param::setup_object_defaults(this, var_info);

	_wo = initial_wc;
	_wc = initial_wo;
	_b = initial_b;
	_r = initial_r;
	_kff = initial_ff;


	// reset input filter to first value received
	_flags._reset_filter = true;


	//TODO :log
	memset(&_ladrc_info, 0, sizeof(_ladrc_info));


}

// set_dt - set time step in seconds
void AC_LADRC::set_dt(float dt)
{
	// set dt and calculate the input filter alpha
	_dt = dt;
}


//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
float AC_LADRC::update_all(float target, float measurement, bool limit)
{
	// don't process inf or NaN
	if (!isfinite(target) || !isfinite(measurement)) {
		return 0.0f;
	}

	// reset input filter to value received
	if (_flags._reset_filter) {
		_flags._reset_filter = false;
		_v2=0.0f;
		_v1=0.0f;
	}
	else {
		update_td(target);
	}
	
	update_eso(measurement);
	update_linearPD();

	// control output limit
	constrain_float(_u,-1.0f,1.0f);

	//todo :log ADRC data
	_ladrc_info.TDv1 = _v1;
	_ladrc_info.TDv2 = _v2;
	_ladrc_info.PD = _u;
	_ladrc_info.z1 = _z1;
	_ladrc_info.z2 = _z2;
	_ladrc_info.z3 = _z3;

	return _u;
}




void AC_LADRC::update_eso(float measurement)
{
	//check if the parameter is initialized
	if (!is_zero(_wo) && is_positive(_dt)) {
		// Ensure that integrator can only be reduced if the output is saturated
		float beta_1 = 3 * _wo;
		float beta_2 = 3 * _wo*_wo;
		float beta_3 = _wo * _wo* _wo;

		float e = measurement - _z1 ;  //measurement - observe 

		_z1 += (_z2 + beta_1 * e)*_dt;
		_z2 += (_z3 + beta_2 * e + _b*_u)*_dt;
		_z3 += (beta_3 * e)*_dt;
	}
	else {
		_z1 = 0.0f;
		_z2 = 0.0f;
		_z3 = 0.0f;
	}

	//log

}
//todo: linear td
void AC_LADRC::update_td(float target) {
	float fh=-_r*_r*(_v1-target)-2*_r*_v2;
	_v1+=_v2*_dt;
	_v2+=fh*_dt;
}


//todo: set a button to switch control law
void AC_LADRC::update_linearPD(void){
	float kp=_wc*_wc;
	float kd=2*_wc;
	_u= kp*(_v1 - _z1)+ kd*(_v2 - _z2);
	_u=(_u-_z3)/_b;
}

void AC_LADRC::reset_z3()
{
	_z3 = 0;
}


void AC_LADRC::load_gains()
{
	_wo.load();
	_wc.load();
	_b.load();
	_r.load();
	_kff.load();
}

// save_gains - save gains to eeprom
void AC_LADRC::save_gains()
{
	_wo.save();
	_wc.save();
	_b.save();
	_r.save();
	_kff.save();
}

// Overload the function call operator to permit easy initialisation
void AC_LADRC::operator()(float wo_val, float wc_val, float b_val, float r_val,float ff_val,float dt)
{
	_wo = wo_val;
	_wc = wc_val;
	_b  = b_val;
	_r  = r_val;
	_kff = ff_val;
	_dt = dt;
}





float AC_LADRC::get_LADRC() const
{
	return _u;
}
float AC_LADRC::get_ESO() const
{
	return _z3 / _b;
}

float AC_LADRC::get_ff() {
	_ladrc_info.FF = _v1 * _kff;
	return _kff * _v1;
}