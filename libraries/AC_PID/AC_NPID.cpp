/// @file	AC_NPID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_NPID.h"

const AP_Param::GroupInfo AC_NPID::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P", 0, AC_NPID, _kp, 0),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I", 1, AC_NPID, _ki, 0),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D", 2, AC_NPID, _kd, 0),

    // 3 was for uint16 IMAX

    // @Param: FF
    // @DisplayName: FF FeedForward Gain
    // @Description: FF Gain which produces an output value that is proportional to the demanded input
    AP_GROUPINFO("FF", 4, AC_NPID, _kff, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 5, AC_NPID, _kimax, 0),

    // Fal D
    //AP_GROUPINFO("alphaP", 6, AC_NPID, _alpha_P, 0),

    // Fal P
    //AP_GROUPINFO("alphaD", 7, AC_NPID, _alpha_D, 0),

    // line range
    //AP_GROUPINFO("delta", 8, AC_NPID, _delta, 0),

    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTT", 9, AC_NPID, _filt_T_hz, AC_PID_TFILT_HZ_DEFAULT),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTE", 10, AC_NPID, _filt_E_hz, AC_PID_EFILT_HZ_DEFAULT),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTD", 11, AC_NPID, _filt_D_hz, AC_PID_DFILT_HZ_DEFAULT),

    // Fal D
    AP_GROUPINFO("aP", 12, AC_NPID, _alpha_P, 0),

    // Fal P
    AP_GROUPINFO("aD", 13, AC_NPID, _alpha_D, 0),

    // line range
    AP_GROUPINFO("dlta", 14, AC_NPID, _delta, 0),

    AP_GROUPEND
};

// Constructor
AC_NPID::AC_NPID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax,float initial_alpha_P,float initial_alpha_D,float initial_delta,float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz, float dt) :
    _dt(dt),
    _integrator(0.0f),
    _error(0.0f),
    _derivative(0.0f)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_p;
    _ki = initial_i;
    _kd = initial_d;
    _kff = initial_ff;
    _kimax = fabsf(initial_imax);
    _alpha_P =initial_alpha_P;
    _alpha_D =initial_alpha_D;
    _delta =initial_delta;
    filt_T_hz(initial_filt_T_hz);
    filt_E_hz(initial_filt_E_hz);
    filt_D_hz(initial_filt_D_hz);

    // reset input filter to first value received
    _flags._reset_filter = true;

    memset(&_pid_info, 0, sizeof(_pid_info));
}

// set_dt - set time step in seconds
void AC_NPID::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
}

// filt_T_hz - set target filter hz
void AC_NPID::filt_T_hz(float hz)
{
    _filt_T_hz.set(fabsf(hz));
}

// filt_E_hz - set error filter hz
void AC_NPID::filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// filt_D_hz - set derivative filter hz
void AC_NPID::filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
float AC_NPID::update_all(float target, float measurement, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _target += get_filt_T_alpha() * (target - _target);
        _error += get_filt_E_alpha() * ((_target - measurement) - _error);

        // calculate and filter derivative
        if (_dt > 0.0f) {
            float derivative = (_error - error_last) / _dt;
            _derivative += get_filt_D_alpha() * (derivative - _derivative);

            _pid_info.rawD = (derivative * _kd); //Add ONE variables
        }
    }

    // update I term
    update_i(limit);

    float P_out = (adrc_fal(_error,_alpha_P,_delta) * _kp);
    float D_out = (adrc_fal(_derivative,_alpha_D,_delta)* _kd);

    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.rawError = _target -measurement;   //Add two variables
    _pid_info.rawTarget = target;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    return P_out + _integrator + D_out;
}

//  update_error - set error input to PID controller and calculate outputs
//  target is set to zero and error is set and filtered
//  the derivative then is calculated and filtered
//  the integral is then updated based on the setting of the limit flag
//  Target and Measured must be set manually for logging purposes.
// todo: remove function when it is no longer used.
float AC_NPID::update_error(float error, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(error)) {
        return 0.0f;
    }

    _target = 0.0f;

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _error = error;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _error += get_filt_E_alpha() * (error - _error);

        // calculate and filter derivative
        if (_dt > 0.0f) {
            float derivative = (_error - error_last) / _dt;
            _derivative += get_filt_D_alpha() * (derivative - _derivative);
        }
    }

    // update I term
    update_i(limit);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);

    _pid_info.target = 0.0f;
    _pid_info.actual = 0.0f;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    return P_out + _integrator + D_out;
}

//  update_i - update the integral
//  If the limit flag is set the integral is only allowed to shrink
void AC_NPID::update_i(bool limit)
{
    if (!is_zero(_ki) && is_positive(_dt)) {
        // Ensure that integrator can only be reduced if the output is saturated
        if (!limit || ((is_positive(_integrator) && is_negative(_error)) || (is_negative(_integrator) && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * _dt;
            _integrator = constrain_float(_integrator, -_kimax, _kimax);
        }
    } else {
        _integrator = 0.0f;
    }
    _pid_info.I = _integrator;
}

float AC_NPID::get_p() const
{
    return _error * _kp;
}

float AC_NPID::get_i() const
{
    return _integrator;
}

float AC_NPID::get_d() const
{
    return _kd * _derivative;
}

float AC_NPID::get_ff()
{
    _pid_info.FF = _target * _kff;
    return _target * _kff;
}

// todo: remove function when it is no longer used.
float AC_NPID::get_ff(float target)
{
    float FF_out = (target * _kff);
    _pid_info.FF = FF_out;
    return FF_out;
}

void AC_NPID::reset_I()
{
    _integrator = 0;
}

void AC_NPID::reset_I_smoothly()
{
    float reset_time = AC_PID_RESET_TC * 3.0f;
    uint64_t now = AP_HAL::micros64();

    if ((now - _reset_last_update) > 5e5 ) {
        _reset_counter = 0;
    }
    if ((float)_reset_counter < (reset_time/_dt)) {
        _integrator = _integrator - (_dt / (_dt + AC_PID_RESET_TC)) * _integrator;
        _reset_counter++;
    } else {
        _integrator = 0;
    }
    _reset_last_update = now;
}

void AC_NPID::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _kff.load();
    _kimax.load();
    _kimax = fabsf(_kimax);
    _alpha_P.load();
    _alpha_D.load();
    _delta.load();
    _filt_T_hz.load();
    _filt_E_hz.load();
    _filt_D_hz.load();
}

// save_gains - save gains to eeprom
void AC_NPID::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _kff.save();
    _kimax.save();
    _alpha_P.save();
    _alpha_D.save();
    _delta.save();
    _filt_T_hz.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

/// Overload the function call operator to permit easy initialisation
void AC_NPID::operator()(float p_val, float i_val, float d_val, float ff_val, float imax_val,float alpha_P_val,float alpha_D_val,float delta_val,float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz, float dt)
{
    _kp = p_val;
    _ki = i_val;
    _kd = d_val;
    _kff = ff_val;
    _kimax = fabsf(imax_val);
    _alpha_P = alpha_P_val;
    _alpha_D = alpha_D_val;
    _delta = delta_val;
    _filt_T_hz = input_filt_T_hz;
    _filt_E_hz = input_filt_E_hz;
    _filt_D_hz = input_filt_D_hz;
    _dt = dt;
}

// get_filt_T_alpha - get the target filter alpha
float AC_NPID::get_filt_T_alpha() const
{
    return get_filt_alpha(_filt_T_hz);
}

// get_filt_E_alpha - get the error filter alpha
float AC_NPID::get_filt_E_alpha() const
{
    return get_filt_alpha(_filt_E_hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float AC_NPID::get_filt_D_alpha() const
{
    return get_filt_alpha(_filt_D_hz);
}

// get_filt_alpha - calculate a filter alpha
float AC_NPID::get_filt_alpha(float filt_hz) const
{
    if (is_zero(filt_hz)) {
        return 1.0f;
    }

    // calculate alpha
    float rc = 1 / (M_2PI * filt_hz);
    return _dt / (_dt + rc);
}

void AC_NPID::set_integrator(float target, float measurement, float i)
{
    set_integrator(target - measurement, i);
}

void AC_NPID::set_integrator(float error, float i)
{
    _integrator = constrain_float(i - error * _kp, -_kimax, _kimax);
    _pid_info.I = _integrator;
}

void AC_NPID::set_integrator(float i)
{
    _integrator = constrain_float(i, -_kimax, _kimax);
    _pid_info.I = _integrator;
}



float AC_NPID::adrc_fal(float e, float alpha, float delta)
{
	if(fabsf(e) <= delta){
		return e / (powf(delta, 1.0f-alpha));
	}else{
		return powf(fabsf(e), alpha) * adrc_sign(e);
	}
}


float AC_NPID::adrc_sign(float val)
{
	if(val >= 0.0f)
		return 1.0f;
	else
		return -1.0f;
}


