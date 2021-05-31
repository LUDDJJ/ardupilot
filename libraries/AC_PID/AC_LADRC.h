#pragma once


/// @file	AC_LADRC.h
/// @brief	Generic LADRC algorithm, with EEPROM-backed storage of constants.
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <AP_Logger/AP_Logger.h>



/// @class	AC_LADRC
/// @brief	Copter LADRC control class
class AC_LADRC {
public:

	// Constructor for LADRC
	AC_LADRC(float initial_wc, float initial_wo, float initial_b, float initial_r, float initial_ff, float dt);

	// set_dt - set time step in seconds
	void set_dt(float dt);

	//  update_all - set target and measured inputs to PID controller and calculate outputs
	//  target and error are filtered
	//  the derivative is then calculated and filtered
	//  the integral is then updated based on the setting of the limit flag
	float update_all(float target, float measurement, bool limit = false);

	void update_td(float target);
	
	//  update_ESO - update the integral
	void update_eso(float measurement);

	//  update control law
	void update_linearPD(void);

	// get results from LADRC controller
	float get_LADRC() const;
	float get_ESO() const;
	float get_ff();

	// reset_z3 - reset the integrator   
	void reset_z3();


	// reset_filter - input filter will be reset to the next value provided to set_input()
	void reset_filter() {
		_flags._reset_filter = true;
	}

	// load gain from eeprom
	void load_gains();

	// save gain to eeprom
	void save_gains();

	/// operator function call for easy initialisation
	void operator()(float wo_val, float wc_val, float b_val, float r_val,float ff_val,float dt);

	// get accessors  访问参数接口
	AP_Float &wo() { return _wo; }
	AP_Float &wc() { return _wc; }
	AP_Float &b()  { return _b; }
	AP_Float &r()  { return _r; }

	// set accessors
	void wo(const float v) { _wo.set(v); }
	void wc(const float v) { _wc.set(v); }
	void b(const float v) { _b.set(v); }
	void r(const float v) { _r.set(v); }

	const AP_Logger::LADRC_Info& get_ladrc_info(void) const { return _ladrc_info; }   
	// parameter var table 
	static const struct AP_Param::GroupInfo var_info[];

protected:

	// parameters
	AP_Float _wo;   //LADRC Observer bandwidth
	AP_Float _wc;   //LADRC Control bandwidth
	AP_Float _b;    //LADRC Observer input coefficient
	AP_Float _r;    //TD  coefficient
	AP_Float _kff;  //Feedforward  control

	// flags
	struct ac_LADRC_flags {
		bool _reset_filter : 1; // true when input filter should be reset during next call to set_input
	} _flags;

	// internal variables
	// ESO
	float _z1;
	float _z2;
	float _z3;

	// control variable
	float _u;

	// input variable
	float _v1;
	float _v2;

	// internal variables
	float _dt;                // timestep in seconds

	//AP_Logger::PID_Info _pid_info;   replace ADRC_info
	AP_Logger::LADRC_Info _ladrc_info;
};
