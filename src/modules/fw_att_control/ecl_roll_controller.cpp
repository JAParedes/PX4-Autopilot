/****************************************************************************
 *
 *   Copyright (c) 2013-2020 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ecl_roll_controller.cpp
 * Implementation of a simple orthogonal roll PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_roll_controller.h"
#include <float.h>
#include <lib/ecl/geo/geo.h>
// #include <mathlib/mathlib.h>

//TODO: Need to do params update based on QGC

float ECL_RollController::control_attitude(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll_setpoint) &&
	      PX4_ISFINITE(ctl_data.roll))) {

		return _rate_setpoint;
	}

	/* Calculate the error */
	float roll_error = ctl_data.roll_setpoint - ctl_data.roll;

	/*  Apply P controller: rate setpoint from current error and time constant */
	_rate_setpoint = roll_error / _tc;

	return _rate_setpoint;
}

float ECL_RollController::control_bodyrate(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.body_x_rate) &&
	      PX4_ISFINITE(ctl_data.body_z_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - ctl_data.body_x_rate;

	/* Initialize RCAC on first iteration and roll switched on. */
	if (RCAC_roll_SW && _rcac_roll.getkk() == 0)
		init_RCAC_roll();

	if (!ctl_data.lock_integrator && _k_i > 0.0f) {

		/* Integral term scales with 1/IAS^2 */
		float id = _rate_error * dt * ctl_data.scaler * ctl_data.scaler;

		/*
		 * anti-windup: do not allow integrator to increase if actuator is at limit
		 */
		if (_last_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		/* add and constrain */
		_integrator = math::constrain(_integrator + id * _k_i, -_integrator_max, _integrator_max);

		/* RCAC lib integrator (No gain with integral) */
		if (RCAC_roll_SW)
			_rcac_roll.update_integral(id, dt);
	}

	/* Apply PI rate controller and store non-limited output */
	/* FF terms scales with 1/TAS and P,I with 1/IAS^2 */
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.scaler +
		       _rate_error * _k_p * ctl_data.scaler * ctl_data.scaler
		       + _integrator;

	// TODO: Import the landed variable so that our implementation is consistent.
	if (RCAC_roll_SW)
	{
		if (_rcac_roll.getkk() == 0) {
			// Initial derivative will be zero.
			init_RCAC_roll();
		}

		matrix::Matrix<float, 1, RCAC_ROLL_L_THETA> Phi_roll;
		Phi_roll(0, 0) = _rate_error;
		Phi_roll(0, 1) = _rcac_roll.get_rcac_integral();
		u_k_roll = _rcac_roll.compute_uk(_rate_error, Phi_roll, _rcac_roll.get_rcac_uk());
	}
	else {
		u_k_roll = 0;
	}

	_last_output = alpha_PID_roll * _last_output + u_k_roll;

	return math::constrain(_last_output, -1.0f, 1.0f);
}

float ECL_RollController::control_euler_rate(const float dt, const ECL_ControlData &ctl_data)
{
	/* Transform setpoint to body angular rates (jacobian) */
	_bodyrate_setpoint = ctl_data.roll_rate_setpoint - sinf(ctl_data.pitch) * ctl_data.yaw_rate_setpoint;

	set_bodyrate_setpoint(_bodyrate_setpoint);

	return control_bodyrate(dt, ctl_data);
}

void ECL_RollController::init_RCAC_roll()
{
	if (RCAC_roll_Rblock_SW)
	{
		rcac_roll_Rblock(0,0) = rcac_roll_Rz;
		rcac_roll_Rblock(1,1) = rcac_roll_Ru;
		_rcac_roll = RCAC<RCAC_ROLL_L_THETA, RCAC_ROLL_L_RBLOCK> (rcac_roll_P0, rcac_roll_lambda, rcac_roll_Rblock, rcac_roll_N, rcac_roll_e_fun, _integrator_max);
	}

	else
	{
		_rcac_roll = RCAC<RCAC_ROLL_L_THETA, RCAC_ROLL_L_RBLOCK> (rcac_roll_P0, rcac_roll_lambda, rcac_roll_N, rcac_roll_e_fun, _integrator_max);
	}
}

matrix::Vector<float, 2> ECL_RollController::get_RCAC_theta()
{
	matrix::Vector<float, 2> RCAC_theta;

	for (size_t i = 0; i < 2; ++i)
	{
		RCAC_theta(i) = _rcac_roll.get_rcac_theta(i);
	}
	return RCAC_theta;
}

matrix::Vector<float, 2> ECL_RollController::get_PX4_theta()
{
	//TODO: Update this function if FF is being used with RCAC Later
	matrix::Vector<float, 2> PX4_theta;
	PX4_theta(0) = _k_p;
	PX4_theta(1) = _k_i;
	return PX4_theta;
}
