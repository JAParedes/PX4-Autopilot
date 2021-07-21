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
 * @file ecl_pitch_controller.cpp
 * Implementation of a simple orthogonal pitch PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_pitch_controller.h"
#include <float.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>

ECL_PitchController::ECL_PitchController() : ECL_Controller()
{
	_rcac_att_public_io = RCAC_Public_IO<RCAC_ATT_L_THETA, RCAC_ATT_L_RBLOCK>(&_rcac_att);
	_rcac_att_params_io = RCACParams_IO(&_rcac_att_params);
}

float ECL_PitchController::control_attitude(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.pitch_setpoint) &&
	      PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.airspeed))) {

		return _rate_setpoint;
	}

	/* Calculate the error */
	float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;

	/*  Apply P controller: rate setpoint from current error and time constant */
	_rate_setpoint =  pitch_error / _tc;

	matrix::Matrix<float, 1, RCAC_ATT_L_THETA> Phi_att;
	Phi_att(0, 0) = pitch_error;
	u_k_att = _rcac_att.compute_uk(_rate_error, Phi_att, _rcac_att.get_rcac_uk());

	_rate_setpoint = _rcac_att_params.tuneParams.alpha_PID * _rate_setpoint + u_k_att;

	return _rate_setpoint;
}

float ECL_PitchController::control_bodyrate(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.body_y_rate) &&
	      PX4_ISFINITE(ctl_data.body_z_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - ctl_data.body_y_rate;

	//TODO: Possibly not need this if RCAC_EN is modified directly instead of changing kk
	/* Initialize RCAC on first iteration and rcac switched on. */
	if (_rcac_rate_params.tuneParams.RCAC_EN && _rcac_rate.getkk() == 0)
		init_RCAC_pitch();

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
		if (_rcac_rate_params.tuneParams.RCAC_EN)
			_rcac_rate.update_integral(id, dt);
	}

	/* Apply PI rate controller and store non-limited output */
	/* FF terms scales with 1/TAS and P,I with 1/IAS^2 */
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.scaler +
		       _rate_error * _k_p * ctl_data.scaler * ctl_data.scaler
		       + _integrator;

	matrix::Matrix<float, 1, RCAC_PITCH_L_THETA> Phi_rate;
	Phi_rate(0, 0) = _rate_error;
	Phi_rate(0, 1) = _rcac_rate.get_rcac_integral();
	u_k_rate = _rcac_rate.compute_uk(_rate_error, Phi_rate, _rcac_rate.get_rcac_uk());

	_last_output = _last_output * _rcac_rate_params.tuneParams.alpha_PID +  u_k_rate;

	return math::constrain(_last_output, -1.0f, 1.0f);
}

float ECL_PitchController::control_euler_rate(const float dt, const ECL_ControlData &ctl_data)
{
	/* Transform setpoint to body angular rates (jacobian) */
	_bodyrate_setpoint = cosf(ctl_data.roll) * _rate_setpoint +
			     cosf(ctl_data.pitch) * sinf(ctl_data.roll) * ctl_data.yaw_rate_setpoint;

	set_bodyrate_setpoint(_bodyrate_setpoint);

	return control_bodyrate(dt, ctl_data);
}

matrix::Vector<float, 2> ECL_PitchController::get_PX4_theta()
{
	//TODO: Update this function if FF is being used with RCAC Later
	matrix::Vector<float, 2> PX4_theta;
	PX4_theta(0) = _k_p;
	PX4_theta(1) = _k_i;
	return PX4_theta;
}

void ECL_PitchController::reset_integrator()
{
	_integrator = 0.0f;
	_rcac_rate.reset_integral();
}

//TODO: Double check if this function is necessary
void ECL_PitchController::set_integrator_max(float max)
{
	_integrator_max = max;
	_rcac_rate.set_lim_int(max);
}

// void ECL_PitchController::reset_RCAC_kk()
// {
// 	_rcac_rate.reset_kk();
// }
