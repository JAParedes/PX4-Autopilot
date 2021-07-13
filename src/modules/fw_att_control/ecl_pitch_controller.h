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
 * @file ecl_pitch_controller.h
 * Definition of a simple orthogonal pitch PID controller.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Paul Riseborough and Andrew Tridgell, 2013,
 *   which in turn is based on initial work of
 *   Jonathan Challinger, 2012.
 */

#ifndef ECL_PITCH_CONTROLLER_H
#define ECL_PITCH_CONTROLLER_H

#define RCAC_PITCH_L_THETA 2 // P-I Control RCAC
#define RCAC_PITCH_L_RBLOCK 2


#include <mathlib/mathlib.h>

#include "ecl_controller.h"

class ECL_PitchController :
	public ECL_Controller
{
public:
	ECL_PitchController() = default;
	~ECL_PitchController() = default;

	float control_attitude(const float dt, const ECL_ControlData &ctl_data) override;
	float control_euler_rate(const float dt, const ECL_ControlData &ctl_data) override;
	float control_bodyrate(const float dt, const ECL_ControlData &ctl_data) override;
	void reset_integrator() override;
	void set_integrator_max(float max) override;

	/* Additional Setters */
	void set_max_rate_pos(float max_rate_pos)
	{
		_max_rate = max_rate_pos;
	}

	void set_max_rate_neg(float max_rate_neg)
	{
		_max_rate_neg = max_rate_neg;
	}

	void set_bodyrate_setpoint(float rate)
	{
		_bodyrate_setpoint = math::constrain(rate, -_max_rate_neg, _max_rate);
	}

	void set_roll_ff(float roll_ff)
	{
		_roll_ff = roll_ff;
	}

	/* Initialize RCAC Variables */
	void init_RCAC_pitch();
	void reset_RCAC_kk();

	/* RCAC Getter Functions - For Publishing */
	int   get_RCAC_pitch_ii() {return _rcac_pitch.getkk();}
	float get_RCAC_pitch_Ru() {return rcac_pitch_Ru;}
	float get_RCAC_pitch_Rz() {return rcac_pitch_Rz;}
	float get_RCAC_pitch_P0() {return rcac_pitch_P0;}
	float get_RCAC_pitch_P11() {return _rcac_pitch.get_rcac_P(0, 0);}
	float get_RCAC_pitch_alpha() {return alpha_PID_pitch;}
	float get_RCAC_pitch_uk() {return _rcac_pitch.get_rcac_uk();}
	bool  get_RCAC_pitch_switch() {return RCAC_pitch_SW;}
	matrix::Vector<float, 2> get_RCAC_theta();
	matrix::Vector<float, 2> get_PX4_theta();

	/* RCAC Setter Functions - For Alteration in QGC */
	void set_RCAC_pitch_Ru(float pitch_Ru_in)
	{
		// if (pitch_Ru_in != rcac_pitch_Ru)
		PX4_INFO("[Pitch RCAC Param Update] Ru: %6.4f", (double)pitch_Ru_in);
		rcac_pitch_Ru = pitch_Ru_in;
	}

	void set_RCAC_pitch_Rz(float pitch_Rz_in)
	{
		// if (rcac_pitch_Rz != pitch_Rz_in)
		PX4_INFO("[Pitch RCAC] Rz Update: %6.4f", (double)pitch_Rz_in);
		rcac_pitch_Rz = pitch_Rz_in;
	}

	void set_RCAC_pitch_P0(float pitch_P0_in)
	{
		// if (rcac_pitch_P0 != pitch_P0_in)
		PX4_INFO("[Pitch RCAC Param Update] P0: %6.4f", (double)pitch_P0_in);
		rcac_pitch_P0 = pitch_P0_in;
	}

	void set_RCAC_pitch_alpha(float pitch_alpha_in)
	{
		// if (alpha_PID_pitch != pitch_alpha_in)
		PX4_INFO("[Pitch RCAC Param Update] Alpha: %6.4f", (double)pitch_alpha_in);
		alpha_PID_pitch = pitch_alpha_in;
	}

	void set_RCAC_pitch_SW(bool pitch_SW_in)
	{
		// if (RCAC_pitch_SW != pitch_SW_in)
		PX4_INFO("[Pitch RCAC Param Update] RCAC Switch: %s", pitch_SW_in ? "true" : "false");
		RCAC_pitch_SW = pitch_SW_in;
	}

protected:
	float _max_rate_neg{0.0f};
	float _roll_ff{0.0f};

private:

	// Jun 24, 2021: New RCAC Variables
	matrix::Matrix<float, RCAC_PITCH_L_RBLOCK, RCAC_PITCH_L_RBLOCK> rcac_pitch_Rblock;

	RCAC<RCAC_PITCH_L_THETA, RCAC_PITCH_L_RBLOCK> _rcac_pitch;
	float z_k_pitch, z_km1_pitch, u_k_pitch, u_km1_pitch;

	bool RCAC_pitch_SW = false;
	bool RCAC_pitch_Rblock_SW = true;

	int rcac_pitch_e_fun = 0;
	float rcac_pitch_P0 = 0.0011f;//
	float rcac_pitch_Rz = 1.0;
	float rcac_pitch_Ru = 1.0;//
	float rcac_pitch_lambda = 1.0;
	float rcac_pitch_N = -1.0;//
	float alpha_PID_pitch = 1.0f;//
};

#endif // ECL_PITCH_CONTROLLER_H
