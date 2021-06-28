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
 * @file ecl_roll_controller.h
 * Definition of a simple orthogonal roll PID controller.
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

#ifndef ECL_ROLL_CONTROLLER_H
#define ECL_ROLL_CONTROLLER_H

#define RCAC_ROLL_L_THETA 2 // P-D Control RCAC
#define RCAC_ROLL_L_RBLOCK 2

#include "ecl_controller.h"
#include <mathlib/mathlib.h>

class ECL_RollController :
	public ECL_Controller
{
public:
	ECL_RollController() = default;
	~ECL_RollController() = default;

	float control_attitude(const float dt, const ECL_ControlData &ctl_data) override;
	float control_euler_rate(const float dt, const ECL_ControlData &ctl_data) override;
	float control_bodyrate(const float dt, const ECL_ControlData &ctl_data) override;

	/* Initialize RCAC Variables */
	void init_RCAC_roll();

	/* RCAC Getter Functions - For Publishing */
	int   get_RCAC_roll_ii() {return _rcac_roll.getkk();}
	float get_RCAC_roll_Ru() {return rcac_roll_Ru;}
	float get_RCAC_roll_Rz() {return rcac_roll_Rz;}
	float get_RCAC_roll_P0() {return rcac_roll_P0;}
	float get_RCAC_roll_P11() {return _rcac_roll.get_rcac_P(0, 0);}
	float get_RCAC_roll_alpha() {return alpha_PID_roll;}
	bool  get_RCAC_roll_switch() {return RCAC_roll_SW;}


	/* RCAC Setter Functions - For Alteration in QGC */
	void set_RCAC_roll_Ru(float roll_Ru_in) {rcac_roll_Ru = roll_Ru_in;}
	void set_RCAC_roll_Rz(float roll_Rz_in) {rcac_roll_Rz = roll_Rz_in;}
	void set_RCAC_roll_P0(float roll_P0_in) {rcac_roll_P0 = roll_P0_in;}
	void set_RCAC_roll_alpha(float roll_alpha_in) {alpha_PID_roll = roll_alpha_in;}
	void set_RCAC_roll_SW(bool roll_SW_in) {RCAC_roll_SW = roll_SW_in;}

private:

	// Jun 24, 2021: New RCAC Variables
	matrix::Matrix<float, RCAC_ROLL_L_RBLOCK, RCAC_ROLL_L_RBLOCK> rcac_roll_Rblock;

	RCAC<RCAC_ROLL_L_THETA, RCAC_ROLL_L_RBLOCK> _rcac_roll;
	float z_k_roll, z_km1_roll, u_k_roll, u_km1_roll;

	bool RCAC_roll_SW = false;
	bool RCAC_roll_Rblock_SW = true;

	int rcac_roll_e_fun = 0;
	float rcac_roll_P0 = 0.0011f;
	float rcac_roll_Rz = 1.0;
	float rcac_roll_Ru = 1.0;
	float rcac_roll_lambda = 1.0;
	float rcac_roll_N = -1.0;
	float alpha_PID_roll = 1.0f;
};

#endif // ECL_ROLL_CONTROLLER_H
