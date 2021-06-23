/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * moication, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

#define RCAC_POS_L_THETA 1
#define RCAC_POS_L_RBLOCK 2

#define RCAC_VEL_L_THETA 3
#define RCAC_VEL_L_RBLOCK 2
// #include "vector"

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */
class PositionControl
{
public:

	PositionControl() = default;
	~PositionControl() = default;

	/**
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }

	/**
	 * Set the velocity control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_up upwards velocity limit
	 * @param vel_down downwards velocity limit
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set the maximum tilt angle in radians the output attitude is allowed to have
	 * @param tilt angle in radians from level orientation
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * Set the normalized hover thrust
	 * @param thrust [0,1] with which the vehicle hovers not acelerating down or up with level orientation
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = hover_thrust; }

	/**
	 * Update the hover thrust without immediately affecting the output
	 * by adjusting the integrator. This prevents propagating the dynamics
	 * of the hover thrust signal directly to the output of the controller.
	 */
	void updateHoverThrust(const float hover_thrust_new);

	/**
	 * Pass the current vehicle state to the controller
	 * @param PositionControlStates structure
	 */
	void setState(const PositionControlStates &states);

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint a vehicle_local_position_setpoint_s structure
	 */
	void setInputSetpoint(vehicle_local_position_setpoint_s setpoint);

	/**
	 * Pass constraints that are stricter than the global limits
	 * Note: NAN value means no constraint, take maximum limit of controller.
	 * @param constraints a PositionControl structure with supported constraints
	 */
	void setConstraints(const vehicle_constraints_s &constraints);

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt, const bool landed);

	/**
	 * Set the integral term in xy to 0.
	 * @see _vel_int
	 */
	void resetIntegral()
	{
		_vel_int.setZero();
		for (int i = 0; i < 3; ++i)
			_rcac_vel(0, i).reset_integral();
	}

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * @param attitude_setpoint reference to struct to fill up
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

		/**
	 * 	Get the
	 * 	@see z_k_Pr_R
	 * 	@return The z variable used by RCAC in the P controller
	 */
	const matrix::Vector3f get_RCAC_pos_z();

	/**
	 * 	Get the
	 * 	@see u_k_Pr_R
	 * 	@return The u variable computed by RCAC in the P controller
	 */
	matrix::Vector3f get_RCAC_pos_u();

	/**
	 * 	Get the
	 * 	@see theta_k_Pr_R
	 * 	@return The theta variable computed by RCAC in the P controller
	 */
	const matrix::Vector3f get_RCAC_pos_theta();

	/**
	 * 	Get the
	 * 	@see Pos P gains
	 * 	@return The P gains
	 */
	const matrix::Vector3f get_PX4_pos_theta();

	/**
	 * 	Get the
	 * 	@see PID gains
	 * 	@return PX4's PID gains in the outer loop
	 */
	const matrix::Matrix<float, 9,1> get_PX4_ol_theta();

	/**
	 * 	Get the
	 * 	@see z_k_Pv_R
	 * 	@return The z variable used by RCAC in the PID velocity controller
	 */
	const matrix::Vector3f get_RCAC_vel_z();

	/**
	 * 	Get the
	 * 	@see u_k_Pr_R
	 * 	@return The u variable computed by RCAC in the P controller
	 */
	const matrix::Vector3f get_RCAC_vel_u();

	/**
	 * 	Get the
	 * 	@see theta_k_Pr_R
	 * 	@return The theta variable computed by RCAC in the P controller
	 */
	const matrix::Matrix<float, 9,1> get_RCAC_vel_theta();

	/**
	 * 	Get the
	 * 	@see ii
	 * 	@return Iteration step of the RCAC position controller
	 */
	int get_RCAC_pos_ii() { return _rcac_pos(0,0).getkk(); }

	/**
	 * 	Get the
	 * 	@see ii
	 * 	@return Iteration step of the RCAC velocity controller
	 */
	int get_RCAC_vel_ii() { return _rcac_vel(0,0).getkk(); }

	/**
	 * 	Set the RCAC position switch.
	 * 	@see _thr_int
	 */
	void set_RCAC_pos_switch(float switch_RCAC);

	/**
	 * 	Set the RCAC velocity switch.
	 * 	@see _thr_int
	 */
	void set_RCAC_vel_switch(float switch_RCAC);

	// /**
	//  * 	Set the RCAC position Rblock switch.
	//  * 	@see _thr_int
	//  */
	// void set_RCAC_pos_Rblock_switch(int switch_Rblock);

	// /**
	//  * 	Set the RCAC velocity Rblock switch.
	//  * 	@see _thr_int
	//  */
	// void set_RCAC_vel_Rblock_switch(int switch_Rblock);

	// /**
	//  * 	Set the RCAC position error function.
	//  * 	@see _thr_int
	//  */
	// void set_RCAC_pos_err_fun(int fun_err);

	// /**
	//  * 	Set the RCAC velocity error function.
	//  * 	@see _thr_int
	//  */
	// void set_RCAC_vel_err_fun(int fun_err);

	/**
	 * 	Set the PID scaling factor.
	 * 	@see _thr_int
	 */
	void set_PID_pv_factor(float PID_factor, float pos_alpha, float vel_alpha);
	/**
	 * 	Get the
	 * 	@see rcac_pos_ON
	 * 	@return Get RCAC pos controller switch
	 */
	const bool &get_RCAC_pos_switch() {return rcac_pos_ON;}

	/**
	 * 	Get the
	 * 	@see rcac_pos_ON
	 * 	@return Get RCAC vel controller switch
	 */
	const bool &get_RCAC_vel_switch() {return rcac_vel_ON;}

	/**
	 * 	Get the
	 * 	@see alpha_PID_pos
	 * 	@return Get gain that multiplies the position PID gains
	 */
	const float &get_pid_pos_alpha() {return alpha_PID_pos;}

	/**
	 * 	Get the
	 * 	@see alpha_PID_vel
	 * 	@return Get gain that multiplies the velocity PID gains
	 */
	const float &get_pid_vel_alpha() {return alpha_PID_vel;}

	/**
	 * 	Get the
	 * 	@see P_Pr_R
	 * 	@return RCAC P(1,1) of the Position controller
	 */
	float get_RCAC_P11_Pos() { return _rcac_pos(0,0).get_rcac_P(0,0); }

	/**
	 * 	Get the
	 * 	@see P_vel_x
	 * 	@return RCAC P(1,1) of the Velcity x controller
	 */
	float get_RCAC_P11_Velx() { return _rcac_vel(0,0).get_rcac_P(0,0); }

	/**
	 * 	Get the
	 *
	 * 	@return RCAC Ru of the Position controller
	 */
	float get_RCAC_Ru_Pos() { return _rcac_pos(0,0).get_rcac_Ru(); }

	/**
	 * 	Get the
	 *
	 * 	@return RCAC Ru of the Velcity x controller
	 */
	float get_RCAC_Ru_Vel() { return _rcac_vel(0,0).get_rcac_Ru(); }

	/**
	 * 	Set P0 from value specified in mc_pos_control_params.c
	 *
	 */
	void set_RCAC_pos_vel_P0(float pos_P0, float vel_P0)
	{
		rcac_pos_P0 = pos_P0;
		rcac_vel_P0 = vel_P0;
	}

	/**
	 * 	Set Ru from value specified in mc_pos_control_params.c
	 *
	 */
	void set_RCAC_pos_vel_Ru(float pos_Ru, float vel_Ru)
	{
		rcac_pos_Ru = pos_Ru;
		rcac_vel_Ru = vel_Ru;
	}


	/**
	 * 	Reset RCAC variables
	 * 	@see _thr_int
	 */
	void init_RCAC_pos();
	void init_RCAC_vel();

private:
	bool _updateSuccessful();

	void _positionControl(const bool landed); ///< Position proportional control
	void _velocityControl(const float dt, const bool landed); ///< Velocity PID control
	void _accelerationControl(); ///< Acceleration setpoint processing

	// Gains
	matrix::Vector3f _gain_pos_p; ///< Position control proportional gain
	matrix::Vector3f _gain_vel_p; ///< Velocity control proportional gain
	matrix::Vector3f _gain_vel_i; ///< Velocity control integral gain
	matrix::Vector3f _gain_vel_d; ///< Velocity control derivative gain

	// Limits
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

	float _hover_thrust{}; ///< Thrust [0,1] with which the vehicle hovers not accelerating down or up with level orientation

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	float _yaw{}; /**< current heading */

	vehicle_constraints_s _constraints{}; /**< variable constraints */

	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _acc_sp; /**< desired acceleration */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */

	// RCAC -- Position Controller
	matrix::Matrix<RCAC<RCAC_POS_L_THETA, RCAC_POS_L_RBLOCK>, 1, 3> _rcac_pos;
	matrix::Matrix<float, 1, RCAC_POS_L_THETA> Phi_pos;
	matrix::Matrix<float, RCAC_POS_L_RBLOCK, RCAC_POS_L_RBLOCK> rcac_pos_Rblock;
	matrix::Vector3f z_k_pos, u_k_pos, u_km1_pos;
	bool rcac_pos_ON = 1;
	int rcac_pos_Rblock_ON = 1;
	int rcac_pos_e_fun = 0;
	float rcac_pos_P0 = 0.005f;
	float rcac_pos_Rz = 1.0;
	float rcac_pos_Ru = 0.01;
	float rcac_pos_lambda = 1.0;
	float rcac_pos_N = -10;
	float alpha_PID_pos = 1.0f;

	// RCAC -- Velocity Controller
	matrix::Matrix<RCAC<RCAC_VEL_L_THETA, RCAC_VEL_L_RBLOCK>, 1, 3> _rcac_vel;
	matrix::Matrix<float, 1, RCAC_VEL_L_THETA> Phi_vel;
	matrix::Matrix<float, RCAC_VEL_L_RBLOCK, RCAC_VEL_L_RBLOCK> rcac_vel_Rblock;
	matrix::Vector3f z_k_vel, u_k_vel, u_km1_vel, Pv_intg;
	bool rcac_vel_ON = 1;
	int rcac_vel_Rblock_ON = 1;
	int rcac_vel_e_fun = 0;
	float rcac_vel_P0 = 0.001f;
	float rcac_vel_Rz = 1.0;
	float rcac_vel_Ru = 1.0;
	float rcac_vel_lambda = 1.0;
	float rcac_vel_N = -10.0;
	float alpha_PID_vel = 1.0f;
};
