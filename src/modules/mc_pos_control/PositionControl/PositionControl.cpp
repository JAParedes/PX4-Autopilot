/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <px4_platform_common/defines.h>
#include <ecl/geo/geo.h>

using namespace matrix;

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	_vel_int(2) += (hover_thrust_new - _hover_thrust) * (CONSTANTS_ONE_G / hover_thrust_new);
	setHoverThrust(hover_thrust_new);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(vehicle_local_position_setpoint_s setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

void PositionControl::setConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.
	if (!PX4_ISFINITE(constraints.tilt) || (constraints.tilt > _lim_tilt)) {
		_constraints.tilt = _lim_tilt;
	}

	if (!PX4_ISFINITE(constraints.speed_up) || (constraints.speed_up > _lim_vel_up)) {
		_constraints.speed_up = _lim_vel_up;
	}

	if (!PX4_ISFINITE(constraints.speed_down) || (constraints.speed_down > _lim_vel_down)) {
		_constraints.speed_down = _lim_vel_down;
	}

	// ignore _constraints.speed_xy TODO: remove it completely as soon as no task uses it anymore to avoid confusion
}

bool PositionControl::update(const float dt)
{
	// x and y input setpoints always have to come in pairs
	const bool valid = (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)))
			   && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)))
			   && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	_positionControl();
	_velocityControl(dt);

	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

	return valid && _updateSuccessful();
}

void PositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// PX4_INFO("vel_sp_ff = \t%8.6f \t%8.6f \t%8.6f", (double)vel_sp_position(0), (double)vel_sp_position(1), (double)vel_sp_position(2));

	for (int i = 0; i <=2; i++)
	{
		if (isnan(_pos_sp(i)) || isnan(_vel_sp(i)))
		{
			islanded = true;
			since_takeoff = 0;
			break;
		}
		else
		{
			islanded = false;
		}
	}

	z_k_r = _pos_sp - _pos;
	u_k_r.setZero();

	if ((RCAC_Pr_ON) && (!islanded))
	{
		if (since_takeoff == 0)
		{
			resetRCAC();
			since_takeoff++;
		}

		for (int i = 0; i <= 2; i++)
		{
			u_k_r(i) = _rcac_r(0,i).compute_uk(z_k_r(i), 0, 0, u_km1_r(i));
		}
		u_km1_r = u_k_r;
	}

	vel_sp_position = alpha_PID_pos*vel_sp_position + u_k_r;
	// PX4_INFO("vel_sp_ff = \t%8.6f \t%8.6f \t%8.6f", (double)vel_sp_position(0), (double)vel_sp_position(1), (double)vel_sp_position(2));

	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);
}

void PositionControl::_velocityControl(const float dt)
{
	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	z_k_v = _vel_sp - _vel;

	if ((RCAC_Pv_ON) && (!islanded))
	{
		for (int i = 0; i <= 2; i++)
		{
			u_k_v(i) = _rcac_v(0,i).compute_uk(z_k_v(i), _vel_int(i), _vel_dot(i), u_km1_v(i));
		}
		u_km1_v = u_k_v;
	}

	acc_sp_velocity = alpha_PID_vel*acc_sp_velocity + u_k_v;

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
		vel_error(2) = 0.f;
	}

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -_lim_thr_max);

	// Get allowed horizontal thrust after prioritizing vertical control
	const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
	const float thrust_z_squared = _thr_sp(2) * _thr_sp(2);
	const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();

	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);
	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;

	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
}

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _constraints.tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool PositionControl::_updateSuccessful()
{
	bool valid = true;

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	// There has to be a valid output accleration and thrust setpoint otherwise there was no
	// setpoint-state pair for each axis that can get controlled
	valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
	valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));
	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}

const matrix::Vector3f PositionControl::get_RCAC_pos_z()
{
	matrix::Vector3f RCAC_z{};

	for (int i = 0; i <= 2; i++) {
		RCAC_z(i) = _rcac_r(0,i).get_rcac_zk();
	}

	return RCAC_z;
}

const matrix::Vector3f PositionControl::get_RCAC_pos_u()
{
	matrix::Vector3f RCAC_u{};

	for (int i = 0; i <= 2; i++) {
		RCAC_u(i) = _rcac_r(0,i).get_rcac_uk();
	}

	return RCAC_u;
}

const matrix::Vector3f PositionControl::get_RCAC_pos_theta()
{
	matrix::Vector3f RCAC_theta{};

	for (int i = 0; i <= 2; i++) {
		RCAC_theta(i) = _rcac_r(0,i).get_rcac_theta(0);

		// for (int j = 0; j <= 2; j++){
		// 	PX4_INFO("RCAC_theta(%d):\t%8.6f", i, (double)_rcac_r(0,i).get_rcac_theta(j));
		// }
	}

	return RCAC_theta;
}

const matrix::Vector3f PositionControl::get_PX4_pos_theta()
{
	matrix::Vector3f PX4_theta{};
	PX4_theta(0) = _gain_pos_p(0);
	PX4_theta(1) = _gain_pos_p(1);
	PX4_theta(2) = _gain_pos_p(2);
	return PX4_theta;
}

const matrix::Matrix<float, 9,1> PositionControl::get_PX4_ol_theta()
{
	matrix::Matrix<float, 9,1> PX4_theta{};
	PX4_theta(0,0) = _gain_vel_p(0);
	PX4_theta(1,0) = _gain_vel_i(0);
	PX4_theta(2,0) = _gain_vel_d(0);

	PX4_theta(3,0) = _gain_vel_p(1);
	PX4_theta(4,0) = _gain_vel_i(1);
	PX4_theta(5,0) = _gain_vel_d(1);

	PX4_theta(6,0) = _gain_vel_p(2);
	PX4_theta(7,0) = _gain_vel_i(2);
	PX4_theta(8,0) = _gain_vel_d(2);

	return PX4_theta;
}

const matrix::Vector3f PositionControl::get_RCAC_vel_z()
{
	matrix::Vector3f RCAC_z{};

	for (int i = 0; i <= 2; i++) {
		RCAC_z(i) = _rcac_v(0,i).get_rcac_zk();
	}

	return RCAC_z;
}

const matrix::Vector3f PositionControl::get_RCAC_vel_u()
{
	matrix::Vector3f RCAC_u{};

	for (int i = 0; i <= 2; i++) {
		RCAC_u(i) = _rcac_v(0,i).get_rcac_uk();
	}

	return RCAC_u;
}

const matrix::Matrix<float, 9,1> PositionControl::get_RCAC_vel_theta()
{
	matrix::Matrix<float, 9,1> RCAC_vel_theta{};
	RCAC_vel_theta.setZero();

	for (int i = 0; i <= 2; i++)
	{
		RCAC_vel_theta(i,0) = _rcac_v(0,0).get_rcac_theta(i);
		RCAC_vel_theta(i+3,0) = _rcac_v(0,1).get_rcac_theta(i);
		RCAC_vel_theta(i+6,0) = _rcac_v(0,2).get_rcac_theta(i);
	}

	return RCAC_vel_theta;
}

void PositionControl::set_RCAC_pos_switch(float switch_RCAC)
{
	if (switch_RCAC<0.0f) {
		RCAC_Pr_ON = 0;
	}
}

void PositionControl::set_RCAC_vel_switch(float switch_RCAC)
{
	if (switch_RCAC<0.0f) {
		RCAC_Pv_ON = 0;
	}
}

void PositionControl::set_PID_pv_factor(float PID_factor, float pos_alpha, float vel_alpha)
{
	if (PID_factor<0.0f) {
		alpha_PID_pos = pos_alpha;
		alpha_PID_vel = vel_alpha;
	}
}

void PositionControl::resetRCAC()
{
	for (int i = 0; i <= 2; i++) {
		_rcac_r(0,i) = RCAC(p0_r);
		_rcac_v(0,i) = RCAC(p0_v);
	}
}
