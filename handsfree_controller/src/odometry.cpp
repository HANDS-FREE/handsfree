/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 * Author: Luke Liao
 */

#include <odometry.h>

#include <boost/bind.hpp>

namespace omni_triangle_controller
{
namespace bacc = boost::accumulators;

Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0)
, x_(0.0)
, y_(0.0)
, heading_(0.0)
, linear_x_(0.0)
, linear_y_(0.0)
, angular_(0.0)
, wheel_center_(0.0)
, wheel_radius_(0.0)
, wheel1_pos_(0.0)
, wheel2_pos_(0.0)
, wheel3_pos_(0.0)
, velocity_rolling_window_size_(velocity_rolling_window_size)
, linear_x_acc_(RollingWindow::window_size = velocity_rolling_window_size)
, linear_y_acc_(RollingWindow::window_size = velocity_rolling_window_size)
, angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
, integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2, _3))
{
}

void Odometry::init(const ros::Time& time)
{
	// Reset accumulators:
	linear_x_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
	linear_y_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
	angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);

	// Reset timestamp:
	timestamp_ = time;
}

bool Odometry::update(double wheel1_vel, double wheel2_vel, double wheel3_vel, const ros::Time &time)
{
	/// Get current wheel joint positions:
	//because of the matrix the value's squence is a little different than us
	const double v1 = wheel2_vel * wheel_radius_;
	const double v2 = wheel3_vel * wheel_radius_;
	const double v3 = wheel1_vel * wheel_radius_;
	const double L = wheel_center_;
	const double th = heading_;
	const double psi = 2 * 3.1415926 / 3;

	/// Update old position with current:
	wheel1_pos_ += v1;
	wheel2_pos_ += v2;
	wheel3_pos_ += v3;

	/// Compute linear and angular omni:
//	const double linear_x  = (v1*(cos(psi/2 + th) + cos(th)))/(sin(psi/2 + th)*cos(th) - cos(psi/2 + th)*sin(th) + cos(psi/2 - th)*sin(th) + sin(psi/2 - th)*cos(th) + cos(psi/2 + th)*sin(psi/2 - th) + sin(psi/2 + th)*cos(psi/2 - th)) - (v3*(cos(psi/2 + th) - cos(psi/2 - th)))/(sin(psi/2 + th)*cos(th) - cos(psi/2 + th)*sin(th) + cos(psi/2 - th)*sin(th) + sin(psi/2 - th)*cos(th) + cos(psi/2 + th)*sin(psi/2 - th) + sin(psi/2 + th)*cos(psi/2 - th)) - (v2*(cos(psi/2 - th) + cos(th)))/(sin(psi/2 + th)*cos(th) - cos(psi/2 + th)*sin(th) + cos(psi/2 - th)*sin(th) + sin(psi/2 - th)*cos(th) + cos(psi/2 + th)*sin(psi/2 - th) + sin(psi/2 + th)*cos(psi/2 - th));
//	const double linear_y  = (v2*(sin(psi/2 - th) - sin(th)))/(sin(psi/2 + th)*cos(th) - cos(psi/2 + th)*sin(th) + cos(psi/2 - th)*sin(th) + sin(psi/2 - th)*cos(th) + cos(psi/2 + th)*sin(psi/2 - th) + sin(psi/2 + th)*cos(psi/2 - th)) - (v3*(sin(psi/2 + th) + sin(psi/2 - th)))/(sin(psi/2 + th)*cos(th) - cos(psi/2 + th)*sin(th) + cos(psi/2 - th)*sin(th) + sin(psi/2 - th)*cos(th) + cos(psi/2 + th)*sin(psi/2 - th) + sin(psi/2 + th)*cos(psi/2 - th)) + (v1*(sin(psi/2 + th) + sin(th)))/(sin(psi/2 + th)*cos(th) - cos(psi/2 + th)*sin(th) + cos(psi/2 - th)*sin(th) + sin(psi/2 - th)*cos(th) + cos(psi/2 + th)*sin(psi/2 - th) + sin(psi/2 + th)*cos(psi/2 - th));
//	const double angular   = (v3*(cos(psi/2 + th)*sin(psi/2 - th) + sin(psi/2 + th)*cos(psi/2 - th)))/(L*cos(psi/2 + th)*sin(psi/2 - th) + L*sin(psi/2 + th)*cos(psi/2 - th) - L*cos(psi/2 + th)*sin(th) + L*sin(psi/2 + th)*cos(th) + L*cos(psi/2 - th)*sin(th) + L*sin(psi/2 - th)*cos(th)) - (v1*(cos(psi/2 + th)*sin(th) - sin(psi/2 + th)*cos(th)))/(L*cos(psi/2 + th)*sin(psi/2 - th) + L*sin(psi/2 + th)*cos(psi/2 - th) - L*cos(psi/2 + th)*sin(th) + L*sin(psi/2 + th)*cos(th) + L*cos(psi/2 - th)*sin(th) + L*sin(psi/2 - th)*cos(th)) + (v2*(cos(psi/2 - th)*sin(th) + sin(psi/2 - th)*cos(th)))/(L*cos(psi/2 + th)*sin(psi/2 - th) + L*sin(psi/2 + th)*cos(psi/2 - th) - L*cos(psi/2 + th)*sin(th) + L*sin(psi/2 + th)*cos(th) + L*cos(psi/2 - th)*sin(th) + L*sin(psi/2 - th)*cos(th));

	const double linear_x = (v1 / 2 / sin(psi / 2) - v2 / 2 / sin(psi / 2));
	const double linear_y = -v3 * 2 / 3 + v1 / 3 + v2 / 3;
	const double angular = v1 / 2 / (L + L / cos(psi / 2)) / cos(psi / 2) + v2 / 2 / (L + L / cos(psi / 2)) / cos(psi / 2) + v3 / (L + L / cos(psi / 2));
	ROS_INFO("velocity is %lf %lf %lf", wheel1_vel, wheel2_vel, wheel3_vel);

	/// We cannot estimate the speed with very small time intervals:
	const double dt = (time - timestamp_).toSec();
	if(dt < 0.0001)
		return false; // Interval too small to integrate with
	timestamp_ = time;

	/// Integrate odometry:
	integrate_fun_(linear_x, -linear_y, angular);
	/// Estimate speeds using a rolling mean to filter them out:
	// attention!!!!!! becaunse of the wrong direction of the axis
	linear_x_acc_(linear_x / dt);
	linear_y_acc_(linear_y / dt);
	angular_acc_(angular / dt);

	linear_x_ = bacc::rolling_mean(linear_x_acc_);
	linear_y_ = bacc::rolling_mean(linear_y_acc_);
	angular_ = bacc::rolling_mean(angular_acc_);

	return true;
}

void Odometry::setWheelParams(double wheel_center, double wheel_radius)
{
	wheel_center_ = wheel_center;
	wheel_radius_     = wheel_radius;
}


// TODO:: to be proof something wrong when using this
void Odometry::integrateRungeKutta2(double linear_x, double linear_y, double angular)
{
	const double direction = heading_ + angular * 0.5;

	/// Runge-Kutta 2nd order integration:
	x_       += linear_x * cos(direction) + linear_y * sin(direction);
	y_       += linear_y * sin(direction) + linear_x * cos(direction);
	heading_ += angular;
}

/**
 * \brief Other possible integration method provided by the class
 * \param linear_x
 * \param linear_y
 * \param angular
 */
void Odometry::integrateExact(double linear_x, double linear_y, double angular)
{
//	if(fabs(angular) < 10e-3)
//		integrateRungeKutta2(linear_x, linear_y, angular);
//	else
	{
		/// Exact integration (should solve problems when angular is zero):
		const double heading_old = heading_;
		heading_ += angular;
		x_       +=  linear_x * cos(heading_old) + linear_y * sin(heading_old);
		y_       +=  -linear_y * cos(heading_old) + linear_x * sin(heading_old);
	}
}

} // namespace diff_drive_controller
