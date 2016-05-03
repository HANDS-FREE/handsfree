/*
 * base_state_interface.h
 *
 *  Created on: Jan 7, 2016
 *      Author: liao
 */

#ifndef BASE_STATE_INTERFACE_H_
#define BASE_STATE_INTERFACE_H_

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace hardware_interface
{
class BaseStateHandle
{
public:
	BaseStateHandle() : name_(), x_(), y_(), theta_() {}

	BaseStateHandle(const std::string& name, const double* x, const double* y, const double* theta, const double* x_vel, const double* y_vel, const double* theta_vel)
	: name_(name), x_(x), y_(y), theta_(theta), x_vel_(x_vel), y_vel_(y_vel), theta_vel_(theta_vel)
	{
	if (!x)
	{
	  throw HardwareInterfaceException("Cannot create handle '" + name + "'. x data pointer is null.");
	}
	if (!y)
	{
	  throw HardwareInterfaceException("Cannot create handle '" + name + "'. y data pointer is null.");
	}
	if (!theta)
	{
	  throw HardwareInterfaceException("Cannot create handle '" + name + "'. theta data pointer is null.");
	}
	}

	std::string getName() const {return name_;}
	double getX() const {assert(x_); return *x_;}
	double getY() const {assert(y_); return *y_;}
	double getTheta() const {assert(theta_); return *theta_;}

	double getXvel() const {assert(x_vel_); return *x_vel_;}
	double getYvel() const {assert(y_vel_); return *y_vel_;}
	double getThetavel() const {assert(theta_vel_); return *theta_vel_;}

private:
	std::string name_;
	const double* x_;
	const double* y_;
	const double* theta_;

	const double* x_vel_;
	const double* y_vel_;
	const double* theta_vel_;
};

class BaseStateInterface : public HardwareResourceManager<BaseStateHandle> {};
}



#endif /* BASE_STATE_INTERFACE_H_ */
