/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SCHUNKWSGDEVICE_HPP_
#define SCHUNKWSGDEVICE_HPP_

#include <string>
#include "StatusCode.hpp"
#include "GraspingState.hpp"
#include <rcc/Device.hpp>

namespace schunkwsg
{

	typedef unsigned char  uint8;
	typedef unsigned short uint16;
	typedef unsigned int   uint32;


	class RTT_EXPORT SchunkWsgDevice: public virtual RPI::Device
	{
	public:
		SchunkWsgDevice(std::string name, RPI::parameter_t parameters): RPI::Device(name, parameters) {};
		virtual ~SchunkWsgDevice() {};

		virtual bool homing(bool openDirection) = 0;
		virtual bool preposition(bool relative, double width, double speed, bool stopOnBlock) = 0;
		virtual bool grasp(double width, double speed) = 0;
		virtual bool release(double openWidth, double speed) = 0;
		virtual bool setAcceleration(double acceleration) = 0;
		virtual bool setForceLimit(double force) = 0;
		virtual bool setSoftLimits(double inner, double outer) = 0;
		virtual bool clearSoftLimits() = 0;
		virtual void stopDevice() = 0;
		virtual bool isBusy() = 0;
		virtual StatusCode getStatusCode() = 0;
		virtual double getOpeningWidth() = 0;
		virtual double getVelocity() = 0;
		virtual double getForce() = 0;
		virtual GraspingState getGraspingState() = 0;
	};

} /* namespace schunkwsg */
#endif /* SCHUNKWSGDEVICE_HPP_ */
