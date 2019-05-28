/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SCHUNKWSGSIMDEVICE_HPP_
#define SCHUNKWSGSIMDEVICE_HPP_

#include "../interface/SchunkWsgDevice.hpp"
#include <rcc/Module.hpp>

namespace schunkwsg
{

	const std::string dev_schunkwsgsim = "schunk_wsg_sim";

	class SchunkWsgSimDevice: public SchunkWsgDevice
	{
	public:
		SchunkWsgSimDevice(std::string name, RPI::parameter_t parameters);
		virtual ~SchunkWsgSimDevice();

		static SchunkWsgDevice* createDevice(std::string name, RPI::parameter_t parameters);
		void updateParameters();
		std::set<std::string> getMutableParameters() const;
		RPI::DeviceState getDeviceState() const;

		void setEStop(bool estop);

		bool homing(bool openDirection);
		bool preposition(bool relative, double width, double speed, bool stopOnBlock);
		bool grasp(double width, double speed);
		bool release(double openWidth, double speed);
		bool setAcceleration(double acceleration);
		bool setForceLimit(double force);
		bool setSoftLimits(double inner, double outer);
		bool clearSoftLimits();
		void stopDevice();
		bool isBusy();
		StatusCode getStatusCode();
		double getOpeningWidth();
		double getVelocity();
		double getForce();
		GraspingState getGraspingState();

		void setPartLost();

	private:
		static const double MAX_OPENING_WIDTH;

		bool eStop;
		bool homed;

		double force;
		double softlimits_inner;
		double softlimits_outer;

		double desiredOpeningWidth;
		double initialOpeningWidth;
		
		double currentOpeningWidth;
		double currentForce;
		GraspingState currentGraspingState;
		StatusCode status_code;

		bool noPart;
		bool busy;
		static const double PENDING_DURATION;
		RTT::os::TimeService::nsecs startTime;
	};

} /* namespace schunkwsg */
#endif /* SCHUNKWSGSIMDEVICE_HPP_ */
