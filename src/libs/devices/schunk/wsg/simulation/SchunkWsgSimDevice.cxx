/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "SchunkWsgSimDevice.hpp"
#include <rtt/extras/PeriodicActivity.hpp>
#include <math.h>

namespace schunkwsg
{
	using namespace std;

	const double SchunkWsgSimDevice::MAX_OPENING_WIDTH = 0.11;
	const double SchunkWsgSimDevice::PENDING_DURATION = 2.0;

	SchunkWsgSimDevice::SchunkWsgSimDevice(std::string name, RPI::parameter_t parameters):
			Device(name, parameters), SchunkWsgDevice(name, parameters)
	{
		eStop = false;
		homed = false;

		force = 0.5;
		softlimits_inner = 0;
		softlimits_outer = MAX_OPENING_WIDTH;

		desiredOpeningWidth = 0;
		initialOpeningWidth = 0;
		
		currentOpeningWidth = 0;
		currentForce = 0;
		currentGraspingState = IDLE;
		status_code = E_SUCCESS;

		startTime = 0;
		busy = false;
	}

	SchunkWsgSimDevice::~SchunkWsgSimDevice()
	{
	}

	bool SchunkWsgSimDevice::homing(bool openDirection)
	{
		if (eStop) {
			status_code = E_ACCESS_DENIED;
		} else {
			desiredOpeningWidth = min(softlimits_outer, max(softlimits_inner, openDirection ? MAX_OPENING_WIDTH : 0));
			initialOpeningWidth = currentOpeningWidth;
			
			currentForce = 0;
			currentGraspingState = POSITIONING;
			homed = true;
			status_code = E_SUCCESS;
			startTime = RTT::os::TimeService::Instance()->getNSecs();
			busy = true;
		}

		return true;
	}

	bool SchunkWsgSimDevice::preposition(bool relative, double width, double speed, bool stopOnBlock)
	{
		double newOpeningWidth = relative ? currentOpeningWidth + width : width;

		if (eStop) {
			status_code = E_ACCESS_DENIED;
		} else if (!homed) {
			status_code = E_NOT_INITIALIZED;
		} else if (newOpeningWidth>softlimits_outer || newOpeningWidth<softlimits_inner) {
			status_code = E_RANGE_ERROR;
		} else {
			desiredOpeningWidth = newOpeningWidth;
			initialOpeningWidth = currentOpeningWidth;
			
			currentForce = 0;
			currentGraspingState = POSITIONING;
			noPart = false;
			status_code = E_SUCCESS;
			startTime = RTT::os::TimeService::Instance()->getNSecs();
			busy = true;
		}

		return true;
	}

	bool SchunkWsgSimDevice::grasp(double width, double speed)
	{
		preposition(false, width, speed, true);
		if (status_code==E_SUCCESS) {
			currentForce = force / 2;
			currentGraspingState = GRASPING;
		}
		return true;
	}

	bool SchunkWsgSimDevice::release(double openWidth, double speed)
	{
		preposition(false, openWidth, speed, false);
		if (status_code==E_SUCCESS) {
			currentGraspingState = RELEASING;
		}
		return true;
	}

	bool SchunkWsgSimDevice::setAcceleration(double acceleration)
	{
		status_code = E_SUCCESS;
		busy = false;
		return true;
	}

	bool SchunkWsgSimDevice::setForceLimit(double force)
	{
		status_code = E_SUCCESS;
		busy = false;
		this->force = force;
		return true;
	}

	bool SchunkWsgSimDevice::setSoftLimits(double inner, double outer)
	{
		status_code = E_SUCCESS;
		busy = false;
		softlimits_inner = inner;
		softlimits_outer = outer;
		return true;
	}

	bool SchunkWsgSimDevice::clearSoftLimits()
	{
		status_code = E_SUCCESS;
		busy = false;
		softlimits_inner = 0;
		softlimits_outer = MAX_OPENING_WIDTH;
		return true;
	}

	void SchunkWsgSimDevice::stopDevice()
	{
		status_code = E_SUCCESS;
		busy = false;
	}

	void SchunkWsgSimDevice::setPartLost() {
		noPart = true;
		if (currentGraspingState==HOLDING) currentGraspingState = PART_LOST;
	}

	bool SchunkWsgSimDevice::isBusy() {
		RTT::os::TimeService::Seconds timeLeft = RTT::os::TimeService::Instance()->getNSecs(startTime) / 1e9;
		if (busy && status_code==E_SUCCESS) {
			if(timeLeft>=PENDING_DURATION) {
				currentOpeningWidth = desiredOpeningWidth;
			
				if (eStop) {
					status_code = E_ACCESS_DENIED;
					currentGraspingState = IDLE;
				} else if (currentGraspingState==GRASPING) {
					if (noPart) {
						status_code = E_CMD_FAILED;
					}
					currentGraspingState = (noPart ? NO_PART_FOUND : HOLDING);
				} else {
					currentGraspingState = IDLE;
				}
				busy = false;
			}
			else {
				double dist = desiredOpeningWidth - initialOpeningWidth;
				double fact = timeLeft / PENDING_DURATION;
				currentOpeningWidth = initialOpeningWidth + (dist * fact);
			}
		}
		return busy;
	}

	StatusCode SchunkWsgSimDevice::getStatusCode() {
		isBusy();
		return status_code;
	}

	double SchunkWsgSimDevice::getOpeningWidth() {
		return currentOpeningWidth;
	}

	double SchunkWsgSimDevice::getVelocity() {
		return 0;
	}

	double SchunkWsgSimDevice::getForce() {
		return currentForce;
	}

	GraspingState SchunkWsgSimDevice::getGraspingState() {
		isBusy();
		return currentGraspingState;
	}

	SchunkWsgDevice* SchunkWsgSimDevice::createDevice(std::string name, RPI::parameter_t parameters) {
		SchunkWsgSimDevice* ret = new SchunkWsgSimDevice(name, parameters);
		return ret;
	}

	void SchunkWsgSimDevice::updateParameters() {

	}

	std::set<std::string> SchunkWsgSimDevice::getMutableParameters() const {
		return std::set<std::string>();
	}

	void SchunkWsgSimDevice::setEStop(bool estop) {
		eStop = estop;
	}

	RPI::DeviceState SchunkWsgSimDevice::getDeviceState() const {
		return RPI::DeviceState::OPERATIONAL;
	}

} /* namespace schunkwsg */
