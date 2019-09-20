/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "SchunkWsgBlocks.hpp"
#include <rtt/Logger.hpp>

namespace schunkwsg
{

	Homing::Homing(std::string name, RPI::Net* net) :
			RPI::StatefulModule(name, net), outFinished("outFinished", this), outStatusCode("outStatusCode", this), propDirection("Direction",
					"true for 'open', false for 'close'.", true), propDevice("Device", "Name of the device", ""), devins()
	{
		setDescription("Module to control homing of a gripper device.");
		this->ports()->addPort(&outFinished, "Finished");
		this->ports()->addPort(&outStatusCode, "StatusCode");

		this->properties()->addProperty(&propDirection);
		this->properties()->addProperty(&propDevice);

		init = true;
	}

	Homing::~Homing()
	{
	}

	bool Homing::configureHook()
	{
		if (!devins.fetchInstance(propDevice.get()))
		{
			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool Homing::startHook()
	{
		return true;
	}

	void Homing::updateHook()
	{
		if (reset())
			init = true;

		SchunkWsgDevice* device = devins.getDevice();
		if (active() && device)
		{
			if (init) {
				init = false;
				if (device->getDeviceState()!=RPI::DeviceState::OPERATIONAL) {
					outStatusCode.Set(31); // 31 is NOT_OPERATIONAL
					outFinished.Set(true);
				}
				else {
					bool result = device->homing(propDirection.get());
					if (!result) {
						outStatusCode.Set(32); // 32 is CONCURRENT_ACCESS
						outFinished.Set(true);
					}
					else {
						outStatusCode.SetNull();
						outFinished.Set(false);
					}
				}
			}
			else {
				if (device->isBusy()) {
					outStatusCode.SetNull();
					outFinished.Set(false);
				}
				else {
					outStatusCode.Set(device->getStatusCode());
					outFinished.Set(true);
				}
			}
		}
	}

	void Homing::stopHook()
	{
	}

	void Homing::cleanupHook()
	{
	}





	Prepositioning::Prepositioning(std::string name, RPI::Net* net) :
			RPI::StatefulModule(name, net), outFinished("outFinished", this), outStatusCode("outStatusCode", this), propWidth("Width",
					"Opening width in [m].", 0), propSpeed("Speed", "Traveling speed in [m/s].", 0), propRelativeMotion("RelativeMotion", "If 'true', the passed width is treated as an offset to the current opening width.", true), propStopOnBlock("StopOnBlock", "Important for the case that the opening movement is blocked by any obstacle. If 'false', the motor is stopped. If 'true', the motor is not turned off automatically, but clamps with the previously set force limit.", false), propDevice("Device", "Name of the device", ""), devins()
	{
		setDescription("Module to control prepositioning of a gripper device.");
		this->ports()->addPort(&outFinished, "Finished");
		this->ports()->addPort(&outStatusCode, "StatusCode");

		this->properties()->addProperty(&propWidth);
		this->properties()->addProperty(&propSpeed);
		this->properties()->addProperty(&propRelativeMotion);
		this->properties()->addProperty(&propStopOnBlock);
		this->properties()->addProperty(&propDevice);

		init = true;
	}

	Prepositioning::~Prepositioning()
	{
	}

	bool Prepositioning::configureHook()
	{
		if (!devins.fetchInstance(propDevice.get()))
		{
			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool Prepositioning::startHook()
	{
		return true;
	}

	void Prepositioning::updateHook()
	{
		if (reset())
			init = true;

		SchunkWsgDevice* device = devins.getDevice();
		if (active() && device)
		{
			if (init) {
				init = false;
				if (device->getDeviceState()!=RPI::DeviceState::OPERATIONAL) {
					outStatusCode.Set(31); // 31 is NOT_OPERATIONAL
					outFinished.Set(true);
				}
				else {
					bool result = device->preposition(propRelativeMotion.get(), propWidth.get(), propSpeed.get(), propStopOnBlock.get());
					if (!result) {
						outStatusCode.Set(32); // 32 is CONCURRENT_ACCESS
						outFinished.Set(true);
					}
					else {
						outStatusCode.SetNull();
						outFinished.Set(false);
					}
				}
			}
			else {
				if (device->isBusy()) {
					outStatusCode.SetNull();
					outFinished.Set(false);
				}
				else {
					outStatusCode.Set(device->getStatusCode());
					outFinished.Set(true);
				}
			}
		}
	}

	void Prepositioning::stopHook()
	{
	}

	void Prepositioning::cleanupHook()
	{
	}







	Grasping::Grasping(std::string name, RPI::Net* net) :
			RPI::StatefulModule(name, net), outFinished("outFinished", this), outStatusCode("outStatusCode", this), propWidth("Width",
					"Expected part width in [m].", 0), propSpeed("Speed", "Traveling speed in [m/s].", 0), propDevice("Device", "Name of the device", ""), devins()
	{
		setDescription("Module to control grasping of a gripper device.");
		this->ports()->addPort(&outFinished, "Finished");
		this->ports()->addPort(&outStatusCode, "StatusCode");

		this->properties()->addProperty(&propWidth);
		this->properties()->addProperty(&propSpeed);
		this->properties()->addProperty(&propDevice);

		init = true;
	}

	Grasping::~Grasping()
	{
	}

	bool Grasping::configureHook()
	{
		if (!devins.fetchInstance(propDevice.get()))
		{
			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool Grasping::startHook()
	{
		return true;
	}

	void Grasping::updateHook()
	{
		if (reset())
			init = true;

		SchunkWsgDevice* device = devins.getDevice();
		if (active() && device)
		{
			if (init) {
				init = false;
				if (device->getDeviceState()!=RPI::DeviceState::OPERATIONAL) {
					outStatusCode.Set(31); // 31 is NOT_OPERATIONAL
					outFinished.Set(true);
				}
				else {
					bool result = device->grasp(propWidth.get(), propSpeed.get());
					if (!result) {
						outStatusCode.Set(32); // 32 is CONCURRENT_ACCESS
						outFinished.Set(true);
					}
					else {
						outStatusCode.SetNull();
						outFinished.Set(false);
					}
				}
			}
			else {
				if (device->isBusy()) {
					outStatusCode.SetNull();
					outFinished.Set(false);
				}
				else {
					outStatusCode.Set(device->getStatusCode());
					outFinished.Set(true);
				}
			}
		}
	}

	void Grasping::stopHook()
	{
	}

	void Grasping::cleanupHook()
	{
	}







	Releasing::Releasing(std::string name, RPI::Net* net) :
			RPI::StatefulModule(name, net), outFinished("outFinished", this), outStatusCode("outStatusCode", this), propOpenWidth("OpenWidth",
					"Opening width in [m].", 0), propSpeed("Speed", "Traveling speed in [m/s].", 0), propDevice("Device", "Name of the device", ""), devins()
	{
		setDescription("Module to control releasing of a gripper device.");
		this->ports()->addPort(&outFinished, "Finished");
		this->ports()->addPort(&outStatusCode, "StatusCode");

		this->properties()->addProperty(&propOpenWidth);
		this->properties()->addProperty(&propSpeed);
		this->properties()->addProperty(&propDevice);
		init = true;
	}

	Releasing::~Releasing()
	{
	}

	bool Releasing::configureHook()
	{
		if (!devins.fetchInstance(propDevice.get()))
		{
			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool Releasing::startHook()
	{
		return true;
	}

	void Releasing::updateHook()
	{
		if (reset())
			init = true;

		SchunkWsgDevice* device = devins.getDevice();
		if (active() && device)
		{
			if (init) {
				init = false;
				if (device->getDeviceState()!=RPI::DeviceState::OPERATIONAL) {
					outStatusCode.Set(31); // 31 is NOT_OPERATIONAL
					outFinished.Set(true);
				}
				else {
					bool result = device->release(propOpenWidth.get(), propSpeed.get());
					if (!result) {
						outStatusCode.Set(32); // 32 is CONCURRENT_ACCESS
						outFinished.Set(true);
					}
					else {
						outStatusCode.SetNull();
						outFinished.Set(false);
					}
				}
			}
			else {
				if (device->isBusy()) {
					outStatusCode.SetNull();
					outFinished.Set(false);
				}
				else {
					outStatusCode.Set(device->getStatusCode());
					outFinished.Set(true);
				}
			}
		}
	}

	void Releasing::stopHook()
	{
	}

	void Releasing::cleanupHook()
	{
	}







	SetAcceleration::SetAcceleration(std::string name, RPI::Net* net) :
			RPI::StatefulModule(name, net), outFinished("outFinished", this), outStatusCode("outStatusCode", this), propAcceleration("Acceleration",
					"Acceleration in [m/s²]. The value is clamped, if it is outside the device's capabilities.", 0), propDevice("Device", "Name of the device", ""), devins()
	{
		setDescription("Module to control the acceleration of a gripper device.");
		this->ports()->addPort(&outFinished, "Finished");
		this->ports()->addPort(&outStatusCode, "StatusCode");

		this->properties()->addProperty(&propAcceleration);
		this->properties()->addProperty(&propDevice);
		init = true;
	}

	SetAcceleration::~SetAcceleration()
	{
	}

	bool SetAcceleration::configureHook()
	{
		if (!devins.fetchInstance(propDevice.get()))
		{
			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool SetAcceleration::startHook()
	{
		return true;
	}

	void SetAcceleration::updateHook()
	{
		if (reset())
			init = true;

		SchunkWsgDevice* device = devins.getDevice();
		if (active() && device)
		{
			if (init) {
				init = false;
				if (device->getDeviceState()!=RPI::DeviceState::OPERATIONAL) {
					outStatusCode.Set(31); // 31 is NOT_OPERATIONAL
					outFinished.Set(true);
				}
				else {
					bool result = device->setAcceleration(propAcceleration.get());
					if (!result) {
						outStatusCode.Set(32); // 32 is CONCURRENT_ACCESS
						outFinished.Set(true);
					}
					else {
						outStatusCode.SetNull();
						outFinished.Set(false);
					}
				}
			}
			else {
				if (device->isBusy()) {
					outStatusCode.SetNull();
					outFinished.Set(false);
				}
				else {
					outStatusCode.Set(device->getStatusCode());
					outFinished.Set(true);
				}
			}
		}
	}

	void SetAcceleration::stopHook()
	{
	}

	void SetAcceleration::cleanupHook()
	{
	}







	SetForceLimit::SetForceLimit(std::string name, RPI::Net* net) :
			RPI::StatefulModule(name, net), outFinished("outFinished", this), outStatusCode("outStatusCode", this), propForce("Force",
					"Force limit in Newtons. The value is clamped, if it is outside the device's capabilities.", 0), propDevice("Device", "Name of the device", ""), devins()
	{
		setDescription("Module to control the acceleration of a gripper device.");
		this->ports()->addPort(&outFinished, "Finished");
		this->ports()->addPort(&outStatusCode, "StatusCode");

		this->properties()->addProperty(&propForce);
		this->properties()->addProperty(&propDevice);
		init = true;
	}

	SetForceLimit::~SetForceLimit()
	{
	}

	bool SetForceLimit::configureHook()
	{
		if (!devins.fetchInstance(propDevice.get()))
		{
			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool SetForceLimit::startHook()
	{
		return true;
	}

	void SetForceLimit::updateHook()
	{
		if (reset())
			init = true;

		SchunkWsgDevice* device = devins.getDevice();
		if (active() && device)
		{
			if (init) {
				init = false;
				if (device->getDeviceState()!=RPI::DeviceState::OPERATIONAL) {
					outStatusCode.Set(31); // 31 is NOT_OPERATIONAL
					outFinished.Set(true);
				}
				else {
					bool result = device->setForceLimit(propForce.get());
					if (!result) {
						outStatusCode.Set(32); // 32 is CONCURRENT_ACCESS
						outFinished.Set(true);
					}
					else {
						outStatusCode.SetNull();
						outFinished.Set(false);
					}
				}
			}
			else {
				if (device->isBusy()) {
					outStatusCode.SetNull();
					outFinished.Set(false);
				}
				else {
					outStatusCode.Set(device->getStatusCode());
					outFinished.Set(true);
				}
			}
		}
	}

	void SetForceLimit::stopHook()
	{
	}

	void SetForceLimit::cleanupHook()
	{
	}







	SetSoftLimits::SetSoftLimits(std::string name, RPI::Net* net) :
			RPI::StatefulModule(name, net), outFinished("outFinished", this), outStatusCode("outStatusCode", this), propInner("Inner",
					"Soft limit opening width in negative motion direction ([m]).", 0), propOuter("Outer", "Soft limit opening width in positive motion direction ([m]).", 0), propDevice("Device", "Name of the device", ""), devins()
	{
		setDescription("Module to control releasing of a gripper device.");
		this->ports()->addPort(&outFinished, "Finished");
		this->ports()->addPort(&outStatusCode, "StatusCode");

		this->properties()->addProperty(&propInner);
		this->properties()->addProperty(&propOuter);
		this->properties()->addProperty(&propDevice);
		init = true;
	}

	SetSoftLimits::~SetSoftLimits()
	{
	}

	bool SetSoftLimits::configureHook()
	{
		if (!devins.fetchInstance(propDevice.get()))
		{
			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool SetSoftLimits::startHook()
	{
		return true;
	}

	void SetSoftLimits::updateHook()
	{
		if (reset())
			init = true;

		SchunkWsgDevice* device = devins.getDevice();
		if (active() && device)
		{
			if (init) {
				init = false;
				if (device->getDeviceState()!=RPI::DeviceState::OPERATIONAL) {
					outStatusCode.Set(31); // 31 is NOT_OPERATIONAL
					outFinished.Set(true);
				}
				else {
					bool result = device->setSoftLimits(propInner.get(), propOuter.get());
					if (!result) {
						outStatusCode.Set(32); // 32 is CONCURRENT_ACCESS
						outFinished.Set(true);
					}
					else {
						outStatusCode.SetNull();
						outFinished.Set(false);
					}
				}
			}
			else {
				if (device->isBusy()) {
					outStatusCode.SetNull();
					outFinished.Set(false);
				}
				else {
					outStatusCode.Set(device->getStatusCode());
					outFinished.Set(true);
				}
			}
		}
	}

	void SetSoftLimits::stopHook()
	{
	}

	void SetSoftLimits::cleanupHook()
	{
	}







	ClearSoftLimits::ClearSoftLimits(std::string name, RPI::Net* net) :
			RPI::StatefulModule(name, net), outFinished("outFinished", this), outStatusCode("outStatusCode", this), propDevice("Device", "Name of the device", ""), devins()
	{
		setDescription("Module to control releasing of a gripper device.");
		this->ports()->addPort(&outFinished, "Finished");
		this->ports()->addPort(&outStatusCode, "StatusCode");

		this->properties()->addProperty(&propDevice);
		init = true;
	}

	ClearSoftLimits::~ClearSoftLimits()
	{
	}

	bool ClearSoftLimits::configureHook()
	{
		if (!devins.fetchInstance(propDevice.get()))
		{
			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool ClearSoftLimits::startHook()
	{
		return true;
	}

	void ClearSoftLimits::updateHook()
	{
		if (reset())
			init = true;

		SchunkWsgDevice* device = devins.getDevice();
		if (active() && device)
		{
			if (init) {
				init = false;
				if (device->getDeviceState()!=RPI::DeviceState::OPERATIONAL) {
					outStatusCode.Set(31); // 31 is NOT_OPERATIONAL
					outFinished.Set(true);
				}
				else {
					bool result = device->clearSoftLimits();
					if (!result) {
						outStatusCode.Set(32); // 32 is CONCURRENT_ACCESS
						outFinished.Set(true);
					}
					else {
						outStatusCode.SetNull();
						outFinished.Set(false);
					}
				}
			}
			else {
				if (device->isBusy()) {
					outStatusCode.SetNull();
					outFinished.Set(false);
				}
				else {
					outStatusCode.Set(device->getStatusCode());
					outFinished.Set(true);
				}
			}
		}
	}

	void ClearSoftLimits::stopHook()
	{
	}

	void ClearSoftLimits::cleanupHook()
	{
	}







//	AcknowledgeFaststopOrFault::AcknowledgeFaststopOrFault(std::string name, RPI::Net* net) :
//			RPI::StatefulModule(name, net), outFinished("outFinished", this), outStatusCode("outStatusCode", this), propDevice("Device", "Name of the device", ""), devins()
//	{
//		setDescription("Module to control releasing of a gribber device.");
//		this->ports()->addPort(&outFinished, "Finished");
//		this->ports()->addPort(&outStatusCode, "StatusCode");
//
//		this->properties()->addProperty(&propDevice);
//		init = true;
//	}
//
//	AcknowledgeFaststopOrFault::~AcknowledgeFaststopOrFault()
//	{
//	}
//
//	bool AcknowledgeFaststopOrFault::configureHook()
//	{
//		if (!devins.fetchInstance(propDevice.get()))
//		{
//			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
//			return false;
//		}
//		return true;
//	}
//
//	bool AcknowledgeFaststopOrFault::startHook()
//	{
//		return true;
//	}
//
//	void AcknowledgeFaststopOrFault::updateHook()
//	{
//		if (reset())
//			init = true;
//
//		SchunkWsgDevice* device = devins.getDevice();
//		if (active() && device)
//		{
//			if (init) {
//				init = false;
//				if (device->getDeviceState()!=RPI::OPERATIONAL) {
//					outStatusCode.Set(31); // 31 is NOT_OPERATIONAL
//					outFinished.Set(true);
//				}
//				else {
//					bool result = device->acknowledgeFaststopOrFault();
//					if (!result) {
//						outStatusCode.Set(32); // 32 is CONCURRENT_ACCESS
//						outFinished.Set(true);
//					}
//					else {
//						outStatusCode.SetNull();
//						outFinished.Set(false);
//					}
//				}
//			}
//			else {
//				if (device->isBusy()) {
//					outStatusCode.SetNull();
//					outFinished.Set(false);
//				}
//				else {
//					outStatusCode.Set(device->getStatusCode());
//					outFinished.Set(true);
//				}
//			}
//		}
//	}
//
//	void AcknowledgeFaststopOrFault::stopHook()
//	{
//	}
//
//	void AcknowledgeFaststopOrFault::cleanupHook()
//	{
//	}







	StopDevice::StopDevice(std::string name, RPI::Net* net) :
			RPI::StatefulModule(name, net), outFinished("outFinished", this), propDevice("Device", "Name of the device", ""), devins()
	{
		setDescription("Module to stop any motion of a gripper device.");
		this->ports()->addPort(&outFinished, "Finished");

		this->properties()->addProperty(&propDevice);
		init = true;
	}

	StopDevice::~StopDevice()
	{
	}

	bool StopDevice::configureHook()
	{
		if (!devins.fetchInstance(propDevice.get()))
		{
			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool StopDevice::startHook()
	{
		return true;
	}

	void StopDevice::updateHook()
	{
		if (reset())
			init = true;
		if (active() && devins.getDevice())
		{
			if (init)
			{
				devins.getDevice()->stopDevice();
				init = false;
			}
			outFinished.Set(true);
		}
	}

	void StopDevice::stopHook()
	{
	}

	void StopDevice::cleanupHook()
	{
	}







//	FastStop::FastStop(std::string name, RPI::Net* net) :
//			RPI::StatefulModule(name, net), outFinished("outFinished", this), propDevice("Device", "Name of the device", ""), devins()
//	{
//		setDescription("Module to set the fast stop flag of a gribber device.");
//		this->ports()->addPort(&outFinished, "Finished");
//
//		this->properties()->addProperty(&propDevice);
//		init = true;
//	}
//
//	FastStop::~FastStop()
//	{
//	}
//
//	bool FastStop::configureHook()
//	{
//		if (!devins.fetchInstance(propDevice.get()))
//		{
//			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
//			return false;
//		}
//		return true;
//	}
//
//	bool FastStop::startHook()
//	{
//		return true;
//	}
//
//	void FastStop::updateHook()
//	{
//		if (reset())
//			init = true;
//		if (active() && devins.getDevice())
//		{
//			if (init)
//			{
//				devins.getDevice()->fastStop();
//				init = false;
//			}
//			outFinished.Set(true);
//		}
//	}
//
//	void FastStop::stopHook()
//	{
//	}
//
//	void FastStop::cleanupHook()
//	{
//	}







	Monitor::Monitor(std::string name, RPI::Net* net) :
			RPI::Module(name, net), outOpeningWidth("outOpeningWidth", this), outSpeed("outSpeed", this), outForce("outForce", this), outGraspingState("outGraspingState", this), propDevice("Device", "Name of the device", ""), devins()
	{
		setDescription("Module which gives information about current values of a gripper device.");
		this->ports()->addPort(&outOpeningWidth, "Opening width");
		this->ports()->addPort(&outSpeed, "Speed");
		this->ports()->addPort(&outForce, "Force");
		this->ports()->addPort(&outGraspingState, "Grasping state");

		this->properties()->addProperty(&propDevice);
	}

	Monitor::~Monitor()
	{
	}

	bool Monitor::configureHook()
	{
		if (!devins.fetchInstance(propDevice.get()))
		{
			RTT::log(RTT::Error) << "SchunkWsg " << propDevice.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool Monitor::startHook()
	{
		return true;
	}

	void Monitor::updateHook()
	{
		if (devins.getDevice())
		{
			outOpeningWidth.Set(devins.getDevice()->getOpeningWidth());
			outSpeed.Set(devins.getDevice()->getVelocity());
			outForce.Set(devins.getDevice()->getForce());
			outGraspingState.Set(devins.getDevice()->getGraspingState());
		}
		else {
			outOpeningWidth.Set(0);
			outSpeed.Set(0);
			outForce.Set(0);
			outGraspingState.Set(0);
		}
	}

	void Monitor::stopHook()
	{
	}

	void Monitor::cleanupHook()
	{
	}

} /* namespace schunkwsg */
