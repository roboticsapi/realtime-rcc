/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "../interface/CartesianPositionInterface.hpp"
#include "rtt/Logger.hpp"
#include <rcc/DeviceInstanceT.hpp>
#include <rcc/Net.hpp>

namespace cartesianposition
{

	class CartesianPosition: public RPI::Module
	{
	public:
		CartesianPosition(std::string name, RPI::Net* net) :
				RPI::Module(name, net),
				inPosition("inPosition", this),
				outError("outError", this),
				outErrorConcurrentAccess("outErrorConcurrentAccess", this),
				outErrorDeviceFailed("outErrorDeviceFailed", this),
				outErrorIllegalPosition("outErrorIllegalPosition", this),
				propRobot("Robot", "Name of the robot", ""),
				devins(), error(1), hasValue(false)
		{
			setDescription("Module to command the Cartesian position.");
			this->ports()->addPort(&inPosition, "Desired position (of moving frame relative to reference frame)");
			this->ports()->addPort(&outError, "Status (nonzero means error)");
			this->ports()->addPort(&outErrorConcurrentAccess,
					"Concurrent access error (device is used by another primitive)");
			this->ports()->addPort(&outErrorIllegalPosition, "Illegal position (the value of inPosition is invalid)");
			this->ports()->addPort(&outErrorDeviceFailed, "Device error (the devic eis not operational)");
			this->properties()->addProperty(&propRobot);

		}

		~CartesianPosition()
		{
		}

		bool configureHook()
		{
			return deviceAvailable(&devins, propRobot.get()) &&
					portConnected(&inPosition);
		}

		bool startHook()
		{
			return true;
		}

		bool isActuator()
		{
			return true;
		}

		std::string getDeviceName()
		{
			return propRobot.get();
		}

		void updateHook()
		{
			pos = inPosition.Get();
			int accessError = *resourceUsed[0] ? -1 : 0;
			int deviceError = devins->getCartesianPositionDeviceError();
			int positionError = devins->checkPosition(pos);

			outErrorConcurrentAccess.Set(accessError != 0);
			outErrorDeviceFailed.Set(deviceError != 0);
			outErrorIllegalPosition.Set(positionError != 0);

			*resourceUsed[0] = true;
			hasValue = false;

			if (accessError != 0)
			{
				error = accessError;
			} else if (deviceError != 0)
			{
				error = deviceError;
			} else if (inPosition.isNull())
			{
				error = 0;
			} else if (positionError != 0)
			{
				error = positionError;
			} else
			{
				error = 0;
				hasValue = true;
			}
			outError.Set(error);
		}

		void updateActuator()
		{
			if (hasValue)
			{
				devins->setPosition(pos, inNet->getTime());
				hasValue = false;
			}
		}

		void stopHook()
		{
			// Your stop code after last updateHook()
		}

		void cleanupHook()
		{
			// Your configuration cleanup code
		}

		std::set<std::string> getResourceNames() const
		{
			std::set<std::string> ret;
			ret.insert(devins->getCartesianPositionResourceName());
			return ret;
		}

	private:
		int error;
		bool hasValue;
		KDL::Frame pos;

		RPI::InPort<KDL::Frame> inPosition;
		RPI::OutPort<int> outError;
		RPI::OutPort<bool> outErrorConcurrentAccess;
		RPI::OutPort<bool> outErrorDeviceFailed;
		RPI::OutPort<bool> outErrorIllegalPosition;

		RPI::Property<std::string> propRobot;

		RPI::DeviceInstanceT<CartesianPositionInterface> devins;

	};

	class CartesianMonitor: public RPI::Module
	{
	public:
		CartesianMonitor(std::string name, RPI::Net* net) :
				Module(name, net),
				outMsrPos("outMeasuredPosition", this),
				outMsrVel("outMeasuredVelocity", this),
				outCmdPos("outCommandedPosition", this),
				outCmdVel("outCommandedVelocity", this),
				outError("outError", this),
				propRobot("Robot", "Name of the robot", ""),
				devins(), error(0)
		{
			setDescription("Module to monitor the velocity and position of a robot base.");
			this->ports()->addPort(&outMsrPos,
					"Current measured position (transformation from reference frame to moving frame)");
			this->ports()->addPort(&outMsrVel,
					"Current measured velocity (velocity of moving frame relative to reference frame, expressed in moving frame)");
			this->ports()->addPort(&outCmdPos,
					"Current commanded position (transformation from reference frame to moving frame)");
			this->ports()->addPort(&outCmdVel,
					"Current commanded velocity (velocity of moving frame relative to reference frame, expressed in moving frame)");
			this->ports()->addPort(&outError, "Status (nonzero means error)");

			this->properties()->addProperty(&propRobot);
		}

		~CartesianMonitor()
		{
		}

		bool configureHook()
		{
			return deviceAvailable(&devins, propRobot.get());
		}

		bool startHook()
		{
			return true;
		}

		bool isSensor()
		{
			return true;
		}

		std::string getDeviceName()
		{
			return propRobot.get();
		}

		void updateSensor()
		{
			pos = devins->getMeasuredPosition();
			vel = devins->getMeasuredVelocity();
			cmdvel = devins->getCommandedVelocity();
			cmdpos = devins->getCommandedPosition();
			error = devins->getCartesianPositionDeviceError();
		}

		void updateHook()
		{
			outMsrPos.Set(pos);
			outMsrVel.Set(vel);
			outCmdPos.Set(cmdpos);
			outCmdVel.Set(cmdvel);
			outError.Set(error);
		}

		void stopHook()
		{
			// Your stop code after last updateHook()
		}

		void cleanupHook()
		{
			// Your configuration cleanup code
		}

	private:

		KDL::Frame pos;
		KDL::Twist vel;
		KDL::Frame cmdpos;
		KDL::Twist cmdvel;

		RPI::OutPort<KDL::Frame> outMsrPos;
		RPI::OutPort<KDL::Twist> outMsrVel;
		RPI::OutPort<KDL::Frame> outCmdPos;
		RPI::OutPort<KDL::Twist> outCmdVel;
		RPI::OutPort<int> outError;
		RPI::Property<std::string> propRobot;

		RPI::DeviceInstanceT<CartesianPositionInterface> devins;

		int error;
	};

}
