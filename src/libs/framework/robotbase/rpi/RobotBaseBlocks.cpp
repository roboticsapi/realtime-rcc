/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "../interface/RobotBaseInterface.hpp"
#include "rtt/Logger.hpp"
#include <rcc/DeviceInstanceT.hpp>
#include <rcc/Net.hpp>

namespace robotbase {

	class BaseVelocity: public RPI::ActiveModule
	{
	public:
		BaseVelocity(std::string name, RPI::Net* net) :
			RPI::ActiveModule(name, net),
			inVel("inVel", this),
			outError("outError", this),
			outErrorConcurrentAccess("outErrorConcurrentAccess", this),
			outErrorBaseFailed("outErrorBaseFailed", this),
			outErrorIllegalVelocity("outErrorIllegalVelocity", this),
			propRobot("Robot", "Name of the robot", ""),
			devins(), error(1)
		{
			setDescription("Module to command the velocity of a robot base.");
			this->ports()->addPort(&inVel, "Desired velocity (of moving frame relative to reference frame, expressed in moving frame)");
			this->ports()->addPort(&outError, "Status (nonzero means error)");
			this->ports()->addPort(&outErrorConcurrentAccess, "Concurrent access error (base is used by another primitive)");
			this->ports()->addPort(&outErrorIllegalVelocity, "Illegal base velocity (the value of inVelocity is invalid)");
			this->ports()->addPort(&outErrorBaseFailed, "Base error (the base is not operational)");
			this->properties()->addProperty(&propRobot);
		}

		~BaseVelocity()
		{
		}

		bool configureHook()
		{
			return deviceAvailable(&devins, propRobot.get()) &&
					portConnected(&inVel);
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
			if (active())
			{
				vel = inVel.Get();
				int accessError = *resourceUsed[0] ? -1 : 0;
				int baseError = devins.getDevice()->getBaseError();
				int velocityError = devins.getDevice()->checkBaseVelocity(vel);

				outErrorConcurrentAccess.Set(accessError != 0);
				outErrorBaseFailed.Set(baseError != 0);
				outErrorIllegalVelocity.Set(velocityError != 0);

				*resourceUsed[0] = true;

				if(accessError != 0)
				{
					error = accessError;
				}
				else if(baseError != 0)
				{
					error = baseError;
				}
				else if(velocityError != 0)
				{
					error = velocityError;
				}
				else
				{
					error = 0;
				}
				outError.Set(error);
			}
		}

		void updateActuator()
		{
			if(error == 0) {
				devins.getDevice()->setBaseVelocity(vel);
				error = 1;
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
			ret.insert(devins->getRobotBaseResourceName());
			return ret;
		}

	private:
		int error;
		KDL::Twist vel;

		RPI::InPort<KDL::Twist> inVel;
		RPI::OutPort<int> outError;
		RPI::OutPort<bool> outErrorConcurrentAccess;
		RPI::OutPort<bool> outErrorBaseFailed;
		RPI::OutPort<bool> outErrorIllegalVelocity;

		RPI::Property<std::string> propRobot;

		RPI::DeviceInstanceT<RobotBaseInterface> devins;

	};


	class BaseMonitor: public RPI::Module
	{
	public:
		BaseMonitor(std::string name, RPI::Net* net) :
			Module(name, net), outPos("outPos", this), outVel("outVel", this), outCmdVel("outCmdVel", this), outError("outError", this),
					propRobot("Robot", "Name of the robot", ""), devins(), error(0)
		{
			setDescription("Module to monitor the velocity and position of a robot base.");
			this->ports()->addPort(&outPos, "Current measured position (transformation from reference frame to moving frame)");
			this->ports()->addPort(&outVel, "Current measured velocity (velocity of moving frame relative to reference frame, expressed in moving frame)");
			this->ports()->addPort(&outCmdVel, "Current commanded velocity (velocity of moving frame relative to reference frame, expressed in moving frame)");
			this->ports()->addPort(&outError, "Status (nonzero means error)");

			this->properties()->addProperty(&propRobot);
		}

		~BaseMonitor()
		{
		}

		bool configureHook()
		{
			if (!devins.fetchInstance(propRobot.get()))
			{
				RTT::log(RTT::Error) << "RobotBase " << propRobot.get() << " not available." << RTT::endlog();
				return false;
			}
			return true;
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
			pos = devins.getDevice()->getMeasuredBasePosition();
			vel = devins.getDevice()->getMeasuredBaseVelocity();
			cmdvel = devins.getDevice()->getCommandedBaseVelocity();
			error = devins.getDevice()->getBaseError();
		}

		void updateHook()
		{
			outPos.Set(pos);
			outVel.Set(vel);
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
		KDL::Twist cmdvel;

		RPI::OutPort<KDL::Frame> outPos;
		RPI::OutPort<KDL::Twist> outVel;
		RPI::OutPort<KDL::Twist> outCmdVel;
		RPI::OutPort<int> outError;
		RPI::Property<std::string> propRobot;

		RPI::DeviceInstanceT<RobotBaseInterface> devins;

		int error;
	};


	class WheelMonitor: public RPI::Module
	{
	public:
		WheelMonitor(std::string name, RPI::Net* net) :
			Module(name, net), outPos("outPos", this), outVel("outVel", this),
					propRobot("Robot", "Name of the robot", ""), propWheel("Wheel", "Number of the wheel", 0), devins(),
					pos(0), vel(0)
		{
			setDescription("Module to monitor the velocity and position of a robot base wheel.");
			this->ports()->addPort(&outPos, "Current position");
			this->ports()->addPort(&outVel, "Current velocity");

			this->properties()->addProperty(&propRobot);
			this->properties()->addProperty(&propWheel);
		}

		~WheelMonitor()
		{
		}

		bool configureHook()
		{
			if (!devins.fetchInstance(propRobot.get()))
			{
				RTT::log(RTT::Error) << "RobotBase " << propRobot.get() << " not available." << RTT::endlog();
				return false;
			}
			if(propWheel.get() < 0 || propWheel.get() > devins->getWheelCount())  {
				RTT::log(RTT::Error) << "RobotBase " << propRobot.get() << " wheel " << propWheel.get() << " not available." << RTT::endlog();
				return false;
			}
			return true;
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
			pos = devins.getDevice()->getWheelPosition(propWheel.get());
			vel = devins.getDevice()->getWheelVelocity(propWheel.get());
		}

		void updateHook()
		{
			outPos.Set(pos);
			outVel.Set(vel);
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

		double pos;
		double vel;

		RPI::OutPort<double> outPos;
		RPI::OutPort<double> outVel;

		RPI::Property<std::string> propRobot;
		RPI::Property<int> propWheel;

		RPI::DeviceInstanceT<RobotBaseInterface> devins;
	};



	class Controller: public RPI::Module
	{
	public:
		Controller(std::string name, RPI::Net* net) :
			Module(name, net), inPos("inPos", this), inVel("inVel", this), inDest("inDest", this),
					outVel("outVel", this), outCompleted("outCompleted", this),
					propVelX("velX", "Maximum X velocity (m/s)", 0.25),
					propVelY("velY", "Maximum Y velocity (m/s)", 0.25),
					propVelYaw("velYaw", "Maximum Rotation velocity (rad/s)", 0.3),
					propPX("pX", "Proportional factor in X direction", 3), propPY("pY", "Proportional factor in Y direction", 3), propPYaw("pYaw", "Proportional factor in yaw direction", 3)
		{
			setDescription("Module to calculate a desired velocity from current position and goal.");
			this->ports()->addPort(&inPos, "Current position (transformation from reference to moving frame)");
			this->ports()->addPort(&inVel, "Current velocity (velocity of moving frame relative to reference frame, expressed in moving frame)");
			this->ports()->addPort(&inDest, "Target position (transformation from reference to goal frame)");
			this->ports()->addPort(&outVel, "Target velocity (velocity of moving frame relative to reference frame, expressed in moving frame)");
			this->ports()->addPort(&outCompleted, "Result (completion)");

			this->properties()->addProperty(&propVelX);
			this->properties()->addProperty(&propVelY);
			this->properties()->addProperty(&propVelYaw);
			this->properties()->addProperty(&propPX);
			this->properties()->addProperty(&propPY);
			this->properties()->addProperty(&propPYaw);
		}

		bool configureHook()
		{
			return true;
		}

		bool startHook()
		{
			return true;
		}

		void updateHook()
		{
			bool completed = false;
			double pX = propPX.get(), pY = propPY.get(), pYaw = propPYaw.get();
			double maxX = propVelX.get(), maxY = propVelY.get(), maxYaw = propVelYaw.get();
			KDL::Frame pos = inPos.Get(), dest = inDest.Get();
			KDL::Twist curvel = inVel.Get();
			double destX = dest.p.x(), posX = pos.p.x(), destY = dest.p.y(), posY = pos.p.y(), destYaw, posYaw, dummy;
			dest.M.GetRPY(dummy, dummy, destYaw);
			pos.M.GetRPY(dummy, dummy, posYaw);
			while(destYaw-posYaw > KDL::PI) destYaw -= 2*KDL::PI;
			while(destYaw-posYaw < -KDL::PI) destYaw += 2*KDL::PI;

			double xvel = (destX - posX) * pX, yvel = (destY - posY) * pY, yawvel = (destYaw - posYaw) * pYaw;
			double fac = 1;
			fac = std::max(fac, fabs(xvel) / maxX);
			fac = std::max(fac, fabs(yvel) / maxY);
			fac = std::max(fac, fabs(yawvel) / maxYaw);
			if (fabs(xvel / pX) < 1e-2 && fabs(yvel / pY) < 1e-2 && fabs(yawvel / pYaw) < 1e-2)
			{
				completed = true;
				xvel = 0;
				yvel = 0;
				yawvel = 0;
			}
			xvel /= fac;
			yvel /= fac;
			yawvel /= fac;

			double xvelLocal = xvel * cos(-posYaw) + yvel * -sin(-posYaw);
			double yvelLocal = xvel * sin(-posYaw) + yvel * cos(-posYaw);

			if(!inVel.isNull()) {
				double curx = curvel.vel.x(), cury = curvel.vel.y(), curyaw = curvel.rot.z();
				double dt = getInNet()->getNetFrequency();

				if(xvelLocal-curx > 0 && (xvelLocal-curx)/dt > maxX)
					xvelLocal = curx + maxX*dt;
				else if(xvelLocal-curx < 0 && (xvelLocal - curx)/dt < -maxX)
					xvelLocal = curx - maxX*dt;

				if(yvelLocal-cury > 0 && (yvelLocal-cury)/dt > maxY)
					yvelLocal = cury + maxY*dt;
				else if(yvelLocal - cury < 0 && (yvelLocal - cury)/dt < -maxY)
					yvelLocal = cury - maxY*dt;

				if(yawvel - curyaw > 0 && (yawvel - curyaw)/dt > maxYaw)
					yawvel = curyaw + maxYaw*dt;
				else if(yawvel - curyaw < 0 && (yawvel - curyaw)/dt < -maxYaw)
					yawvel = curyaw - maxYaw*dt;
			}

			outVel.Set(KDL::Twist(KDL::Vector(xvelLocal, yvelLocal, 0), KDL::Vector(0, 0, yawvel)));
			outCompleted.Set(completed);
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

		RPI::InPort<KDL::Frame> inPos;
		RPI::InPort<KDL::Twist> inVel;
		RPI::InPort<KDL::Frame> inDest;
		RPI::OutPort<KDL::Twist> outVel;
		RPI::OutPort<bool> outCompleted;

		RPI::Property<double> propVelX;
		RPI::Property<double> propVelY;
		RPI::Property<double> propVelYaw;
		RPI::Property<double> propPX;
		RPI::Property<double> propPY;
		RPI::Property<double> propPYaw;

	};


}
