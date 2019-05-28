/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "../interface/RobotArmInterface.hpp"
#include <rtt/Logger.hpp>
#include <kdl/frames.hpp>
#include <rcc/DeviceInstanceT.hpp>
#include <rcc/Net.hpp>

namespace robotarm
{

	/**
	 * Module for controlling a robot axis using position
	 */
	class JointPosition: public RPI::ActiveModule
	{

	public:
		JointPosition(std::string name, RPI::Net* net) :
				RPI::ActiveModule(name, net),
				inPosition("inPosition", this),
				outErrorConcurrentAccess("outErrorConcurrentAccess", this),
				outErrorJointFailed("outErrorJointFailed", this),
				outErrorIllegalPosition("outErrorIllegalPosition", this),
				outError("outError", this),
				propRobot("Robot", "Name of the robot", ""),
				propAxis("Axis", "Number of axis to control (0-based)"),
				devins(), hasValues(false), axis(0), jointPos(0), time(0), jointError(0)
		{
			setDescription("Module for controlling a robot axis using position");
			this->ports()->addPort(&inPosition, "Destination value for selected axis");
			this->ports()->addPort(&outError, "Result (nonzero means error)");
			this->ports()->addPort(&outErrorConcurrentAccess, "Concurrent access error (joint is used by another primitive)");
			this->ports()->addPort(&outErrorIllegalPosition, "Illegal joint position (the value of inPosition is invalid)");
			this->ports()->addPort(&outErrorJointFailed, "Joint error (the joint is not operational)");
			this->properties()->addProperty(&propRobot);
			this->properties()->addProperty(&propAxis);
		}

		~JointPosition()
		{
		}

		bool configureHook()
		{
			if (!devins.fetchInstance(propRobot.get()))
			{
				RTT::log(RTT::Error) << "Robot " << propRobot.get() << " not available." << RTT::endlog();
				return false;
			}

			axis = propAxis.get();
			if (axis < 0 || axis >= devins.getDevice()->getJointCount())
			{
				RTT::log(RTT::Error) << "Robot " << propRobot.get() << ": invalid axis: " << propAxis.get()
						<< RTT::endlog();
				return false;
			}

			return true;
		}

		bool startHook()
		{
			return true;
		}

		void stopHook()
		{
		}

		void cleanupHook()
		{
		}

		std::set<std::string> getResourceNames() const
		{
			std::set<std::string> ret;
			std::stringstream name, name2;
			name << propRobot.get() << "_Joint_" << propAxis.get();
			ret.insert(name.str());
			name2 << propRobot.get() << "_ToolParam_" << propAxis.get();
			ret.insert(name2.str());
			return ret;
		}

		std::string getDeviceName()
		{
			return propRobot.get();
		}

		bool isSensor()
		{
			return true;
		}

		bool isActuator()
		{
			return true;
		}

		void updateSensor()
		{
			jointError = devins.getDevice()->getJointError(axis);
		}

		void updateHook()
		{
			if (active())
			{
				time = inNet->getTime();
				jointPos = (float) inPosition.Get();
				hasValues = false;

				robotarm::JointPositionError positionError = devins.getDevice()->checkJointPosition(axis, jointPos);
				int accessError = *resourceUsed[0] ? -1 : 0;

				outErrorJointFailed.Set(jointError != robotarm::JP_OK);
				outErrorConcurrentAccess.Set(accessError != 0);
				outErrorIllegalPosition.Set(positionError != 0);

				*resourceUsed[0] = true;

				if (accessError != 0)
				{
					outError.Set(accessError);
				}
				else if (jointError != 0)
				{
					outError.Set(jointError);
				}
				else if (positionError != 0)
				{
					outError.Set(positionError);
				}
				else
				{
					hasValues = true;
					outError.Set(0);
				}
			}
		}

		void updateActuator()
		{
			if (hasValues)
			{
				devins.getDevice()->setJointPosition(axis, jointPos, time);
				hasValues = false;
			}
		}

	protected:
		RPI::InPort<double> inPosition;
		RPI::OutPort<bool> outErrorConcurrentAccess;
		RPI::OutPort<bool> outErrorJointFailed;
		RPI::OutPort<bool> outErrorIllegalPosition;
		RPI::OutPort<int> outError;
		RPI::Property<std::string> propRobot;
		RPI::Property<int> propAxis;

	private:
		RPI::DeviceInstanceT<RobotArmInterface> devins;
		int axis;
		int jointError;
		bool hasValues;
		float jointPos;
		RTT::os::TimeService::nsecs time;
	};

	/**
	 * Monitor reading last commanded and measured positions from a robot
	 */
	class JointMonitor: public RPI::Module
	{

	public:
		JointMonitor(std::string name, RPI::Net* net) :
				RPI::Module(name, net), outCmdPos("outCommandedPosition", this), outMsrPos("outMeasuredPosition", this), outCmdVel(
						"outCommandedVelocity", this), outMsrVel("outMeasuredVelocity", this), outCmdAcc(
						"outCommandedAcceleration", this), outMsrAcc("outMeasuredAcceleration", this), outError("outError", this), propRobot(
						"Robot", "Name of the LBR", ""), propAxis("Axis", "Number of axis to control (0-based)"), devins(), cmdpos(
						0), cmdvel(0), cmdacc(0), msrpos(0), msrvel(0), msracc(0), error(0)
		{
			setDescription("Monitor reading last commanded and measured positions from a robot");
			this->ports()->addPort(&outCmdPos, "Last commanded position of selected axis");
			this->ports()->addPort(&outMsrPos, "Last measured position of selected axis");
			this->ports()->addPort(&outCmdVel, "Last commanded velocity of selected axis");
			this->ports()->addPort(&outMsrVel, "Last measured velocity of selected axis");
			this->ports()->addPort(&outCmdAcc, "Last commanded acceleration of selected axis");
			this->ports()->addPort(&outMsrAcc, "Last measured acceleration of selected axis");
			this->ports()->addPort(&outError, "Status (nonzero means error)");

			this->properties()->addProperty(&propRobot);
			this->properties()->addProperty(&propAxis);

		}

		~JointMonitor()
		{
		}

		bool configureHook()
		{
			if (!devins.fetchInstance(propRobot.get()))
			{
				RTT::log(RTT::Error) << "Robot " << propRobot.get() << " not available." << RTT::endlog();
				return false;
			}
			if (propAxis.get() < 0 || propAxis.get() >= devins.getDevice()->getJointCount())
			{
				RTT::log(RTT::Error) << "Robot " << propRobot.get() << ": invalid axis: " << propAxis.get()
						<< RTT::endlog();
				return false;
			}

			return true;
		}

		bool startHook()
		{
			return true;
		}

		void stopHook()
		{
		}

		void cleanupHook()
		{
		}

		std::string getDeviceName()
		{
			return propRobot.get();
		}

		bool isSensor()
		{
			return true;
		}

		void updateSensor()
		{
			int axis = propAxis.get();
			cmdpos = devins.getDevice()->getCommandedJointPosition(axis);
			cmdvel = devins.getDevice()->getCommandedJointVelocity(axis);
			cmdacc = devins.getDevice()->getCommandedJointAcceleration(axis);
			msrpos = devins.getDevice()->getMeasuredJointPosition(axis);
			msrvel = devins.getDevice()->getMeasuredJointVelocity(axis);
			msracc = devins.getDevice()->getMeasuredJointAcceleration(axis);
			error = devins.getDevice()->getJointError(axis);
		}

		void updateHook()
		{
			outCmdPos.Set(cmdpos);
			outCmdVel.Set(cmdvel);
			outCmdAcc.Set(cmdacc);
			outMsrPos.Set(msrpos);
			outMsrVel.Set(msrvel);
			outMsrAcc.Set(msracc);
			outError.Set(error);
		}

	protected:
		RPI::OutPort<double> outCmdPos, outMsrPos, outCmdVel, outMsrVel, outCmdAcc, outMsrAcc;
		RPI::OutPort<int> outError;
		RPI::Property<std::string> propRobot;
		RPI::Property<int> propAxis;
	private:
		RPI::DeviceInstanceT<RobotArmInterface> devins;
		double cmdpos, cmdvel, cmdacc, msrpos, msrvel, msracc;
		int error;

	};

	/**
	 * Module for configuring the tool of a robot
	 */
	class ToolParameters: public RPI::ActiveModule
	{
	public:
		ToolParameters(std::string name, RPI::Net* net) :
				ActiveModule(name, net), outCompleted("outCompleted", this), outError("outError", this),
				inCOM("inCOM",this),inMOI("inMOI",this),inMass("inMass",this),propRobot(
						"Robot", "Name of the Robot", ""), propAxis("Axis", "Axis to set payload for (0 based)", 0), propMass(
						"Mass", "Payload mass", 0), propCOM("COM", "Center of mass", KDL::Vector()),propMOI("MOI", "Moment of inertia", KDL::Vector()),
						massVal(std::numeric_limits<double>::quiet_NaN())
		{

			setDescription("Module for configuring the tool of a robot");

			this->ports()->addPort(&outCompleted, "true, when the switching action has completed");
			this->ports()->addPort(&outError, "Result (nonzero means jointError)");
			this->ports()->addPort(&inCOM,"Tool's center of mass");
			this->ports()->addPort(&inMOI,"Tool's moment of inertia");
			this->ports()->addPort(&inMass,"Tool's mass");

			this->properties()->addProperty(&propRobot);
			this->properties()->addProperty(&propAxis);
			this->properties()->addProperty(&propMass);
			this->properties()->addProperty(&propCOM);
			this->properties()->addProperty(&propMOI);

			finished = false;
			changedValues = false;
			error = 0;
		}

		~ToolParameters()
		{
		}

		bool configureHook()
		{
			if (!devins.fetchInstance(propRobot.get()))
			{
				RTT::log(RTT::Error) << "Robot " << propRobot.get() << " not available." << RTT::endlog();
				return false;
			}

			return true;
		}

		bool startHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				bool used = false;
				for (int i = 0; i < resourceCount; i++)
					used |= *resourceUsed[i];

				if (used)
				{
					error = 1;
				} else
				{
					for (int i = 0; i < resourceCount; i++)
						*resourceUsed[i] = true;

					if (moiVal != inMOI.Get(propMOI) || comVal != inCOM.Get(propCOM) || massVal != inMass.Get(propMass))
					{
						changedValues = true;
						moiVal = inMOI.Get(propMOI);
						comVal = inCOM.Get(propCOM);
						massVal = inMass.Get(propMass);
					}
					outCompleted.Set(finished);
				}
				outError.Set(error);
			}
		}

		void stopHook()
		{
		}

		void cleanupHook()
		{
		}

		std::set<std::string> getResourceNames() const
		{
			std::set<std::string> ret;
			std::stringstream name;
			name << propRobot.get() << "_ToolParam_" << propAxis.get();
			ret.insert(name.str());
			return ret;
		}

		std::string getDeviceName()
		{
			return propRobot.get();
		}

		bool isActuator()
		{
			return true;
		}

		void updateActuator()
		{
			if (changedValues) {

				changedValues = false;
				devins.getDevice()->setToolCOM(comVal, propAxis.get());
				devins.getDevice()->setToolMOI(moiVal, propAxis.get());
				devins.getDevice()->setToolMass(massVal, propAxis.get());
			}
			error = devins.getDevice()->getToolError(propAxis.get());
			finished = devins.getDevice()->getToolFinished(propAxis.get());
		}

	protected:
		RPI::OutPort<bool> outCompleted;
		RPI::OutPort<int> outError;
		RPI::InPort<double> inMass;
		RPI::InPort<KDL::Vector> inCOM;
		RPI::InPort<KDL::Vector> inMOI;

		RPI::Property<double> propMass; ///< mass of load (in kg)
		RPI::Property<KDL::Vector> propCOM; ///< coordinates of center of mass in flange coordinate system
		RPI::Property<KDL::Vector> propMOI;
		RPI::Property<std::string> propRobot;
		RPI::Property<int> propAxis;

	private:
		RPI::DeviceInstanceT<RobotArmInterface> devins;
		bool finished, changedValues;
		KDL::Vector comVal,moiVal;
		double massVal;
		int error;
	};

}
