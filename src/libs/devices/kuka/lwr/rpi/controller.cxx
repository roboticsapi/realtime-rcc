/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include <rtt/Logger.hpp>
#include "controller.hpp"

namespace kuka_iiwa
{
	using namespace RPI;
	using namespace std;

	ControlStrategy::ControlStrategy(std::string name, RPI::Net* net) :
			ActiveModule(name, net), inStrategy("inStrategy", this), outCompleted("outCompleted", this), outError(
					"outError", this), propRobot("Robot", "Name of the IIWA", ""), devins()
	{

		setDescription("Module for switching the control mode of the IIWA robot");
		this->ports()->addPort(&inStrategy, "The required control strategy");
		this->ports()->addPort(&outCompleted, "true, when the switching action has completed");
		this->ports()->addPort(&outError, "Result (nonzero means error)");

		this->properties()->addProperty(&propRobot);
	}

	ControlStrategy::~ControlStrategy()
	{
	}

	bool ControlStrategy::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "IIWA " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}

		return true;
	}

	bool ControlStrategy::startHook()
	{
		return true;
	}

	void ControlStrategy::updateHook()
	{
		if (active())
		{
			int error = 0;
			bool used = false;
			for (int i = 0; i < 7; i++)
				used |= *resourceUsed[i];

			if (used)
			{
				error = 1;
			} else
			{
				for (int i = 0; i < 7; i++)
					*resourceUsed[i] = true;

				if (!inStrategy.isNull())
					if (devins.getDevice()->setKRCDesiredControlScheme(inStrategy.Get()))
						devins.getDevice()->setKRCModeFinished(false);

				outCompleted.Set(devins.getDevice()->getKRCModeFinished());

				error = devins.getDevice()->getKRCModeError();

			}
			outError.Set(error);
		}
	}

	void ControlStrategy::stopHook()
	{
	}

	void ControlStrategy::cleanupHook()
	{
	}

	set<string> ControlStrategy::getResourceNames() const
	{
		set<string> ret;
		// we need all axes of the LWR
		for (int i = 0; i < 7; i++)
		{
			stringstream name;
			name << "Iiwa_ControlStrategy_" << propRobot.get() << "_" << i;
			ret.insert(name.str());
		}
		return ret;
	}

	string ControlStrategy::getDeviceName()
	{
		return propRobot.get();
	}

	bool ControlStrategy::isActuator()
	{
		return true;
	}

	void ControlStrategy::updateActuator()
	{
	}

	ToolParameters::ToolParameters(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inTCP("inTCP", this), inMass("inMass", this), inCOM("inCOM", this), inMOI("inMOI",
						this), outCompleted("outCompleted", this), outError("outError", this), propRobot("robot",
						"Name of the IIWA", "")
		{

			setDescription("Module for configuring the tool of IIWA robot");

			this->ports()->addPort(&inTCP, "tool center point and orientation");
			this->ports()->addPort(&inMass, "Mass of load (in kg)");
			this->ports()->addPort(&inCOM, "Center of mass");
			this->ports()->addPort(&inMOI, "Moment of inertia");

			this->ports()->addPort(&outCompleted, "true, when the switching action has completed");
			this->ports()->addPort(&outError, "Result (nonzero means error)");

			this->properties()->addProperty(&propRobot);
		}

		ToolParameters::~ToolParameters()
		{
		}

		bool ToolParameters::configureHook()
		{
			if (!devins.fetchInstance(propRobot.get()))
			{
				RTT::log(RTT::Error) << "IIWA " << propRobot.get() << " not available." << RTT::endlog();
				return false;
			}

			return true;
		}

		bool ToolParameters::startHook()
		{
			return true;
		}

		void ToolParameters::updateHook()
		{
			if (active())
			{
				int error = 0;
				bool used = false;
				for (int i = 0; i < 7; i++)
					used |= *resourceUsed[i];

				if (used)
				{
					error = 1;
				} else
				{
					for (int i = 0; i < 7; i++)
						*resourceUsed[i] = true;

					bool updated = false;

					if (!inTCP.isNull())
						updated |= devins.getDevice()->setKRCToolTCP(inTCP.Get());
					if (!inCOM.isNull())
						updated |= devins.getDevice()->setKRCToolCOM(inCOM.Get());
					if (!inMOI.isNull())
						updated |= devins.getDevice()->setKRCToolMOI(inMOI.Get());
					if (!inMass.isNull())
						updated |= devins.getDevice()->setKRCToolMass(inMass.Get());

					if (updated)
						devins.getDevice()->setKRCModeFinished(false);

					outCompleted.Set(devins.getDevice()->getKRCModeFinished());

					error = devins.getDevice()->getKRCModeError();

				}
				outError.Set(error);
			}
		}

		void ToolParameters::stopHook()
		{
		}

		void ToolParameters::cleanupHook()
		{
		}

		set<string> ToolParameters::getResourceNames() const
		{
			set<string> ret;
			// we need all axes of the LWR
			for (int i = 0; i < 7; i++)
			{
				stringstream name;
				name << "Iiwa_ToolParam_" << propRobot.get() << "_" << i;
				ret.insert(name.str());
			}
			return ret;
		}

		string ToolParameters::getDeviceName()
		{
			return propRobot.get();
		}

		bool ToolParameters::isActuator()
		{
			return true;
		}

		void ToolParameters::updateActuator()
		{

		}

		CartSinImpParameters::CartSinImpParameters(std::string name, RPI::Net* net) :
					ActiveModule(name, net), inAmplitude("inAmplitude", this), inFrequency("inFrequency", this),
					propRobot("Robot", "Name of the IIWA", ""), devins(), hasValues(false)
			{

				setDescription("Module for changing Cartesian amplitude and frequency for the IIWA robot");
				this->ports()->addPort(&inAmplitude, "Amlitude value for selected cartesian");
				this->ports()->addPort(&inFrequency, "Frequency value for selected cartesian");

				this->properties()->addProperty(&propRobot);
			}

			CartSinImpParameters::~CartSinImpParameters()
			{
			}

			bool CartSinImpParameters::configureHook()
			{
				if (!devins.fetchInstance(propRobot.get()))
				{
					RTT::log(RTT::Error) << "IIWA " << propRobot.get() << " not available." << RTT::endlog();
					return false;
				}
				return true;
			}

			bool CartSinImpParameters::startHook()
			{
				return true;
			}

			void CartSinImpParameters::updateHook()
			{
				if (active())
				{
					if (*resourceUsed[0])
					{
						// oops, better not change values here again, another module in this net already did so
					} else
					{
						*resourceUsed[0] = true;
						hasValues = true;
					}
				}
			}

			void CartSinImpParameters::stopHook()
			{
			}

			void CartSinImpParameters::cleanupHook()
			{
			}

			set<string> CartSinImpParameters::getResourceNames() const
			{
				stringstream name;
				set<string> ret;
				name << "Iiwa_CartSinImpParam_" << propRobot.get();
				ret.insert(name.str());
				return ret;
			}

			string CartSinImpParameters::getDeviceName()
			{
				return propRobot.get();
			}

			bool CartSinImpParameters::isActuator()
			{
				return true;
			}

			void CartSinImpParameters::updateActuator()
			{
				if (hasValues)
				{
					hasValues = false;

					if (inAmplitude.connected())
					{
						KDL::Wrench wrench;
						wrench = inAmplitude.Get();
						devins.getDevice()->setCartSinImpAmplitude(wrench.force.x(), wrench.force.y(), wrench.force.z(),
								wrench.torque.z(), wrench.torque.y(), wrench.torque.x());
					}

					if (inFrequency.connected())
					{
						KDL::Wrench wrench;
						wrench = inFrequency.Get();
						devins.getDevice()->setCartSinImpFrequency(wrench.force.x(), wrench.force.y(), wrench.force.z(),
								wrench.torque.z(), wrench.torque.y(), wrench.torque.x());
					}

				}
			}

}
