/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include <rtt/Logger.hpp>
#include "Lbr.hpp"
#include "../kinematics/Lbr_Kin.hpp"

namespace kuka_lwr
{
	using namespace RPI;
	using namespace std;

	ControlStrategy::ControlStrategy(std::string name, RPI::Net* net) :
			ActiveModule(name, net), inStrategy("inStrategy", this), outCompleted("outCompleted", this), outError(
					"outError", this), propRobot("Robot", "Name of the LBR", ""), devins()
	{

		setDescription("Module for switching the control mode of the LWR robot");
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
			RTT::log(RTT::Error) << "LBR " << propRobot.get() << " not available." << RTT::endlog();
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
			name << "Lbr_ControlStrategy_" << propRobot.get() << "_" << i;
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
					"Name of the LBR", "")
	{

		setDescription("Module for configuring the tool of LWR robot");

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
			RTT::log(RTT::Error) << "LBR " << propRobot.get() << " not available." << RTT::endlog();
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
			name << "Lbr_ToolParam_" << propRobot.get() << "_" << i;
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

	JointImpParameters::JointImpParameters(std::string name, RPI::Net* net) :
			ActiveModule(name, net), inStiffness("inStiffness", this), inDamping("inDamping", this), inAddTorque(
					"inAddTorque", this), propRobot("Robot", "Name of the LBR", ""), propAxis("Axis",
					"Number of axis to control (0-based)"), devins(), hasValues(false)
	{

		setDescription("Module for changing joint impedance control values (stiffness etc.)");
		this->ports()->addPort(&inStiffness, "Stiffness value for selected axis");
		this->ports()->addPort(&inDamping, "Damping value for selected axis");
		this->ports()->addPort(&inAddTorque, "Additional torque for selected axis");

		this->properties()->addProperty(&propRobot);
		this->properties()->addProperty(&propAxis);
	}

	JointImpParameters::~JointImpParameters()
	{
	}

	bool JointImpParameters::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}
		if (propAxis.get() < 0 || propAxis.get() > 6)
		{
			RTT::log(RTT::Error) << "LBR " << propRobot.get() << ": invalid axis: " << propAxis.get() << RTT::endlog();
			return false;
		}
		return true;
	}

	bool JointImpParameters::startHook()
	{
		return true;
	}

	void JointImpParameters::updateHook()
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

	void JointImpParameters::stopHook()
	{
	}

	void JointImpParameters::cleanupHook()
	{
	}

	set<string> JointImpParameters::getResourceNames() const
	{
		stringstream name;
		set<string> ret;
		name << "Lbr_JntParam_" << propRobot.get() << "_" << propAxis.get();
		ret.insert(name.str());
		return ret;
	}

	string JointImpParameters::getDeviceName()
	{
		return propRobot.get();
	}

	bool JointImpParameters::isActuator()
	{
		return true;
	}

	void JointImpParameters::updateActuator()
	{
		if (hasValues)
		{
			if (inStiffness.connected())
				devins.getDevice()->setJntImpStiffness(inStiffness.Get(), propAxis.get());
			if (inDamping.connected())
				devins.getDevice()->setJntImpDamping(inDamping.Get(), propAxis.get());
			if (inAddTorque.connected())
				devins.getDevice()->setJntImpAddTorque(inAddTorque.Get(), propAxis.get());
			hasValues = false;
		}
	}

	CartImpParameters::CartImpParameters(std::string name, RPI::Net* net) :
			ActiveModule(name, net), inStiffness("inStiffness", this), inDamping("inDamping", this), inAddTorque(
					"inAddTorque", this), propRobot("Robot", "Name of the LBR", ""), devins(), hasValues(false)
	{

		setDescription("Module for changing Cartesian impedance values (stiffness etc.)");
		this->ports()->addPort(&inStiffness, "Stiffness value for selected axis");
		this->ports()->addPort(&inDamping, "Damping value for selected axis");
		this->ports()->addPort(&inAddTorque, "Additional torque for selected axis");

		this->properties()->addProperty(&propRobot);
	}

	CartImpParameters::~CartImpParameters()
	{
	}

	bool CartImpParameters::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool CartImpParameters::startHook()
	{
		return true;
	}

	void CartImpParameters::updateHook()
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

	void CartImpParameters::stopHook()
	{
	}

	void CartImpParameters::cleanupHook()
	{
	}

	set<string> CartImpParameters::getResourceNames() const
	{
		stringstream name;
		set<string> ret;
		name << "Lbr_CartParam_" << propRobot.get();
		ret.insert(name.str());
		return ret;
	}

	string CartImpParameters::getDeviceName()
	{
		return propRobot.get();
	}

	bool CartImpParameters::isActuator()
	{
		return true;
	}

	void CartImpParameters::updateActuator()
	{
		if (hasValues)
		{
			hasValues = false;

			if (inStiffness.connected())
			{
				KDL::Wrench wrench;
				wrench = inStiffness.Get();
				devins.getDevice()->setCartImpStiffness(wrench.force.x(), wrench.force.y(), wrench.force.z(),
						wrench.torque.z(), wrench.torque.y(), wrench.torque.x());
			}

			if (inDamping.connected())
			{
				KDL::Wrench wrench;
				wrench = inDamping.Get();
				devins.getDevice()->setCartImpDamping(wrench.force.x(), wrench.force.y(), wrench.force.z(),
						wrench.torque.z(), wrench.torque.y(), wrench.torque.x());
			}
			if (inAddTorque.connected())
			{
				KDL::Wrench wrench;
				wrench = inAddTorque.Get();
				devins.getDevice()->setCartImpAddTcpFT(wrench.force.x(), wrench.force.y(), wrench.force.z(),
						wrench.torque.z(), wrench.torque.y(), wrench.torque.x());
			}
		}
	}

	FTMonitor::FTMonitor(std::string name, RPI::Net* net) :
			Module(name, net), outMsrJ1("outMsrJ1", this), outMsrJ2("outMsrJ2", this), outMsrJ3("outMsrJ3", this), outMsrJ4(
					"outMsrJ4", this), outMsrJ5("outMsrJ5", this), outMsrJ6("outMsrJ6", this), outMsrJ7("outMsrJ7",
					this), outEstJ1("outEstJ1", this), outEstJ2("outEstJ2", this), outEstJ3("outEstJ3", this), outEstJ4(
					"outEstJ4", this), outEstJ5("outEstJ5", this), outEstJ6("outEstJ6", this), outEstJ7("outEstJ7",
					this), outTcpFx("outTcpFx", this), outTcpFy("outTcpFy", this), outTcpFz("outTcpFz", this), outTcpTz(
					"outTcpTz", this), outTcpTy("outTcpTy", this), outTcpTx("outTcpTx", this), propRobot("Robot",
					"Name of the LBR", ""), devins()
	{
		setDescription("Force/Torque Sensor for LBR robot");

		this->ports()->addPort(&outMsrJ1, "Measured torque of axis 1");
		this->ports()->addPort(&outMsrJ2, "Measured torque of axis 2");
		this->ports()->addPort(&outMsrJ3, "Measured torque of axis 3");
		this->ports()->addPort(&outMsrJ4, "Measured torque of axis 4");
		this->ports()->addPort(&outMsrJ5, "Measured torque of axis 5");
		this->ports()->addPort(&outMsrJ6, "Measured torque of axis 6");
		this->ports()->addPort(&outMsrJ7, "Measured torque of axis 7");
		this->ports()->addPort(&outEstJ1, "Estimated torque of axis 1");
		this->ports()->addPort(&outEstJ2, "Estimated torque of axis 2");
		this->ports()->addPort(&outEstJ3, "Estimated torque of axis 3");
		this->ports()->addPort(&outEstJ4, "Estimated torque of axis 4");
		this->ports()->addPort(&outEstJ5, "Estimated torque of axis 5");
		this->ports()->addPort(&outEstJ6, "Estimated torque of axis 6");
		this->ports()->addPort(&outEstJ7, "Estimated torque of axis 7");
		this->ports()->addPort(&outTcpFx, "Estimated force in X direction");
		this->ports()->addPort(&outTcpFy, "Estimated force in Y direction");
		this->ports()->addPort(&outTcpFz, "Estimated force in Z direction");
		this->ports()->addPort(&outTcpTz, "Estimated torque around Z direction");
		this->ports()->addPort(&outTcpTy, "Estimated torque around Y direction");
		this->ports()->addPort(&outTcpTx, "Estimated torque around X direction");
		this->properties()->addProperty(&propRobot);
	}

	FTMonitor::~FTMonitor()
	{
	}

	bool FTMonitor::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool FTMonitor::startHook()
	{
		return true;
	}

	void FTMonitor::updateHook()
	{

	}

	void FTMonitor::stopHook()
	{
	}

	void FTMonitor::cleanupHook()
	{
	}

	string FTMonitor::getDeviceName()
	{
		return propRobot.get();
	}

	bool FTMonitor::isSensor()
	{
		return true;
	}

	void FTMonitor::updateSensor()
	{
		float* tcpEst = devins.getDevice()->getTcpEst();
		float a[7];
		float e[7];
		for (int i = 0; i < 7; i++)
		{
			a[i] = devins.getDevice()->getAxisTqAct(i);
			e[i] = devins.getDevice()->getAxisTqEst(i);
		}
		outMsrJ1.Set(a[0]);
		outMsrJ2.Set(a[1]);
		outMsrJ3.Set(a[2]);
		outMsrJ4.Set(a[3]);
		outMsrJ5.Set(a[4]);
		outMsrJ6.Set(a[5]);
		outMsrJ7.Set(a[6]);
		outEstJ1.Set(e[0]);
		outEstJ2.Set(e[1]);
		outEstJ3.Set(e[2]);
		outEstJ4.Set(e[3]);
		outEstJ5.Set(e[4]);
		outEstJ6.Set(e[5]);
		outEstJ7.Set(e[6]);
		outTcpFx.Set(tcpEst[0]);
		outTcpFy.Set(tcpEst[1]);
		outTcpFz.Set(tcpEst[2]);
		outTcpTz.Set(tcpEst[3]);
		outTcpTy.Set(tcpEst[4]);
		outTcpTx.Set(tcpEst[5]);
	}

	FMonitor::FMonitor(std::string name, RPI::Net* net) :
			Module(name, net), outTcpFx("outTcpFx", this), outTcpFy("outTcpFy", this), outTcpFz("outTcpFz", this), outTcpTz(
					"outTcpTz", this), outTcpTy("outTcpTy", this), outTcpTx("outTcpTx", this), propRobot("Robot",
					"Name of the LBR", ""), devins()
	{
		setDescription("Force Sensor for LBR robot");

		this->ports()->addPort(&outTcpFx, "Estimated force in X direction");
		this->ports()->addPort(&outTcpFy, "Estimated force in Y direction");
		this->ports()->addPort(&outTcpFz, "Estimated force in Z direction");
		this->ports()->addPort(&outTcpTz, "Estimated torque around Z direction");
		this->ports()->addPort(&outTcpTy, "Estimated torque around Y direction");
		this->ports()->addPort(&outTcpTx, "Estimated torque around X direction");
		this->properties()->addProperty(&propRobot);
	}

	FMonitor::~FMonitor()
	{
	}

	bool FMonitor::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool FMonitor::startHook()
	{
		return true;
	}

	void FMonitor::updateHook()
	{

	}

	void FMonitor::stopHook()
	{
	}

	void FMonitor::cleanupHook()
	{
	}

	string FMonitor::getDeviceName()
	{
		return propRobot.get();
	}

	bool FMonitor::isSensor()
	{
		return true;
	}

	void FMonitor::updateSensor()
	{
		float* tcpEst = devins.getDevice()->getTcpEst();

		outTcpFx.Set(tcpEst[0]);
		outTcpFy.Set(tcpEst[1]);
		outTcpFz.Set(tcpEst[2]);
		outTcpTz.Set(tcpEst[3]);
		outTcpTy.Set(tcpEst[4]);
		outTcpTx.Set(tcpEst[5]);
	}

	TMonitor::TMonitor(std::string name, RPI::Net* net) :
			Module(name, net), outMsr("outMsr", this), outEst("outEst", this), propRobot("Robot", "Name of the LBR",
					""), propAxis("Axis", "Axis of LWR for Torque", 0), devins()
	{
		setDescription("Torque Sensor for LBR robot");

		this->ports()->addPort(&outMsr, "Measured torque of axis");
		this->ports()->addPort(&outEst, "Estimated torque of axis");
		this->properties()->addProperty(&propRobot);
		this->properties()->addProperty(&propAxis);
	}

	TMonitor::~TMonitor()
	{
	}

	bool TMonitor::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}

		if (propAxis.get() < 0 || propAxis.get() > 6)
		{
			RTT::log(RTT::Error) << "Axis " << propAxis.get() << " invalid." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool TMonitor::startHook()
	{
		return true;
	}

	void TMonitor::updateHook()
	{

	}

	void TMonitor::stopHook()
	{
	}

	void TMonitor::cleanupHook()
	{
	}

	string TMonitor::getDeviceName()
	{
		return propRobot.get();
	}

	bool TMonitor::isSensor()
	{
		return true;
	}

	void TMonitor::updateSensor()
	{
		outMsr.Set(devins.getDevice()->getAxisTqAct(propAxis.get()));
		outEst.Set(devins.getDevice()->getAxisTqEst(propAxis.get()));
	}

	InvKin::InvKin(std::string name, Net* net) :
			ActiveModule(name, net), inFrame("inFrame", this), inHintJ1("inHintJ1", this), inHintJ2("inHintJ2", this), inHintJ3(
					"inHintJ3", this), inHintJ4("inHintJ4", this), inHintJ5("inHintJ5", this), inHintJ6("inHintJ6",
					this), inHintJ7("inHintJ7", this), inNullspaceJ1("inNullspaceJ1", this), inNullspaceJ2(
					"inNullspaceJ2", this), inNullspaceJ3("inNullspaceJ3", this), inNullspaceJ4("inNullspaceJ4", this), inNullspaceJ5(
					"inNullspaceJ5", this), inNullspaceJ6("inNullspaceJ6", this), inNullspaceJ7("inNullspaceJ7", this), outJ1(
					"outJ1", this), outJ2("outJ2", this), outJ3("outJ3", this), outJ4("outJ4", this), outJ5("outJ5",
					this), outJ6("outJ6", this), outJ7("outJ7", this), propStrategy("Strategy", "Redundancy strategy",
					1), devins(), propRobot("Robot", "Name of the LBR","")
	{
		setDescription("Inverse kinematics module for LBR robots");
		this->ports()->addPort(&inHintJ1, "Hint joint value 1");
		this->ports()->addPort(&inHintJ2, "Hint joint value 2");
		this->ports()->addPort(&inHintJ3, "Hint joint value 3");
		this->ports()->addPort(&inHintJ4, "Hint joint value 4");
		this->ports()->addPort(&inHintJ5, "Hint joint value 5");
		this->ports()->addPort(&inHintJ6, "Hint joint value 6");
		this->ports()->addPort(&inHintJ7, "Hint joint value 7");
		this->ports()->addPort(&inNullspaceJ1, "Nullspace joint value 1");
		this->ports()->addPort(&inNullspaceJ2, "Nullspace joint value 2");
		this->ports()->addPort(&inNullspaceJ3, "Nullspace joint value 3");
		this->ports()->addPort(&inNullspaceJ4, "Nullspace joint value 4");
		this->ports()->addPort(&inNullspaceJ5, "Nullspace joint value 5");
		this->ports()->addPort(&inNullspaceJ6, "Nullspace joint value 6");
		this->ports()->addPort(&inNullspaceJ7, "Nullspace joint value 7");
		this->ports()->addPort(&inFrame, "Destination frame");
		this->ports()->addPort(&outJ1, "Result angle for joint 1");
		this->ports()->addPort(&outJ2, "Result angle for joint 2");
		this->ports()->addPort(&outJ3, "Result angle for joint 3");
		this->ports()->addPort(&outJ4, "Result angle for joint 4");
		this->ports()->addPort(&outJ5, "Result angle for joint 5");
		this->ports()->addPort(&outJ6, "Result angle for joint 6");
		this->ports()->addPort(&outJ7, "Result angle for joint 7");

		this->properties()->addProperty(&propStrategy);
		this->properties()->addProperty(&propRobot);
	}
	/**
	 * This function is for the configuration code.
	 * Return false to abort configuration.
	 */
	bool InvKin::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}
		if (!inFrame.connected())
			return false;
		if (!inHintJ1.connected())
			return false;
		if (!inHintJ2.connected())
			return false;
		if (!inHintJ3.connected())
			return false;
		if (!inHintJ4.connected())
			return false;
		if (!inHintJ5.connected())
			return false;
		if (!inHintJ6.connected())
			return false;
		if (!inHintJ7.connected())
			return false;
		return true;
	}

	/**
	 * This function is for the application's startup code.
	 * Return false to abort startup.
	 */
	bool InvKin::startHook()
	{
		return true;
	}
	/**
	 * This function is periodically called.
	 */
	void InvKin::updateHook()
	{
		if (active())
		{
			double j0 = inHintJ1.Get(), j1 = inHintJ2.Get(), j2 = inHintJ3.Get(), j3 = inHintJ4.Get(), j4 =
					inHintJ5.Get(), j5 = inHintJ6.Get(), j6 = inHintJ7.Get();
			int strategy = propStrategy.get();

			double r0 = j0, r1 = j1, r2 = j2, r3 = j3, r4 = j4, r5 = j5, r6 = j6;
			KDL::Frame pos;
			double al;
			double n0 = inNullspaceJ1.Get(r0), n1 = inNullspaceJ2.Get(r1), n2 = inNullspaceJ3.Get(r2), n3 =
					inNullspaceJ4.Get(r3), n4 = inNullspaceJ5.Get(r4), n5 = inNullspaceJ6.Get(r5), n6 =
					inNullspaceJ7.Get(r6);
			devins->alphaKin(n0, n1, n2, n3, n4, n5, n6, pos, al);
			//Lbr_Kin::lbrKin(n0, n1, n2, n3, n4, n5, n6, pos, al, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7);
			pos = inFrame.Get();
			devins->alphaInvKin(pos, al, r0, r1, r2, r3, r4, r5, r6);
			//Lbr_Kin::lbrInvKin(pos, al, r0, r1, r2, r3, r4, r5, r6, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7);
			if (strategy == 1)
			{
				double step = 0.001, PI = 4 * atan(1.0), lastal = al;
				for (int i = 0; i < 5; i++)
				{
					double a0 = j0, a1 = j1, a2 = j2, a3 = j3, a4 = j4, a5 = j5, a6 = j6;
					//Lbr_Kin::lbrInvKin(pos, al + step, a0, a1, a2, a3, a4, a5, a6, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7);
					devins->alphaInvKin(pos, al + step, a0, a1, a2, a3, a4, a5, a6);
					double d0 = r0 - a0, d1 = r1 - a1, d2 = r2 - a2, d3 = r3 - a3, d4 = r4 - a4, d5 = r5 - a5, d6 = r6
							- a6;
					double c0 = j0 - r0, c1 = j1 - r1, c2 = j2 - r2, c3 = j3 - r3, c4 = j4 - r4, c5 = j5 - r5, c6 = j6
							- r6;
					double dist = (c0 * d0 + c1 * d1 + c2 * d2 + c3 * d3 + c4 * d4 + c5 * d5 + c6 * d6)
							/ (d0 * d0 + d1 * d1 + d2 * d2 + d3 * d3 + d4 * d4 + d5 * d5 + d6 * d6);
					al = al - step * dist;
					step = step * 0.5;
					if (al > PI)
						al -= 2 * PI;
					if (al < -PI)
						al += 2 * PI;
					double b0 = j0, b1 = j1, b2 = j2, b3 = j3, b4 = j4, b5 = j5, b6 = j6;
					//Lbr_Kin::lbrInvKin(pos, al, b0, b1, b2, b3, b4, b5, b6, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7);
					devins->alphaInvKin(pos, al, b0, b1, b2, b3, b4, b5, b6);
					if ((a0 - j0) * (a0 - j0) + (a1 - j1) * (a1 - j1) + (a2 - j2) * (a2 - j2) + (a3 - j3) * (a3 - j3)
							+ (a4 - j4) * (a4 - j4) + (a5 - j5) * (a5 - j5) + (a6 - j6) * (a6 - j6)
							< (b0 - j0) * (b0 - j0) + (b1 - j1) * (b1 - j1) + (b2 - j2) * (b2 - j2)
									+ (b3 - j3) * (b3 - j3) + (b4 - j4) * (b4 - j4) + (b5 - j5) * (b5 - j5)
									+ (b6 - j6) * (b6 - j6))
					{
						al = lastal;
					} else
					{
						r0 = b0, r1 = b1, r2 = b2, r3 = b3, r4 = b4, r5 = b5, r6 = b6, lastal = al;
					}
				}
			}
			outJ1.Set(r0);
			outJ2.Set(r1);
			outJ3.Set(r2);
			outJ4.Set(r3);
			outJ5.Set(r4);
			outJ6.Set(r5);
			outJ7.Set(r6);
		}
	}

	/**
	 * This function is called when the task is stopped.
	 */
	void InvKin::stopHook()
	{
	}

	/**
	 * This function is called when the task is being deconfigured.
	 */
	void InvKin::cleanupHook()
	{
	}

	Kin::Kin(std::string name, Net* net) :
			ActiveModule(name, net), inJ1("inJ1", this), inJ2("inJ2", this), inJ3("inJ3", this), inJ4("inJ4", this), inJ5(
					"inJ5", this), inJ6("inJ6", this), inJ7("inJ7", this), outFrame("outFrame", this), outAlpha(
					"outAlpha", this), devins(), propRobot("Robot", "Name of the LBR","")
	{
		setDescription("Direct kinematics module for LWR robots");
		this->ports()->addPort(&inJ1, "Value of Axis 1");
		this->ports()->addPort(&inJ2, "Value of Axis 2");
		this->ports()->addPort(&inJ3, "Value of Axis 3");
		this->ports()->addPort(&inJ4, "Value of Axis 4");
		this->ports()->addPort(&inJ5, "Value of Axis 5");
		this->ports()->addPort(&inJ6, "Value of Axis 6");
		this->ports()->addPort(&inJ7, "Value of Axis 7");
		this->ports()->addPort(&outFrame, "Resulting frame");
		this->ports()->addPort(&outAlpha, "Resulting alpha value");
		this->properties()->addProperty(&propRobot);
	}

	bool Kin::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}
		if (!inJ1.connected() || !inJ2.connected() || !inJ3.connected() || !inJ4.connected() || !inJ5.connected()
				|| !inJ6.connected() || !inJ7.connected())
			return false;
		return true;
	}

	bool Kin::startHook()
	{
		return true;
	}

	void Kin::updateHook()
	{
		if (active())
		{
			KDL::Frame pos;
			double alpha;
			/*Lbr_Kin::lbrKin(inJ1.Get(), inJ2.Get(), inJ3.Get(), inJ4.Get(), inJ5.Get(), inJ6.Get(), inJ7.Get(), pos,
					alpha, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7);*/
			devins->alphaKin(inJ1.Get(), inJ2.Get(), inJ3.Get(), inJ4.Get(), inJ5.Get(), inJ6.Get(), inJ7.Get(), pos, alpha);
			outFrame.Set(pos);
			outAlpha.Set(alpha);
		}
	}

	void Kin::stopHook()
	{
		// Your stop code after last updateHook()
	}

	void Kin::cleanupHook()
	{
		// Your configuration cleanup code
	}

	VelKin::VelKin(std::string name, Net* net) :
			ActiveModule(name, net), inJ1("inJ1", this), inJ2("inJ2", this), inJ3("inJ3", this), inJ4("inJ4", this), inJ5(
					"inJ5", this), inJ6("inJ6", this), inJ7("inJ7", this), inV1("inV1", this), inV2("inV2", this), inV3(
					"inV3", this), inV4("inV4", this), inV5("inV5", this), inV6("inV6", this), inV7("inV7", this), outTwist(
					"outTwist", this), outVAlpha("outVAlpha", this), devins(), propRobot("Robot", "Name of the LBR","")
	{
		setDescription("Direct velocity kinematics module for LWR robots");
		this->ports()->addPort(&inJ1, "Position of Axis 1");
		this->ports()->addPort(&inJ2, "Position of Axis 2");
		this->ports()->addPort(&inJ3, "Position of Axis 3");
		this->ports()->addPort(&inJ4, "Position of Axis 4");
		this->ports()->addPort(&inJ5, "Position of Axis 5");
		this->ports()->addPort(&inJ6, "Position of Axis 6");
		this->ports()->addPort(&inJ7, "Position of Axis 7");
		this->ports()->addPort(&inV1, "Velocity of Axis 1");
		this->ports()->addPort(&inV2, "Velocity of Axis 2");
		this->ports()->addPort(&inV3, "Velocity of Axis 3");
		this->ports()->addPort(&inV4, "Velocity of Axis 4");
		this->ports()->addPort(&inV5, "Velocity of Axis 5");
		this->ports()->addPort(&inV6, "Velocity of Axis 6");
		this->ports()->addPort(&inV7, "Velocity of Axis 7");
		this->ports()->addPort(&outTwist, "Resulting Twist");
		this->ports()->addPort(&outVAlpha, "Resulting alpha velocity");
		this->properties()->addProperty(&propRobot);
	}

	bool VelKin::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}
		if (!inJ1.connected() || !inJ2.connected() || !inJ3.connected() || !inJ4.connected() || !inJ5.connected()
				|| !inJ6.connected() || !inJ7.connected())
			return false;
		if (!inV1.connected() || !inV2.connected() || !inV3.connected() || !inV4.connected() || !inV5.connected()
				|| !inV6.connected() || !inV7.connected())
			return false;
		return true;
	}

	bool VelKin::startHook()
	{
		return true;
	}

	void VelKin::updateHook()
	{
		if (active())
		{
			KDL::Twist twist;
			double alpha;
			/*Lbr_Kin::lbrVelKin(inJ1.Get(), inJ2.Get(), inJ3.Get(), inJ4.Get(), inJ5.Get(), inJ6.Get(), inJ7.Get(),
					inV1.Get(), inV2.Get(), inV3.Get(), inV4.Get(), inV5.Get(), inV6.Get(), inV7.Get(), twist,
					alpha, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7);*/
			devins->alphaVelKin(inJ1.Get(), inJ2.Get(), inJ3.Get(), inJ4.Get(), inJ5.Get(), inJ6.Get(), inJ7.Get(),
					inV1.Get(), inV2.Get(), inV3.Get(), inV4.Get(), inV5.Get(), inV6.Get(), inV7.Get(), twist, alpha);
			outTwist.Set(twist);
			outVAlpha.Set(alpha);
		}
	}

	void VelKin::stopHook()
	{
		// Your stop code after last updateHook()
	}

	void VelKin::cleanupHook()
	{
		// Your configuration cleanup code
	}

}
