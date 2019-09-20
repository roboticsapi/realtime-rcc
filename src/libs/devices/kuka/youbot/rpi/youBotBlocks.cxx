/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <boost/algorithm/string.hpp>
#include <kdl/frames.hpp>

#include "../device/youBotArm.hpp"
#include "../web/web.hpp"
#include "../device/youBotBase.hpp"

#include <rcc/Extension.hpp>
#include <rcc/DeviceFactory.hpp>

#include "youBotBlocks.hpp"

#define GRIPPER_UPPER_RANGE_IN_METER 0.023

namespace kuka_youbot
{
	using namespace RPI;
	using namespace std;

	/** SlaveError primitive */
	SlaveError::SlaveError(string name, RPI::Net* net) :
			Module(name, net), bit0("bit0", this), bit1("bit1", this), bit2("bit2", this), bit3("bit3", this), bit4(
					"bit4", this), bit5("bit5", this), bit8("bit8", this), bit9("bit9", this), bit10("bit10", this), bit11(
					"bit11", this), bit14("bit14", this), bit15("bit15", this), bit16("bit16", this), bit17("bit17",
					this), propRobot("Robot", "Name of LBR", ""), propPort("Port", "Port in FRI packet to be used",
					0), devins()
	{
		setDescription("Read error value from axis");
		this->ports()->addPort(&bit0, "Overcurrent flag");
		this->ports()->addPort(&bit1, "Undervoltage flag");
		this->ports()->addPort(&bit2, "Overvoltage flag");
		this->ports()->addPort(&bit3, "Overtemperature flag");
		this->ports()->addPort(&bit4, "Motor halted flag");
		this->ports()->addPort(&bit5, "Hall error flag");
		this->ports()->addPort(&bit8, "PWM mode active flag");
		this->ports()->addPort(&bit9, "Velocity mode active flag");
		this->ports()->addPort(&bit10, "Position mode active flag");
		this->ports()->addPort(&bit11, "Torque mode active flag");
		this->ports()->addPort(&bit14, "Position end flag");
		this->ports()->addPort(&bit15, "Module initialized flag");
		this->ports()->addPort(&bit16, "Ethercat timeout flag");
		this->ports()->addPort(&bit17, "I2t Exceeded flag");

		this->properties()->addProperty(&propRobot);
		this->properties()->addProperty(&propPort);
	}

	SlaveError::~SlaveError()
	{
	}

	bool SlaveError::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "YB " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool SlaveError::startHook()
	{
		return true;
	}

	void SlaveError::updateHook()
	{

	}

	void SlaveError::stopHook()
	{
	}

	void SlaveError::cleanupHook()
	{
	}

	string SlaveError::getDeviceName()
	{
		return propRobot.get();
	}

	bool SlaveError::isSensor()
	{
		return true;
	}

	void SlaveError::updateSensor()
	{
		bit0.Set(devins.getDevice()->getJointError(propPort.get() + 1, 0));
		bit1.Set(devins.getDevice()->getJointError(propPort.get() + 1, 1));
		bit2.Set(devins.getDevice()->getJointError(propPort.get() + 1, 2));
		bit3.Set(devins.getDevice()->getJointError(propPort.get() + 1, 3));
		bit4.Set(devins.getDevice()->getJointError(propPort.get() + 1, 4));
		bit5.Set(devins.getDevice()->getJointError(propPort.get() + 1, 5));
		bit8.Set(devins.getDevice()->getJointError(propPort.get() + 1, 8));
		bit9.Set(devins.getDevice()->getJointError(propPort.get() + 1, 9));
		bit10.Set(devins.getDevice()->getJointError(propPort.get() + 1, 10));
		bit11.Set(devins.getDevice()->getJointError(propPort.get() + 1, 11));
		bit14.Set(devins.getDevice()->getJointError(propPort.get() + 1, 14));
		bit15.Set(devins.getDevice()->getJointError(propPort.get() + 1, 15));
		bit16.Set(devins.getDevice()->getJointError(propPort.get() + 1, 16));
		bit17.Set(devins.getDevice()->getJointError(propPort.get() + 1, 17));
	}


	/** Gripper primitive */
	Gripper::Gripper(string name, RPI::Net* net) :
			ActiveModule(name, net), outCompleted("outCompleted", this), propDistance("Distance",
					"Gripper finger distance", 0), propRobot("Robot", "Name of youBot", ""), devins(), gripperPositionSend(
					false)
	{
		setDescription("Double variable to move the gripper");
		this->ports()->addPort(&outCompleted, "gripper request has completed");
		this->properties()->addProperty(&propDistance);
		this->properties()->addProperty(&propRobot);
	}

	Gripper::~Gripper()
	{
	}

	bool Gripper::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "YB " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}

		if ((propDistance.get() < 0) || (ceilf(propDistance.get() * 1000) / 1000) > GRIPPER_UPPER_RANGE_IN_METER) //Round up and compare
		{
			RTT::log(RTT::Error) << "YB - Gripper: " << propDistance.get() << " out of range!" << RTT::endlog();
			return false;
		}
		return true;
	}

	bool Gripper::startHook()
	{
		return true;
	}

	void Gripper::updateHook()
	{
		if (!active()) return;

		if (gripperPositionSend)
			outCompleted.Set(!devins->gripperBusy());
		else
		{
			gripperPositionSend = devins->setGripperPosition(propDistance.get());
			outCompleted.Set(false);
		}
	}

	void Gripper::stopHook()
	{
	}

	void Gripper::cleanupHook()
	{
	}

	string Gripper::getDeviceName()
	{
		return propRobot.get();
	}

	bool Gripper::isActuator()
	{
		return true;
	}

	void Gripper::updateActuator()
	{

		return;
	}



	/** GripperMonitor primitive */
	GripperMonitor::GripperMonitor(string name, RPI::Net* net) :
			Module(name, net), outDistance("outDistance", this), propRobot("Robot", "Name of youBot", ""), devins(), distance(0)
	{
		setDescription("Monitor the gripper position");
		this->ports()->addPort(&outDistance, "Finger distance [m]");
		this->properties()->addProperty(&propRobot);
	}

	GripperMonitor::~GripperMonitor()
	{
	}

	bool GripperMonitor::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "YB " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool GripperMonitor::startHook()
	{
		return true;
	}

	void GripperMonitor::updateHook()
	{
		outDistance.Set(distance);
	}

	void GripperMonitor::stopHook()
	{
	}

	void GripperMonitor::cleanupHook()
	{
	}

	string GripperMonitor::getDeviceName()
	{
		return propRobot.get();
	}

	bool GripperMonitor::isSensor()
	{
		return true;
	}

	void GripperMonitor::updateSensor()
	{
		distance = devins->getGripperPosition();
	}


	/** Position preserving Project */
	PositionPreservingProject::PositionPreservingProject(string name, RPI::Net* net) :
			Module(name, net), outValue("outValue", this), inFlange("inFlange",
					this), inMotionCenter("inMotionCenter", this)
	{
		setDescription("Projects a transformation into the reachable subspace for a youBot arm (respecting motion center)");
		this->ports()->addPort(&inFlange, "Flange position");
		this->ports()->addPort(&inMotionCenter, "Motion center position (relative to flange)");
		this->ports()->addPort(&outValue, "Projected flange transformation");
	}

	PositionPreservingProject::~PositionPreservingProject()
	{
	}

	bool PositionPreservingProject::configureHook()
	{
		if(!inFlange.connected())
			return false;
		return true;
	}

	bool PositionPreservingProject::startHook()
	{
		return true;
	}

	void PositionPreservingProject::updateHook()
	{
		KDL::Frame ret =
				projector.project(inFlange.Get(), inMotionCenter.Get(KDL::Frame()));
		outValue.Set(ret);
	}

	void PositionPreservingProject::stopHook()
	{
	}

	void PositionPreservingProject::cleanupHook()
	{
	}


	/** Palletizing Project */
	PalletizingProject::PalletizingProject(string name, RPI::Net* net) :
			Module(name, net), outValue("outValue", this), inFlange("inFlange",
					this), inMotionCenter("inMotionCenter", this)
	{
		setDescription("Projects a transformation so that the Z direction points upwards or downwards (respecting motion center)");
		this->ports()->addPort(&inFlange, "Flange position");
		this->ports()->addPort(&inMotionCenter, "Motion center position (relative to flange)");
		this->ports()->addPort(&outValue, "Projected flange transformation");
	}

	PalletizingProject::~PalletizingProject()
	{
	}

	bool PalletizingProject::configureHook()
	{
		if(!inFlange.connected())
			return false;
		return true;
	}

	bool PalletizingProject::startHook()
	{
		return true;
	}

	void PalletizingProject::updateHook()
	{
		KDL::Frame ret =
				projector.project(inFlange.Get(), inMotionCenter.Get(KDL::Frame()));
		outValue.Set(ret);
	}

	void PalletizingProject::stopHook()
	{
	}

	void PalletizingProject::cleanupHook()
	{
	}

	/** ArmControlStrategy*/
	ArmControlStrategy::ArmControlStrategy(string name, RPI::Net* net) :
			ActiveModule(name, net),  outSuccess("outSuccess", this),
			inController("inController",this), propController("Controller","Index of Controller to set"),
			propRobot("Robot", "Name of youBot", ""), devins(), selectController(-1)
	{
		setDescription("Integer variable to select controller");
		this->ports()->addPort(&outSuccess, "controller selection has completed");
		this->ports()->addPort(&inController, "Index to select Controller");
		this->properties()->addProperty(&propRobot);
		this->properties()->addProperty(&propController);
	}

	ArmControlStrategy::~ArmControlStrategy()
	{
	}

	bool ArmControlStrategy::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "YB " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}

		return true;
	}

	bool ArmControlStrategy::startHook()
	{
		return true;
	}

	void ArmControlStrategy::updateHook()
	{
		if (!active())return;

		// get selected Controller
		selectController = inController.Get(propController);

		// check index
		if(devins->checkControllerIndex(selectController)) {
			// valid value
			outSuccess.Set(true);
		} else {
			// value invalid
			outSuccess.Set(false);
			selectController = -1;
		}
	}

	void ArmControlStrategy::stopHook()
	{
	}

	void ArmControlStrategy::cleanupHook()
	{
	}

	string ArmControlStrategy::getDeviceName()
	{
		return propRobot.get();
	}

	bool ArmControlStrategy::isActuator()
	{
		return true;
	}

	void ArmControlStrategy::updateActuator()
	{
		// check value
		if (selectController<0) return;

		// set selected controller to arm
		devins->setControllerIndex(selectController);

		// set back
		selectController = -1;
	}



	/** JointImpParameters*/
	JointImpParameters::JointImpParameters(string name, RPI::Net* net) :
			ActiveModule(name, net), outSuccess("outSuccess", this),  inStiffness("inStiffness", this), inDamping("inDamping",this),
			inAddTorque("inAddTorque",this), propAddTorque("AddTorque","Additional Torque in Nm"),
			propRobot("Robot", "Name of youBot", ""),propJointIndex("JointIndex","Joint Index to set paramters"),
			propStiffness("Stiffness","Stiffness Parameter in Nm/rad"),propDamping("Damping","Damping Parameter relative in 0..1 range"),
			inMaxTorque("inMaxTorque",this), propMaxTorque("MaxTorque","Maximum Torque caused by the impedance controller in Nm, <=0 for unlimited"),
			devins(), dampingVal(0),stiffnessVal(0),jointIndex(0),torqueVal(0), receivedValues(false)
	{
		setDescription("Set joint Impedance parameter stiffness and damping");
		this->ports()->addPort(&outSuccess, "Parameter setting was successful");
		this->ports()->addPort(&inStiffness, "Stiffness to set");
		this->ports()->addPort(&inDamping, "Damping to set");
		this->ports()->addPort(&inAddTorque,"Additional Torque in Nm");
		this->ports()->addPort(&inMaxTorque,"Maximum Torque caused by impedance controller in Nm, <=0 for unlimited");
		this->properties()->addProperty(&propRobot);
		this->properties()->addProperty(&propStiffness);
		this->properties()->addProperty(&propDamping);
		this->properties()->addProperty(&propAddTorque);
		this->properties()->addProperty(&propMaxTorque);
		this->properties()->addProperty(&propJointIndex);
	}

	JointImpParameters::~JointImpParameters()
	{
	}

	bool JointImpParameters::configureHook()
	{
		if (!devins.fetchInstance(propRobot.get()))
		{
			RTT::log(RTT::Error) << "YB " << propRobot.get() << " not available." << RTT::endlog();
			return false;
		}

		// get joint Index
		if (propJointIndex.get() < 0 || propJointIndex.get() > 4) {
			RTT::log(RTT::Error) << "YB " << propJointIndex.get() << " is wrong index." << RTT::endlog();
			return false;
		} else {
			// set joint index
			jointIndex = propJointIndex.get();
		}

		return true;
	}

	bool JointImpParameters::startHook()
	{
		return true;
	}

	void JointImpParameters::updateHook()
	{
		if (!active())
			return;

		// get stiffness
		stiffnessVal = (float) inStiffness.Get(propStiffness);

		// get damping
		dampingVal = (float) inDamping.Get(propDamping);

		// get additional torque
		torqueVal = (float) inAddTorque.Get(propAddTorque);

		maxTorqueVal = (float) inMaxTorque.Get(propMaxTorque);

		// check values
		if (devins->checkControllerJointImpedanceParameters(jointIndex,stiffnessVal,dampingVal,maxTorqueVal)
				&& devins->checkControllerJointImpedanceAdditionalTorque(jointIndex,torqueVal)) {
			// valid values
			receivedValues = true;
			outSuccess.Set(true);
		} else {
			// invalid values
			outSuccess.Set(false);
		}
	}

	void JointImpParameters::stopHook()
	{
	}

	void JointImpParameters::cleanupHook()
	{
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
		if(!receivedValues)return;

		// set stiffness and damping to arm
		devins->setControllerJointImpedanceParameters(jointIndex,stiffnessVal,dampingVal,maxTorqueVal);

		// set additional torque to arm
		devins->setControllerJointImpedanceAddionalTorque(jointIndex,torqueVal);

		// set back
		receivedValues = false;
	}
}
