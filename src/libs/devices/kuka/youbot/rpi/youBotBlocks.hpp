/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef YOUBOTBLOCKS_HPP_
#define YOUBOTBLOCKS_HPP_

#include "../device/youBotArm.hpp"
#include "../device/youBotBase.hpp"
#include "../device/youBotGripper.hpp"
#include "../kinematics/PositionPreservingProjector.hpp"
#include "../kinematics/PalletizingProjector.hpp"
#include <rcc/DeviceInstanceT.hpp>

namespace kuka_youbot
{
	/**
	 * This class implements a possibility to read the error value from a Joint
	 */
	class SlaveError: public RPI::Module
	{

	public:
		SlaveError(std::string name, RPI::Net* net);
		~SlaveError();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isSensor();
		void updateSensor();

	protected:
		RPI::OutPort<bool> bit0, bit1, bit2, bit3, bit4, bit5;
		RPI::OutPort<bool> bit8, bit9, bit10, bit11;
		RPI::OutPort<bool> bit14, bit15, bit16, bit17;
		RPI::Property<std::string> propRobot;
		RPI::Property<int> propPort;
	private:
		RPI::DeviceInstanceT<youBotArm> devins;

	};

	/**
	 * This class implements a possibility to open and close the gripper of the youBot
	 */
	class Gripper: public RPI::ActiveModule
	{
	public:
		Gripper(std::string name, RPI::Net* net);
		~Gripper();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isActuator();
		void updateActuator();

	protected:
		RPI::OutPort<bool> outCompleted;
		RPI::Property<double> propDistance;
		RPI::Property<std::string> propRobot;
	private:
		RPI::DeviceInstanceT<youBotGripper> devins;
		bool gripperPositionSend;
	};

	/**
	 * This class implements a possibility monitor the gripper of the youBot
	 */
	class GripperMonitor: public RPI::Module
	{
	public:
		GripperMonitor(std::string name, RPI::Net* net);
		~GripperMonitor();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isSensor();
		void updateSensor();

	protected:
		RPI::OutPort<double> outDistance;
		RPI::Property<std::string> propRobot;
	private:
		double distance;
		RPI::DeviceInstanceT<youBotGripper> devins;
	};

	/**
	 * This class projects a transformation into the reachable space of a youBot arm
	 */
	class PositionPreservingProject: public RPI::Module
	{
	public:
		PositionPreservingProject(std::string name, RPI::Net* net);
		~PositionPreservingProject();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

	protected:
		RPI::OutPort<KDL::Frame> outValue;
		RPI::InPort<KDL::Frame> inFlange;
		RPI::InPort<KDL::Frame> inMotionCenter;

	private:
		PositionPreservingProjector projector;
	};


	/**
	 * This class projects a transformation so that the motion center Z direction points upwards or downwards
	 */
	class PalletizingProject: public RPI::Module
	{
	public:
		PalletizingProject(std::string name, RPI::Net* net);
		~PalletizingProject();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

	protected:
		RPI::OutPort<KDL::Frame> outValue;
		RPI::InPort<KDL::Frame> inFlange;
		RPI::InPort<KDL::Frame> inMotionCenter;

	private:
		PalletizingProjector projector;
	};


	/**
	 * This class implements a possibility to configure different control
	 * strategys for the youBot.
	 */
	class ArmControlStrategy: public RPI::ActiveModule
	{
	public:
		ArmControlStrategy(std::string name, RPI::Net* net);
		~ArmControlStrategy();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isActuator();
		void updateActuator();

	protected:
		RPI::OutPort<bool> outSuccess;
		RPI::InPort<int> inController;
		RPI::Property<int> propController;
		RPI::Property<std::string> propRobot;
	private:
		RPI::DeviceInstanceT<youBotArm> devins;
		int selectController;
	};


	/**
	 * This class implements a possibility to set stiffness and damping parameters for
	 * the joint impedance controller
	 */
	class JointImpParameters: public RPI::ActiveModule
	{
	public:
		JointImpParameters(std::string name, RPI::Net* net);
		~JointImpParameters();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isActuator();
		void updateActuator();

	protected:
		RPI::OutPort<bool> outSuccess;
		RPI::InPort<double> inStiffness;
		RPI::InPort<double> inDamping;
		RPI::InPort<double> inAddTorque;
		RPI::InPort<double> inMaxTorque;
		RPI::Property<double> propStiffness;
		RPI::Property<double> propDamping;
		RPI::Property<double> propAddTorque;
		RPI::Property<double> propMaxTorque;
		RPI::Property<int> propJointIndex;
		RPI::Property<std::string> propRobot;
	private:
		RPI::DeviceInstanceT<youBotArm> devins;
		float stiffnessVal,dampingVal, torqueVal, maxTorqueVal;
		int jointIndex;
		bool receivedValues;
	};
}

#endif /* YOUBOTBLOCKS_HPP_ */
