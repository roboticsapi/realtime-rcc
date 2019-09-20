/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef KUKA_YOUBOTARM_EC_HPP_
#define KUKA_YOUBOTARM_EC_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>

#include <rcc/Module.hpp>
#include <rcc/Device.hpp>
#include <rcc/Server/HTTPServer.hpp>
#include <rcc/Registry.hpp>
#include <rcc/DeviceInstanceT.hpp>

#include "../device/youBotArm.hpp"
#include "EthercatAxis.hpp"
#include "EthercatMaster.hpp"
#include <libs/framework/robotarm/interface/RobotArmInterface.hpp>
#include <libs/framework/robotarm/device/CyclicPositionRobotArm.hpp>
#include <libs/framework/armkinematics/interface/ArmKinematicsInterface.hpp>
#include "../device/youBotArm.hpp"
#include "../device/youBotArmController.hpp"
#include "../kinematics/yb_Kin.hpp"
#include "youBotArmPositionController.hpp"
#include "youBotArmJointImpedanceController.hpp"

//#include "../kinematics/InvKinABCZ.hpp"



namespace kuka_youbot
{
	// Device name of LBR_Joint device
	const std::string dev_kuka_youbot_arm_ec = "kuka_youbot_arm_ec";


	const int YOUBOT_NUMAXIS = 5;


	class youBotArmDataSourceEthercat : public youBotArmDataSource
	{
	public:
		void setCommandedJointPosition(int joint, double pos) { cmdPos[joint] = pos; }
		void setCommandedJointVelocity(int joint, double vel) { cmdVel[joint] = vel; }
		void setMeasuredJointPosition(int joint, double pos) { msrPos[joint] = pos; }
		void setMeasuredJointVelocity(int joint, double vel) { msrVel[joint] = vel; }
		void setCycleTime(double time) { cycleTime = time; }

		double getCycleTime() {return cycleTime; }
		double getCommandedJointPosition(int joint) { return cmdPos[joint]; }
		double getMeasuredJointPosition(int joint) { return msrPos[joint]; }
		double getCommandedJointVelocity(int joint) { return cmdVel[joint]; }
		double getMeasuredJointVelocity(int joint) { return msrVel[joint];  }
		double getCommandedJointAcceleration(int joint) { return 0; }
		double getMeasuredJointAcceleration(int joint) { return 0;  }
	private:
		double cmdPos[5], cmdVel[5], msrPos[5], msrVel[5];
		double cycleTime;
	};

	/**
	 * This class implements a youBot controller interfaced by the youBot API
	 */
	class youBotArmEthercat:
			public youBotArm,
			public RTT::TaskContext,
			public RPI::Device,
			public ethercat::EthercatDevice,
			robotarm::CyclicPositionRobotArm,
			YoubotEthercatSlaveNotify,
			youBotArmControllable
	{
	public:
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

		static youBotArmEthercat* createDevice(std::string name, RPI::parameter_t parameters);

		void getJointFirmware(std::string& j1, std::string& j2, std::string& j3, std::string& j4, std::string& j5);
		void getJointParameter(int& j1, int& j2, int& j3, int& j4, int& j5, int ParamNumber);
		bool getJointError(int axis, int bit);

		virtual ~youBotArmEthercat();

		void updateParameters();
		std::set<std::string> getMutableParameters() const;

		void setEStop(bool estop);
		RPI::DeviceState getDeviceState() const;

		bool setGripperPositionBlocking(float value);
		void calibrateGripperBlocking();

		virtual void setTorque(int joint, double torque);
		virtual void setVelocity(int joint, double velocity);

		// EthercatDevice
		virtual void startupDevice();
		virtual void updateDevice();
		virtual void shutdownDevice();
		virtual void inSafeOp(int slaveno);

		// controller access
		virtual void setControllerIndex(int controllerIndex);
		virtual bool checkControllerIndex(int controllerIndex);
		virtual void setControllerJointImpedanceParameters(int joint, float stiffness, float damping, float maxTorque);
		virtual bool checkControllerJointImpedanceParameters(int joint, float stiffness, float damping, float maxTorque);
		virtual void setControllerPositionGainConstant(int joint, float gain);
		virtual void setControllerJointImpedanceAddionalTorque(int joint, float torque);
		virtual bool checkControllerJointImpedanceAdditionalTorque(int joint, float torque);

		// RobotArmDriver implementation
		int getJointCount() const;
		int getJointError(int joint);
		robotarm::JointPositionError checkJointPosition(int joint, double position);
		double getMeasuredJointPosition(int joint);
		double getMeasuredJointVelocity(int joint);
		double getMeasuredJointAcceleration(int joint);

		virtual void setToolCOM(KDL::Vector com, int axis);
		virtual void setToolMOI(KDL::Vector moi, int axis);
		virtual void setToolMass(double mass, int axis);
		virtual bool getToolFinished(int axis) const;
		virtual int getToolError(int axis) const;

		// CyclicPositionRobotArm
		virtual double getMaximumAcceleration(int joint) const;
		virtual double getMaximumVelocity(int joint) const;

		// ArmKinematics implementation
		virtual KDL::Frame Kin(const RPI::Array<double>& joints);
		virtual void InvKin(const RPI::Array<double>& hintjoints, const KDL::Frame& position, RPI::Array<double>& resultJoints);

	private:
		youBotArmEthercat(std::string name, RPI::parameter_t parameters);
		bool initEtherCat();
		void initArm();
		void calibrateArm();
		void startArm();

		float msrPos[YOUBOT_NUMAXIS], msrVel[YOUBOT_NUMAXIS], msrAcc[YOUBOT_NUMAXIS];

		float JointAngleToZero[YOUBOT_NUMAXIS];
		float minAngle[YOUBOT_NUMAXIS], maxAngle[YOUBOT_NUMAXIS];
		int msrError[YOUBOT_NUMAXIS];
		int jointDirection[YOUBOT_NUMAXIS];

		int selectedController;

		RTT::os::TimeService::nsecs rpiLast[5];
		float rpiCmdPos[5];
		RTT::os::Mutex jointMutex;
		RTT::os::Mutex deviceMutex;

		ybAxis* armAxis[YOUBOT_NUMAXIS];
		bool isEStop;

		int maxEncoderValueGripper;
		double maxGripperDistance;
		double gripperOffset;

		bool isInit, startInit;
		int armNr;

		yb_kin kin;
//		InvKinABCZ invkin;
		YoubotEthercatSlave* slaves[5];
		RPI::DeviceInstanceT<ethercat::EthercatMaster> master;

		youBotArmController* armController[2];
		youBotArmPositionController *positionController;
		youBotArmJointImpedanceController *jointImpedanceController;

		RPI::RoundRobinLog<double>* dumpMsrPos;
		RPI::RoundRobinLog<double>* dumpMsrVel;
		RPI::RoundRobinLog<double>* dumpECVel;
		RPI::RoundRobinLog<double>* dumpECTorque;

		youBotArmDataSourceEthercat datasource;

	};
}

#endif /* KUKA_YOUBOTARM_EC_HPP_ */
