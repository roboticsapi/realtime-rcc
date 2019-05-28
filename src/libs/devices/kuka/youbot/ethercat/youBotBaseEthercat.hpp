/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef KUKA_YOUBOTBASE_EC_HPP_
#define KUKA_YOUBOTBASE_EC_HPP_

#include <vector>

#include <rtt/TaskContext.hpp>
#include <rcc/Device.hpp>
#include <rcc/DeviceInstanceT.hpp>
#include <rcc/RoundRobinLog.hpp>

#include "EthercatMaster.hpp"
#include "EthercatAxis.hpp"

#include "../device/youBotBase.hpp"

#include <libs/framework/robotbase/interface/RobotBaseInterface.hpp>
#include <libs/framework/cartesianposition/device/CartesianPositionDevice.hpp>

#include "../kinematics/base_Kin.hpp"


namespace kuka_youbot
{
	const std::string dev_kuka_youbot_base_ec = "kuka_youbot_base_ec";


	class youBotBaseEthercat:
			public RTT::TaskContext,
			public RPI::Device,
			public youBotBase,
			public ethercat::EthercatDevice,
			public cartesianposition::CartesianPositionDevice,
			YoubotEthercatSlaveNotify
	{
	public:
		youBotBaseEthercat(std::string name, RPI::parameter_t parameters);
		virtual ~youBotBaseEthercat();

		bool startHook();
		bool configureHook();
		void updateHook();

		static youBotBaseEthercat* createDevice(std::string name, RPI::parameter_t parameters);

		void updateParameters();
		std::set<std::string> getMutableParameters() const;

		void setEStop(bool estop);
		RPI::DeviceState getDeviceState() const;

		bool setVelocity(double forward, double left, double yaw);
		void getPosition(double& x, double& y, double& yaw);
		void getVelocity(double& x, double& y, double& yaw);


		// RobotBaseDriver implementation
		int getBaseError();
		std::string getRobotBaseResourceName();

		int checkBaseVelocity(KDL::Twist velocity);
		void setBaseVelocity(KDL::Twist velocity);
		KDL::Frame getMeasuredBasePosition();
		KDL::Twist getMeasuredBaseVelocity();
		KDL::Twist getCommandedBaseVelocity();

		int getWheelCount() const;
		double getWheelPosition(int wheel) const;
		double getWheelVelocity(int wheel) const;

		// CartesianPosition implementation
		int getCartesianPositionDeviceError();
		std::string getCartesianPositionResourceName();
		int checkPosition(KDL::Frame position);
		KDL::Frame getMeasuredPosition();
		KDL::Twist getMeasuredVelocity();

		// EthercatDevice
		virtual void startupDevice();
		virtual void updateDevice();
		virtual void shutdownDevice();
		virtual void inSafeOp(int slaveno);

	private:

		YoubotEthercatSlave* slaves[4];
		RPI::DeviceInstanceT<ethercat::EthercatMaster> master;
		int baseNr;

		void initialize();

		ybAxis* baseAxes[4];
		double wheelPos[4];
		base_Kin kin;

		int IDX_FRONTLEFT, IDX_FRONTRIGHT, IDX_REARLEFT, IDX_REARRIGHT;
		double direction[4];

		KDL::Frame msrPos;
		KDL::Twist msrVel;

		KDL::Twist rpiVel;
		RTT::os::TimeService::nsecs rpiTime;

		RTT::os::TimeService::nsecs ecTime;

		bool isEStop;

		bool isInit;
		bool startInit;


		RPI::RoundRobinLog<double>* dumpCmdPos;
		RPI::RoundRobinLog<double>* dumpMsrPos;
		RPI::RoundRobinLog<double>* dumpMsrVel;
		RPI::RoundRobinLog<double>* dumpCmdVel;
		RPI::RoundRobinLog<double>* dumpRpiVel;
		RPI::RoundRobinLog<double>* dumpMsrWheelPos;
		RPI::RoundRobinLog<double>* dumpMsrWheelVel;
		RPI::RoundRobinLog<double>* dumpCmdWheelVel;

	};

} /* namespace youbot */
#endif /* KUKA_YOUBOTBASE_EC_HPP_ */
