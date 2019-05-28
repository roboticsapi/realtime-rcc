/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef KRDEVICE_HPP_
#define KRDEVICE_HPP_

#include <libs/framework/robotarm/interface/RobotArmInterface.hpp>
#include <libs/framework/armkinematics/interface/ArmKinematicsInterface.hpp>
#include <rcc/Device.hpp>
#include "../kinematics/KR_KinInterface.hpp"
#include "../kinematics/KR_Kin.hpp"
#include "KR_Controller.hpp"

namespace kuka_kr
{

	class RTT_EXPORT KR_Robot_Device: public virtual RPI::Device,
			virtual public robotarm::RobotArmInterface,
			virtual public armkinematics::ArmKinematicsInterface
	{
	public:
		static const unsigned int kr_jointcount = 6;

		KR_Robot_Device(std::string name, RPI::parameter_t parameter, KR_Controller* controller);
		virtual ~KR_Robot_Device();

		// Interface Device
		void updateParameters();
		std::set<std::string> getMutableParameters() const;
		void setEStop(bool estop);
		virtual RPI::DeviceState getDeviceState() const;

		// Interface ArmKinematics
		KDL::Frame Kin(const RPI::Array<double>& joints);
		void InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position, RPI::Array<double>& values);

		// Interface RobotArm
		virtual int getJointCount() const;
		virtual robotarm::JointPositionError checkJointPosition(int joint, double position);

		// RobotArm interface
		virtual int getJointError(int joint);

		virtual void setJointPosition(int joint, double position, RTT::os::TimeService::nsecs time);

		virtual double getMeasuredJointPosition(int joint);
		virtual double getCommandedJointPosition(int joint);

		virtual double getMeasuredJointVelocity(int joint);
		virtual double getCommandedJointVelocity(int joint);

		virtual double getMeasuredJointAcceleration(int joint);
		virtual double getCommandedJointAcceleration(int joint);

		virtual void setToolCOM(KDL::Vector com, int axis);
		virtual void setToolMOI(KDL::Vector moi, int axis);
		virtual void setToolMass(double mass, int axis);
		virtual bool getToolFinished(int axis) const;
		virtual int getToolError(int axis) const;

		KR_Controller* getController() const;

	protected:
		KR_Controller* controller;
		std::string name;

		KR_Kin_Interface* kincalc;

		std::vector<double> minj, maxj, max_vel, max_acc;
	};

}
/* namespace kuka_kr */

#endif /* KRDEVICE_HPP_ */
