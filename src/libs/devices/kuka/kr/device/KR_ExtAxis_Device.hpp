/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef KREXTAXISDEVICE_HPP_
#define KREXTAXISDEVICE_HPP_

#include <libs/framework/robotarm/interface/RobotArmInterface.hpp>
#include <rcc/Device.hpp>
#include <rcc/DeviceInstanceT.hpp>
#include "KR_Robot_Device.hpp"

namespace kuka_kr
{

	class RTT_EXPORT KR_ExtAxis_Device: public RPI::Device, virtual public robotarm::RobotArmInterface
	{
	public:
		const static std::string ext_devicename;

		KR_ExtAxis_Device(std::string name, RPI::parameter_t parameter);
		virtual ~KR_ExtAxis_Device();

		static KR_ExtAxis_Device* createDevice(std::string name, RPI::parameter_t parameters);

		// Interface Device
		void setEStop(bool estop);
		virtual RPI::DeviceState getDeviceState() const;
		void updateParameters();
		std::set<std::string> getMutableParameters() const;


		// Interface RobotArm
		virtual int getJointCount() const;
		virtual int getJointError(int joint);

		virtual robotarm::JointPositionError checkJointPosition(int joint, double position);
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

	protected:
		std::string name;
	private:
		RPI::DeviceInstanceT<KR_Robot_Device> kr_arm;
		int kr_ext_axiscount;
		int kr_first_axis;
		bool initialized;
	};

}
/* namespace kuka_kr */

#endif /* KREXTAXISDEVICE_HPP_ */
