/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef LWASIMULATION_HPP_
#define LWASIMULATION_HPP_

#include <libs/framework/armkinematics/interface/ArmKinematicsInterface.hpp>
#include <libs/framework/robotarm/simulation/RobotArmDriverSimulation.hpp>

#include <rcc/Device.hpp>

#include "../kinematics/Schunk_KinGeo.hpp"

namespace schunk_lwa
{

	class LWA_Simulation: public robotarm::RobotArmDriverSimulation,
			public armkinematics::ArmKinematicsInterface,
			public RPI::Device
	{
	public:
		static const unsigned int lwa_jointcount = 6;
		static const std::string lwa_devicename;

		LWA_Simulation(std::string name, RPI::parameter_t parameter);
		virtual ~LWA_Simulation();

		static LWA_Simulation* createDevice(std::string name, RPI::parameter_t parameters);

		virtual bool getPower() const;

		// Interface Device
		void updateParameters();
		std::set<std::string> getMutableParameters() const;
		void setEStop(bool estop);
		RPI::DeviceState getDeviceState() const;

		// Interface ArmKinematics
		KDL::Frame Kin(const RPI::Array<double>& joints);
		void InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position, RPI::Array<double>& values);

		// Interface RobotArm
		virtual int getJointCount() const;
		virtual robotarm::JointPositionError checkJointPosition(int joint, double position);

	private:
		std::string name;
		bool isEStop;

		dh_parameters dh_param;
		Schunk_KinGeo_Calculation* kincalc;

		std::vector<double> minj, maxj;
	};

} /* namespace kuka_kr */
#endif /* LWASIMULATION_HPP_ */
