/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef KRSIMULATION_HPP_
#define KRSIMULATION_HPP_

#include <libs/framework/robotarm/simulation/RobotArmDriverSimulation.hpp>

#include <rcc/Device.hpp>

#include "../kinematics/KR_Kin.hpp"
#include "../device/KR_Controller.hpp"

namespace kuka_kr
{

	class RTT_EXPORT KR_Controller_Sim: public robotarm::RobotArmDriverSimulation, public virtual KR_Controller
	{
	public:
		static const int kr_sim_jointcount = 12;

		KR_Controller_Sim(std::string name);
		virtual ~KR_Controller_Sim();

		virtual bool getPower() const;

		// Interface Device
		virtual void updateParameters();
		virtual std::set<std::string> getMutableParameters() const;
		virtual void setEStop(bool estop);
		virtual RPI::DeviceState getDeviceState() const;


		virtual int getJointCount() const;
		virtual robotarm::JointPositionError checkJointPosition(int joint, double position);

		virtual void setJointConfig(int axis, double max_vel, double max_acc, double minj, double maxj);
		virtual void setInitialToolConfig(double mass, KDL::Vector com);
	private:
		bool isEstop;

		std::vector<double> max_vel, max_acc, minj, maxj;

	};

} /* namespace kuka_kr */
#endif /* KRSIMULATION_HPP_ */
