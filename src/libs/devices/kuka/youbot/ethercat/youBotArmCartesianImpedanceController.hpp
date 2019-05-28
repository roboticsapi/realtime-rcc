/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef YOUBOTARMCARTESIANIMPEDANCECONTROLLER_HPP_
#define YOUBOTARMCARTESIANIMPEDANCECONTROLLER_HPP_

#include "../device/youBotArmController.hpp"

#include "../kinematics/yb_KindynChain.hpp"
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <libs/framework/robotarm/interface/RobotArmInterface.hpp>
#include <kdl/utilities/utility.h>
#include <memory>

// ToDo Find right import for kdl::jntarrayvel and kdl::framevel
#include "../kinematics/yb_Kin.hpp"


namespace kuka_youbot
{
	class youBotArmCartesianImpedanceController: public youBotArmController
	{

	public:
		youBotArmCartesianImpedanceController(youBotArmDataSource* dataSource, youBotArmControllable* controllable,int jointDirection[]);
		virtual ~youBotArmCartesianImpedanceController();
		virtual void updateHook();

	private:
		youBotArmDataSource* m_dataSource;
		youBotArmControllable* m_controllable;

		int m_jointDirection[5];

		KDL::Chain m_chain;
		KDL::Wrenches m_wrenches;
		std::shared_ptr<KDL::ChainIdSolver_RNE> m_id_solver;
		KDL::JntArray m_q, m_qdot, m_qdotdot, m_torques;
		KDL::Twist m_stiffnessKart, m_dampingKart;
		KDL::JntArrayVel m_JntVelArrayMeasured,m_JntVelArrayCommanded;
		KDL::Twist tm,td, delta_x, delta_x_dot;
		KDL::Frame pm, pd;
		KDL::FrameVel m_frameVelM,m_frameVelC;
	};
}

#endif /* YOUBOTARMCARTESIANIMPEDANCECONTROLLER_HPP_ */
