/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef YOUBOTARMJOINTIMPEDANCECONTROLLER_HPP_
#define YOUBOTARMJOINTIMPEDANCECONTROLLER_HPP_

#include "../device/youBotArmController.hpp"

#include "../kinematics/yb_KindynChain.hpp"
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <libs/framework/robotarm/interface/RobotArmInterface.hpp>
#include <memory>


namespace kuka_youbot
{
	class youBotArmJointImpedanceController: public youBotArmController
	{

	public:
		youBotArmJointImpedanceController(youBotArmDataSource* dataSource, youBotArmControllable* controllable,int jointDirection[]);
		virtual ~youBotArmJointImpedanceController();
		virtual void updateHook();

		// configure mass spring damper systems
		void setStiffness(int joint, float stiffness);
		void setDampingConstant(int joint, float dampingConstant);
		void setAdditionalTorque(int joint, float torque);
		void setMaximumTorque(int joint, float torque);

		// configure tool
		void setToolMass(double mass);
		void setToolCOM(KDL::Vector com);
		void setToolMOI(KDL::Vector moi);

	private:

		void updateTool();

		// data+controll access
		youBotArmDataSource* m_dataSource;
		youBotArmControllable* m_controllable;

		// work with own joint directions
		int m_jointDirection[5];

		// simulate mass spring damper system
		float m_stiffnessAxes[5];
		float m_dampingConstant[5];

		// additional torque
		float m_maxTorque[5];
		float m_additionalTorque[5];

		// tool handling
		double m_toolMass;
		KDL::Vector m_toolCOM;
		KDL::RotationalInertia m_toolMOI;

		// youbot values
		KDL::Chain m_chain;

		// addional wrenches (not used)
		KDL::Wrenches m_wrenches;

		// jointpostion, jointvelocity, jointacceleration, jointtorque
		KDL::JntArray m_q, m_qdot, m_qdotdot, m_torques;
	};
}

#endif /* YOUBOTARMJOINTIMPEDANCECONTROLLER_HPP_ */
