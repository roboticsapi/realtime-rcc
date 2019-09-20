/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "youBotArmJointImpedanceController.hpp"

namespace kuka_youbot
{
	youBotArmJointImpedanceController::youBotArmJointImpedanceController(youBotArmDataSource* dataSource, youBotArmControllable* controllable,int jointDirection[])
	{
		// store pointer
		m_dataSource = dataSource;
		m_controllable = controllable;

		for (int i = 0; i < 5; i++)
		{
			// use given joint direction
			m_jointDirection[i] = jointDirection[i];

			// set default values
			m_stiffnessAxes[i] = 10;
			m_dampingConstant[i] = 0.5;
			m_additionalTorque[i] = 0;
			m_maxTorque[i] = 0;
		}

		// default tool with zero values
		m_toolMass = 0;
		m_toolCOM = KDL::Vector();

		// configure solver with youbot chain
		m_chain = yb_kindyn_chain::getYBChain();

		// setup arrays
		m_wrenches.resize(m_chain.getNrOfSegments());
		m_q.resize(5);
		m_qdot.resize(5);
		m_qdotdot.resize(5);
		m_torques.resize(5);
		SetToZero (m_q);
		SetToZero (m_qdot);
		SetToZero (m_qdotdot);
		SetToZero (m_torques);
		SetToZero(m_wrenches[m_wrenches.size()-1]);

	}

	youBotArmJointImpedanceController::~youBotArmJointImpedanceController()
	{
		// TODO Auto-generated destructor stub
	}

	void youBotArmJointImpedanceController::updateHook()
	{
		// calc kinematics
		KDL::ChainIdSolver_RNE m_id_solver(m_chain, KDL::Vector(0, 0, -9.81));

		float springVal, dampingVal, torque;
		float mass = m_toolMass;

		// get measured values from data source
		for (unsigned int i = 0; i < 5; i++)
		{
			m_qdotdot(i) = m_jointDirection[i] * m_dataSource->getMeasuredJointAcceleration(i);
			m_q(i) = m_jointDirection[i] * m_dataSource->getMeasuredJointPosition(i);
			m_qdot(i) = m_jointDirection[i] * m_dataSource->getMeasuredJointVelocity(i);
		}

		// calculate joint torques from cartesian forces
		int ret = m_id_solver.CartToJnt(m_q, m_qdot, m_qdotdot, m_wrenches, m_torques);

		//double cmdPos[5], cmdVel[5];
		//getValuesToCommand(cmdPos, cmdVel);

		for (int i = 4; i >= 0; i--)
		{
			// mass pulled by joint i
			mass += m_chain.getSegment(i + 1).getInertia().getMass();

			// damping
			dampingVal = -(m_dampingConstant[i] * 2 * sqrt(mass * m_stiffnessAxes[i]) * m_dataSource->getMeasuredJointVelocity(i) * m_jointDirection[i]);

			// spring value
			springVal = (m_dataSource->getCommandedJointPosition(i) - m_dataSource->getMeasuredJointPosition(i))
					* m_stiffnessAxes[i] * m_jointDirection[i];

			float addTorque = springVal + dampingVal + m_additionalTorque[i] * m_jointDirection[i];

			if(m_maxTorque[i] > 0 && addTorque > m_maxTorque[i]) {
				addTorque = m_maxTorque[i];
			} else if(m_maxTorque[i] > 0 && addTorque < -m_maxTorque[i]) {
				addTorque = -m_maxTorque[i];
			}
			// calculate torque to set use return from id solver
			torque = (m_torques(i) + addTorque) * m_jointDirection[i];

			// set torque
			m_controllable->setTorque(i,torque);

			//springVal = (cmdPos[i] - msrPos[i]) * m_stiffnessAxes[i] * m_jointDirection[i];
			//armAxis[i]->setJointTorque(m_torques(i) + springVal + dampingVal);
		}
	}

	void youBotArmJointImpedanceController::setStiffness(int joint, float stiffness)
	{
		// security check
		if(joint < 0 || joint > 4) return;

		// set stiffness
		m_stiffnessAxes[joint] = stiffness;

	}

	void youBotArmJointImpedanceController::setMaximumTorque(int joint, float torque)
	{
		// security check
		if(joint < 0 || joint > 4) return;

		// set stiffness
		m_maxTorque[joint] = torque;

	}

	void youBotArmJointImpedanceController::setDampingConstant(int joint, float dampingConstant)
	{
		// security check
		if(joint < 0 || joint > 4) return;

		// set damping
		m_dampingConstant[joint] = dampingConstant;
	}

	void youBotArmJointImpedanceController::setAdditionalTorque(int joint, float torque)
	{
		// security check
		if(joint < 0 || joint > 4) return;

		m_additionalTorque[joint] = torque;
	}

	// set mass of tool
	void youBotArmJointImpedanceController::setToolMass(double mass)
	{
		// store value
		m_toolMass = mass;

		// update
		updateTool();
	}

	// set center of gravity of tool
	void youBotArmJointImpedanceController::setToolCOM(KDL::Vector com)
	{
		// store value
		m_toolCOM = com;

		// update
		updateTool();
	}

	// set moment of inertia to tool
	void youBotArmJointImpedanceController::setToolMOI(KDL::Vector moi)
	{
		// store as rotational inerita
		m_toolMOI = KDL::RotationalInertia(moi.x(), moi.y(), moi.z(), 0, 0, 0);

		// update
		updateTool();
	}

	// update chain for new tool values
	void youBotArmJointImpedanceController::updateTool()
	{
		// setup rigidbody inertia with our values
		KDL::RigidBodyInertia toolRigidBodyInerta = KDL::RigidBodyInertia(m_toolMass, m_toolCOM, m_toolMOI);

		// update chain
		m_chain.segments[m_chain.getNrOfSegments() - 1].setInertia(toolRigidBodyInerta);
	}

} // end namespace yb

