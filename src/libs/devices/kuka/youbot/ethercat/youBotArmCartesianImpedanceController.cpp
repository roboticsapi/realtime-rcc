/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "youBotArmCartesianImpedanceController.hpp"
#include <kdl/chainfksolvervel_recursive.hpp>


namespace kuka_youbot
{
	youBotArmCartesianImpedanceController::youBotArmCartesianImpedanceController(youBotArmDataSource* dataSource, youBotArmControllable* controllable,int jointDirection[])
	{
		m_dataSource = dataSource;
		m_controllable = controllable;

		for (int i = 0; i < 5; i++)
			m_jointDirection[i] = jointDirection[i];


		m_chain = yb_kindyn_chain::getYBChain();
		m_id_solver.reset(new KDL::ChainIdSolver_RNE(m_chain, KDL::Vector(0, 0, -9.81)));

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

		double stiff_vel = 50;
		double stiff_rot = 5; //10;
		m_stiffnessKart = KDL::Twist(KDL::Vector(stiff_vel, stiff_vel, stiff_vel),
				KDL::Vector(stiff_rot, stiff_rot, stiff_rot));

		double damp_vel = 0; //-10;
		double damp_rot = 0; //-0.1;
		m_dampingKart = KDL::Twist(KDL::Vector(damp_vel, damp_vel, damp_vel),
				KDL::Vector(damp_rot, damp_rot, damp_rot));

		m_JntVelArrayMeasured.resize(5);
		m_JntVelArrayCommanded.resize(5);
		SetToZero(m_JntVelArrayMeasured);
		SetToZero(m_JntVelArrayCommanded);
	}

	youBotArmCartesianImpedanceController::~youBotArmCartesianImpedanceController()
	{
		// TODO Auto-generated destructor stub
	}

	void youBotArmCartesianImpedanceController::updateHook()
	{
		int ret;

		for (unsigned int i = 0; i < 5; i++)
		{
			m_JntVelArrayMeasured.q(i) = m_jointDirection[i] * m_dataSource->getMeasuredJointPosition(i);
			m_JntVelArrayMeasured.qdot(i) = m_jointDirection[i] * m_dataSource->getMeasuredJointVelocity(i);
			m_JntVelArrayCommanded.q(i) = m_jointDirection[i] * m_dataSource->getCommandedJointPosition(i);
			m_JntVelArrayCommanded.qdot(i) = m_jointDirection[i] * m_dataSource->getCommandedJointVelocity(i);
		}

		KDL::ChainFkSolverVel_recursive fksolver(m_chain);

		fksolver.JntToCart(m_JntVelArrayMeasured, m_frameVelM);
		fksolver.JntToCart(m_JntVelArrayCommanded, m_frameVelC);

		// get position and twist
		pm = m_frameVelM.GetFrame();
		tm = m_frameVelM.GetTwist();
		pd = m_frameVelC.GetFrame();
		td = m_frameVelC.GetTwist();

		delta_x = diff(pm, pd);
		delta_x_dot = tm;

		// set wrenches
		m_wrenches[5].force.x(
				-(m_stiffnessKart.vel.x() * delta_x.vel.x() + m_dampingKart.vel.x() * delta_x_dot.vel.x()));
		m_wrenches[5].force.y(
				-(m_stiffnessKart.vel.y() * delta_x.vel.y() + m_dampingKart.vel.y() * delta_x_dot.vel.y()));
		m_wrenches[5].force.z(
				-(m_stiffnessKart.vel.z() * delta_x.vel.z() + m_dampingKart.vel.z() * delta_x_dot.vel.z()));
		m_wrenches[5].torque.x(
				-(m_stiffnessKart.rot.x() * delta_x.rot.x() + m_dampingKart.rot.x() * delta_x_dot.rot.x()));
		m_wrenches[5].torque.y(
				-(m_stiffnessKart.rot.y() * delta_x.rot.y() + m_dampingKart.rot.y() * delta_x_dot.rot.y()));
		m_wrenches[5].torque.z(
				-(m_stiffnessKart.rot.z() * delta_x.rot.z() + m_dampingKart.rot.z() * delta_x_dot.rot.z()));

		for (unsigned int i = 0; i < 5; i++)
		{
			m_qdotdot(i) = m_jointDirection[i] * m_dataSource->getMeasuredJointAcceleration(i);
			m_q(i) = m_jointDirection[i] * m_dataSource->getMeasuredJointPosition(i);
			m_qdot(i) = m_jointDirection[i] * m_dataSource->getMeasuredJointVelocity(i);
		}

		ret = m_id_solver->CartToJnt(m_q, m_qdot, m_qdotdot, m_wrenches, m_torques);

		for (int i = 4; i >= 0; i--)
		{
			m_controllable->setTorque(i, m_torques(i));
		}
	}
} // end namespace yb

