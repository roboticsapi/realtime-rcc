/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "LwrSimulation.hpp"

#include "../kinematics/Lbr_Kin.hpp"

namespace kuka_lwr
{
	using namespace std;
	using namespace RPI;

	LwrSimulation::LwrSimulation(std::string name, RPI::parameter_t parameters) :
			LwrDevice(name, parameters), RobotArmDriverSimulation(NUM_AXIS), IODeviceSimulation(8, 8)
	{
		for (int i = 0; i < NUM_AXIS; i++)
		{
			jntstiff[i] = 0;
			jntdamp[i] = 0;
			jntaddtrq[i] = 0;
		}

		for (int i = 0; i < 6; ++i)
		{
			cartstiff[i] = 0;
			cartdamp[i] = 0;
			cartaddtcpft[i] = 0;

			tcpest[i] = 0;
		}

		isEStop = false;
		controlscheme = 1; // position control

		for(const auto& dumper: RobotArmDriverSimulation::getCrashDumpers())
			addCrashDumper(dumper);
	}

	LwrSimulation::~LwrSimulation()
	{

	}

	LwrSimulation* LwrSimulation::createDevice(string name, parameter_t parameters)
	{
		LwrSimulation* ret = new LwrSimulation(name, parameters);
		return ret;
	}

	void LwrSimulation::setJntImpStiffness(float stiff, int axis)
	{
		if (axis >= 0 && axis < NUM_AXIS)
			jntstiff[axis] = stiff;
	}
	void LwrSimulation::setJntImpDamping(float damp, int axis)

	{
		if (axis >= 0 && axis < NUM_AXIS)
			jntdamp[axis] = damp;
	}
	void LwrSimulation::setJntImpAddTorque(float trq, int axis)
	{
		if (axis >= 0 && axis < NUM_AXIS)
			jntaddtrq[axis] = trq;
	}
	void LwrSimulation::setCartImpStiffness(float x, float y, float z, float a, float b, float c)
	{
		cartstiff[0] = x;
		cartstiff[1] = y;
		cartstiff[2] = z;
		cartstiff[3] = a;
		cartstiff[4] = b;
		cartstiff[5] = c;
	}
	void LwrSimulation::setCartImpDamping(float x, float y, float z, float a, float b, float c)
	{
		cartdamp[0] = x;
		cartdamp[1] = y;
		cartdamp[2] = z;
		cartdamp[3] = a;
		cartdamp[4] = b;
		cartdamp[5] = c;
	}
	void LwrSimulation::setCartImpAddTcpFT(float x, float y, float z, float a, float b, float c)
	{
		cartaddtcpft[0] = x;
		cartaddtcpft[1] = y;
		cartaddtcpft[2] = z;
		cartaddtcpft[3] = a;
		cartaddtcpft[4] = b;
		cartaddtcpft[5] = c;
	}

	float LwrSimulation::getJntImpStiffness(int axis)
	{
		if (axis >= 0 && axis < NUM_AXIS)
			return jntstiff[axis];
		return 0;
	}
	float LwrSimulation::getJntImpDamping(int axis)
	{
		if (axis >= 0 && axis < NUM_AXIS)
			return jntdamp[axis];
		return 0;
	}
	float LwrSimulation::getJntImpAddTorque(int axis)
	{
		if (axis >= 0 && axis < NUM_AXIS)
			return jntaddtrq[axis];
		return 0;
	}
	void LwrSimulation::getCartImpStiffness(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartstiff[0];
		y = cartstiff[1];
		z = cartstiff[2];
		a = cartstiff[3];
		b = cartstiff[4];
		c = cartstiff[5];
	}
	void LwrSimulation::getCartImpDamping(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartdamp[0];
		y = cartdamp[1];
		z = cartdamp[2];
		a = cartdamp[3];
		b = cartdamp[4];
		c = cartdamp[5];
	}
	void LwrSimulation::getCartImpAddTcpFT(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartaddtcpft[0];
		y = cartaddtcpft[1];
		z = cartaddtcpft[2];
		a = cartaddtcpft[3];
		b = cartaddtcpft[4];
		c = cartaddtcpft[5];
	}

	float LwrSimulation::getAxisTqAct(int axis)
	{
		return 0;
	}
	float LwrSimulation::getAxisTqEst(int axis)
	{
		return 0;
	}
	float* LwrSimulation::getTcpEst()
	{
		return tcpest;
	}

	// Cartesian position as provided by KRC
	void LwrSimulation::getCartPosition(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		double jn[NUM_AXIS];
		for(int i = 0; i < NUM_AXIS; ++i)
			jn[i] = getMeasuredJointPosition(i);

		KDL::Frame ret;
		double alpha = 0;

		alphaKin(jn[0], jn[1], jn[2], jn[3], jn[4], jn[5], jn[6], ret, alpha);
		//Lbr_Kin::lbrKin(jn[0], jn[1], jn[2], jn[3], jn[4], jn[5], jn[6], ret, alpha, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7);

		x = ret.p.x();
		y = ret.p.y();
		z = ret.p.z();
		double ina, inb, inc;
		ret.M.GetRPY(inc, inb, ina);
		a = ina;
		b = inb;
		c = inc;
	}

	int LwrSimulation::getControlScheme()
	{
		return controlscheme;
	}
	int LwrSimulation::getState()
	{
		return 2; // command mode
	}
	int LwrSimulation::getQuality()
	{
		return 3; // perfect
	}

	bool LwrSimulation::getPower() const
	{
		return !isEStop;
	}
	bool LwrSimulation::isSpringRelaxed()
	{
		return true;
	}

	bool LwrSimulation::loadDataChanged()
	{
		return true;
	}
	bool LwrSimulation::controlSchemeChanged()
	{
		return true;
	}

	int LwrSimulation::getKRCModeError() const
	{
		return 0;
	}
	bool LwrSimulation::getKRCModeFinished() const
	{
		return true;
	}
	void LwrSimulation::setKRCModeFinished(bool state)
	{
	}

	bool LwrSimulation::setKRCDesiredControlScheme(int scheme)
	{
		controlscheme = scheme;
		return true;
	}
	bool LwrSimulation::setKRCToolTCP(const KDL::Frame& tcp)
	{
		return true;
	}
	bool LwrSimulation::setKRCToolCOM(const KDL::Frame& com)
	{
		return true;
	}
	bool LwrSimulation::setKRCToolMOI(const KDL::Vector& moi)
	{
		return true;
	}
	bool LwrSimulation::setKRCToolMass(double mass)
	{
		return true;
	}

	int LwrSimulation::getJointCount() const
	{
		return NUM_AXIS;
	}

	void LwrSimulation::setEStop(bool value)
	{
		isEStop = value;
	}

	RPI::DeviceState LwrSimulation::getDeviceState() const
	{
		return isEStop ? RPI::DeviceState::SAFE_OPERATIONAL : RPI::DeviceState::OPERATIONAL;
	}

	robotarm::JointPositionError LwrSimulation::checkJointPosition(int axis, double j)
	{
		if(j != j) // value is NaN
			return robotarm::JP_INVALID;
		if(axis%2==0) { // even axes can move +- 170°
			if(j > KDL::PI / 180 * 170 || j < -KDL::PI / 180 * 170) return robotarm::JP_OUTOFRANGE;
		} else { // odd axes only allow +-120°
			if(j > KDL::PI / 180 * 120 || j < -KDL::PI / 180 * 120) return robotarm::JP_OUTOFRANGE;
		}
		return robotarm::JP_OK;
	}


	set<string> LwrSimulation::getMutableParameters() const
	{
		return set<string>();
	}

	void LwrSimulation::updateParameters()
	{

	}

} /* namespace FRI */
