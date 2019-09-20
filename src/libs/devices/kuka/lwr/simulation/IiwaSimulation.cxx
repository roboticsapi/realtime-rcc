/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "IiwaSimulation.hpp"

#include "../kinematics/Lbr_Kin.hpp"

namespace kuka_iiwa
{
	using namespace std;
	using namespace RPI;

	IiwaSimulation::IiwaSimulation(std::string name, RPI::parameter_t parameters) :
			IiwaDevice(name, parameters), RobotArmDriverSimulation(NUM_AXIS), IODeviceSimulation(8, 8)
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
		controlscheme = 1; // position

		for(const auto& dumper: RobotArmDriverSimulation::getCrashDumpers())
			addCrashDumper(dumper);
	}

	IiwaSimulation::~IiwaSimulation()
	{

	}

	IiwaSimulation* IiwaSimulation::createDevice(string name, parameter_t parameters)
	{
		IiwaSimulation* ret = new IiwaSimulation(name, parameters);
		return ret;
	}

	void IiwaSimulation::setJntImpStiffness(float stiff, int axis)
	{
		if (axis >= 0 && axis < NUM_AXIS)
			jntstiff[axis] = stiff;
	}
	void IiwaSimulation::setJntImpDamping(float damp, int axis)

	{
		if (axis >= 0 && axis < NUM_AXIS)
			jntdamp[axis] = damp;
	}
	void IiwaSimulation::setJntImpAddTorque(float trq, int axis)
	{
		if (axis >= 0 && axis < NUM_AXIS)
			jntaddtrq[axis] = trq;
	}
	void IiwaSimulation::setCartImpStiffness(float x, float y, float z, float a, float b, float c)
	{
		cartstiff[0] = x;
		cartstiff[1] = y;
		cartstiff[2] = z;
		cartstiff[3] = a;
		cartstiff[4] = b;
		cartstiff[5] = c;
	}
	void IiwaSimulation::setCartImpDamping(float x, float y, float z, float a, float b, float c)
	{
		cartdamp[0] = x;
		cartdamp[1] = y;
		cartdamp[2] = z;
		cartdamp[3] = a;
		cartdamp[4] = b;
		cartdamp[5] = c;
	}
	void IiwaSimulation::setCartImpAddTcpFT(float x, float y, float z, float a, float b, float c)
	{
		cartaddtcpft[0] = x;
		cartaddtcpft[1] = y;
		cartaddtcpft[2] = z;
		cartaddtcpft[3] = a;
		cartaddtcpft[4] = b;
		cartaddtcpft[5] = c;
	}

	void IiwaSimulation::setCartSinImpAmplitude(float x, float y, float z, float a, float b, float c){
		cartsinamp[0] = x;
		cartsinamp[1] = y;
		cartsinamp[2] = z;
		cartsinamp[3] = a;
		cartsinamp[4] = b;
		cartsinamp[5] = c;
	}
	void IiwaSimulation::setCartSinImpFrequency(float x, float y, float z, float a, float b, float c){
		cartsinfreq[0] = x;
		cartsinfreq[1] = y;
		cartsinfreq[2] = z;
		cartsinfreq[3] = a;
		cartsinfreq[4] = b;
		cartsinfreq[5] = c;
	}

	float IiwaSimulation::getJntImpStiffness(int axis)
	{
		if (axis >= 0 && axis < NUM_AXIS)
			return jntstiff[axis];
		return 0;
	}
	float IiwaSimulation::getJntImpDamping(int axis)
	{
		if (axis >= 0 && axis < NUM_AXIS)
			return jntdamp[axis];
		return 0;
	}
	float IiwaSimulation::getJntImpAddTorque(int axis)
	{
		if (axis >= 0 && axis < NUM_AXIS)
			return jntaddtrq[axis];
		return 0;
	}
	void IiwaSimulation::getCartImpStiffness(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartstiff[0];
		y = cartstiff[1];
		z = cartstiff[2];
		a = cartstiff[3];
		b = cartstiff[4];
		c = cartstiff[5];
	}
	void IiwaSimulation::getCartImpDamping(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartdamp[0];
		y = cartdamp[1];
		z = cartdamp[2];
		a = cartdamp[3];
		b = cartdamp[4];
		c = cartdamp[5];
	}
	void IiwaSimulation::getCartImpAddTcpFT(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartaddtcpft[0];
		y = cartaddtcpft[1];
		z = cartaddtcpft[2];
		a = cartaddtcpft[3];
		b = cartaddtcpft[4];
		c = cartaddtcpft[5];
	}

	void IiwaSimulation::getCartSinImpAmplitude(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartsinamp[0];
		y = cartsinamp[1];
		z = cartsinamp[2];
		a = cartsinamp[3];
		b = cartsinamp[4];
		c = cartsinamp[5];
	}

	void IiwaSimulation::getCartSinImpFrequency(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartsinfreq[0];
		y = cartsinfreq[1];
		z = cartsinfreq[2];
		a = cartsinfreq[3];
		b = cartsinfreq[4];
		c = cartsinfreq[5];
	}


	float IiwaSimulation::getAxisTqAct(int axis)
	{
		return 0;
	}
	float IiwaSimulation::getAxisTqEst(int axis)
	{
		return 0;
	}
	float* IiwaSimulation::getTcpEst()
	{
		return tcpest;
	}

	// Cartesian position as provided by KRC
	void IiwaSimulation::getCartPosition(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		double jn[NUM_AXIS];
		for(int i = 0; i < NUM_AXIS; ++i)
			jn[i] = getMeasuredJointPosition(i);

		KDL::Frame ret;
		double alpha = 0;

		alphaKin(jn[0], jn[1], jn[2], jn[3], jn[4], jn[5], jn[6], ret, alpha);

		x = ret.p.x();
		y = ret.p.y();
		z = ret.p.z();
		double ina, inb, inc;
		ret.M.GetRPY(inc, inb, ina);
		a = ina;
		b = inb;
		c = inc;
	}

	int IiwaSimulation::getControlScheme()
	{
		return controlscheme;
	}
	int IiwaSimulation::getState()
	{
		return 2; // command mode
	}
	int IiwaSimulation::getQuality()
	{
		return 3; // perfect
	}

	bool IiwaSimulation::getPower() const
	{
		return !isEStop;
	}
	bool IiwaSimulation::isSpringRelaxed()
	{
		return true;
	}

	bool IiwaSimulation::loadDataChanged()
	{
		return true;
	}
	bool IiwaSimulation::controlSchemeChanged()
	{
		return true;
	}

	int IiwaSimulation::getKRCModeError() const
	{
		return 0;
	}
	bool IiwaSimulation::getKRCModeFinished() const
	{
		return true;
	}
	void IiwaSimulation::setKRCModeFinished(bool state)
	{
	}

	bool IiwaSimulation::setKRCDesiredControlScheme(int scheme)
	{
		controlscheme = scheme;
		return true;
	}
	bool IiwaSimulation::setKRCToolTCP(const KDL::Frame& tcp)
	{
		return true;
	}
	bool IiwaSimulation::setKRCToolCOM(const KDL::Frame& com)
	{
		return true;
	}
	bool IiwaSimulation::setKRCToolMOI(const KDL::Vector& moi)
	{
		return true;
	}
	bool IiwaSimulation::setKRCToolMass(double mass)
	{
		return true;
	}

	int IiwaSimulation::getJointCount() const
	{
		return NUM_AXIS;
	}

	void IiwaSimulation::setEStop(bool value)
	{
		isEStop = value;
	}

	RPI::DeviceState IiwaSimulation::getDeviceState() const
	{
		return isEStop ? RPI::DeviceState::SAFE_OPERATIONAL : RPI::DeviceState::OPERATIONAL;
	}

	robotarm::JointPositionError IiwaSimulation::checkJointPosition(int axis, double j)
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


	set<string> IiwaSimulation::getMutableParameters() const
	{
		return set<string>();
	}

	void IiwaSimulation::updateParameters()
	{

	}


} /* namespace FRI */
