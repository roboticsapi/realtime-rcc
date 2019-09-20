/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#pragma once

#include "../device/IiwaDevice.hpp"
#include <libs/framework/robotarm/simulation/RobotArmDriverSimulation.hpp>
#include <libs/framework/io/simulation/IODeviceSimulation.hpp>

#define NUM_AXIS 7

namespace kuka_iiwa
{
	const std::string dev_iiwa_joint_sim = "kuka_iiwa_sim";

	class IiwaSimulation: virtual public IiwaDevice,
			virtual public robotarm::RobotArmDriverSimulation,
			virtual public IO::IODeviceSimulation
	{
	public:
		IiwaSimulation(std::string name, RPI::parameter_t parameters);
		virtual ~IiwaSimulation();

		static IiwaSimulation* createDevice(std::string name, RPI::parameter_t parameters);

		virtual void setJntImpStiffness(float stiff, int axis);
		virtual void setJntImpDamping(float damp, int axis);
		virtual void setJntImpAddTorque(float trq, int axis);
		virtual void setCartImpStiffness(float x, float y, float z, float a, float b, float c);
		virtual void setCartImpDamping(float x, float y, float z, float a, float b, float c);
		virtual void setCartImpAddTcpFT(float x, float y, float z, float a, float b, float c);

		void setCartSinImpAmplitude(float x, float y, float z, float a, float b, float c);
		void setCartSinImpFrequency(float x, float y, float z, float a, float b, float c);
		void getCartSinImpAmplitude(float& x, float& y, float& z, float& a, float& b, float& c);
		void getCartSinImpFrequency(float& x, float& y, float& z, float& a, float& b, float& c);

		virtual float getJntImpStiffness(int axis);
		virtual float getJntImpDamping(int axis);
		virtual float getJntImpAddTorque(int axis);
		virtual void getCartImpStiffness(float& x, float& y, float& z, float& a, float& b, float& c);
		virtual void getCartImpDamping(float& x, float& y, float& z, float& a, float& b, float& c);
		virtual void getCartImpAddTcpFT(float& x, float& y, float& z, float& a, float& b, float& c);

		virtual float getAxisTqAct(int axis);
		virtual float getAxisTqEst(int axis);
		virtual float* getTcpEst();

		// Cartesian position as provided by KRC
		virtual void getCartPosition(float& x, float& y, float& z, float& a, float& b, float& c);

		virtual int getControlScheme();
		virtual int getState();
		virtual int getQuality();

		virtual bool getPower() const;
		virtual bool isSpringRelaxed();

		virtual bool loadDataChanged();
		virtual bool controlSchemeChanged();

		virtual int getKRCModeError() const;
		virtual bool getKRCModeFinished() const;
		virtual void setKRCModeFinished(bool state);

		virtual bool setKRCDesiredControlScheme(int scheme);
		virtual bool setKRCToolTCP(const KDL::Frame& tcp);
		virtual bool setKRCToolCOM(const KDL::Frame& com);
		virtual bool setKRCToolMOI(const KDL::Vector& moi);
		virtual bool setKRCToolMass(double mass);

		virtual int getJointCount() const;

		void setEStop(bool value);
		RPI::DeviceState getDeviceState() const;

		robotarm::JointPositionError checkJointPosition(int joint, double position);

		std::set<std::string> getMutableParameters() const;
		void updateParameters();

	private:
		float jntstiff[NUM_AXIS], jntdamp[NUM_AXIS], jntaddtrq[NUM_AXIS];
		float cartstiff[6], cartdamp[6], cartaddtcpft[6];
		float cartsinamp[6], cartsinfreq[6];
		float tcpest[6];

		bool isEStop;
		int controlscheme;

	};

} /* namespace FRI */
