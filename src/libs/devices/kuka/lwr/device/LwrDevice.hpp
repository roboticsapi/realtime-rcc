/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef LWRDEVICE_HPP_
#define LWRDEVICE_HPP_

#include <rcc/Device.hpp>
#include <libs/framework/io/interface/IOInterface.hpp>
#include <libs/framework/robotarm/interface/RobotArmInterface.hpp>
#include <libs/framework/armkinematics/interface/ArmKinematicsInterface.hpp>

namespace kuka_lwr
{
	/**
	 * This class implements a KRC controller interfaced by FRI
	 */
	class RTT_EXPORT LwrDevice: public RPI::Device,
			virtual public IO::IOInterface,
			virtual public robotarm::RobotArmInterface,
			virtual public armkinematics::ArmKinematicsInterface
	{
	public:
		LwrDevice(std::string name, RPI::parameter_t parameters);
		virtual ~LwrDevice() { };

		virtual void setJntImpStiffness(float stiff, int axis) = 0;
		virtual void setJntImpDamping(float damp, int axis) = 0;
		virtual void setJntImpAddTorque(float trq, int axis) = 0;
		virtual void setCartImpStiffness(float x, float y, float z, float a, float b, float c) = 0;
		virtual void setCartImpDamping(float x, float y, float z, float a, float b, float c) = 0;
		virtual void setCartImpAddTcpFT(float x, float y, float z, float a, float b, float c) = 0;

		virtual float getJntImpStiffness(int axis) = 0;
		virtual float getJntImpDamping(int axis) = 0;
		virtual float getJntImpAddTorque(int axis) = 0;
		virtual void getCartImpStiffness(float& x, float& y, float& z, float& a, float& b, float& c) = 0;
		virtual void getCartImpDamping(float& x, float& y, float& z, float& a, float& b, float& c) = 0;
		virtual void getCartImpAddTcpFT(float& x, float& y, float& z, float& a, float& b, float& c) = 0;


		virtual float getAxisTqAct(int axis) = 0;
		virtual float getAxisTqEst(int axis) = 0;
		virtual float* getTcpEst() = 0;

		// Cartesian position as provided by KRC
		virtual void getCartPosition(float& x, float& y, float& z, float& a, float& b, float& c) = 0;

		virtual int getControlScheme() = 0;
		virtual int getState() = 0;
		virtual int getQuality() = 0;


		virtual bool getPower() const = 0;
		virtual bool isSpringRelaxed() = 0;

		virtual bool loadDataChanged() = 0;
		virtual bool controlSchemeChanged() = 0;

		virtual int getKRCModeError() const = 0;
		virtual bool getKRCModeFinished() const = 0;
		virtual void setKRCModeFinished(bool state) = 0;

		virtual bool setKRCDesiredControlScheme(int scheme) = 0;
		virtual bool setKRCToolTCP(const KDL::Frame& tcp) = 0;
		virtual bool setKRCToolCOM(const KDL::Frame& com) = 0;
		virtual bool setKRCToolMOI(const KDL::Vector& moi) = 0;
		virtual bool setKRCToolMass(double mass) = 0;

		virtual KDL::Frame Kin(const RPI::Array<double>& joints);
		virtual void InvKin(const RPI::Array<double>& hintjoints, const KDL::Frame& position,
				RPI::Array<double>& resultJoints);

		virtual void alphaKin(double j0, double j1, double j2, double j3, double j4, double j5, double j6, KDL::Frame& pos, double& alpha);
		virtual void alphaInvKin(KDL::Frame pos, double alpha, double& j0, double& j1, double& j2, double& j3, double& j4, double& j5, double& j6);
		virtual void alphaVelKin(double j0, double j1, double j2, double j3, double j4, double j5, double j6,
				double v0, double v1, double v2, double v3, double v4, double v5, double v6, KDL::Twist& twist, double& alpha);

		virtual void getLinkLengths(double& l0, double& l1, double& l2,  double& l3, double& l4, double& l5, double& l6, double& l7);

	private:
		std::vector<double> link_lengths;

	};

} /* namespace FRI */
#endif /* LWRDEVICE_HPP_ */
