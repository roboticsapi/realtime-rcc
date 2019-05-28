/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef FRI_HPP_
#define FRI_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>

#include "../kinematics/Lbr_Kin.hpp"
#include "frisdk/friudp.h"
#include "frisdk/friremote.h"
#include <rcc/Module.hpp>
#include <rcc/Device.hpp>
#include <rcc/Server/HTTPServer.hpp>
#include <rcc/Registry.hpp>

#include <libs/framework/io/interface/IOInterface.hpp>
#include <libs/framework/robotarm/device/CyclicPositionRobotArm.hpp>
#include <libs/framework/robotarm/interface/RobotArmInterface.hpp>
#include <libs/framework/armkinematics/interface/ArmKinematicsInterface.hpp>

#include "../device/LwrDevice.hpp"

#define FRILOGSIZE 1000

namespace kuka_lwr
{
	// Device name of LBR_Joint device
	const std::string dev_lbr_joint = "kuka_lwr_fri";



	struct IOPortStateDig
	{
		std::string name;
		bool value;
	};

	struct IOPortStateAn
	{
		std::string name;
		double value;
	};

	struct IOPortState
	{
		std::vector<IOPortStateDig> ioDigOut;
		std::vector<IOPortStateDig> ioDigIn;
		std::vector<IOPortStateAn> ioAnOut;
		std::vector<IOPortStateAn> ioAnIn;
	};

	/**
	 * This class implements a KRC controller interfaced by FRI
	 */
	class FRIDriver: public RTT::TaskContext,
			public robotarm::CyclicPositionRobotArm,
			virtual public LwrDevice
	{
		friend class LBRFRIControllerHandler;
	public:
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

		static FRIDriver* createDevice(std::string name, RPI::parameter_t parameters);

		bool checkJointPosition(float j, int axis);

		void setJntImpStiffness(float stiff, int axis);
		void setJntImpDamping(float damp, int axis);
		void setJntImpAddTorque(float trq, int axis);
		void setCartImpStiffness(float x, float y, float z, float a, float b, float c);
		void setCartImpDamping(float x, float y, float z, float a, float b, float c);
		void setCartImpAddTcpFT(float x, float y, float z, float a, float b, float c);

		virtual float getJntImpStiffness(int axis);
		virtual float getJntImpDamping(int axis);
		virtual float getJntImpAddTorque(int axis);
		virtual void getCartImpStiffness(float& x, float& y, float& z, float& a, float& b, float& c);
		virtual void getCartImpDamping(float& x, float& y, float& z, float& a, float& b, float& c);
		virtual void getCartImpAddTcpFT(float& x, float& y, float& z, float& a, float& b, float& c);


		void getCartPosition(float& x, float& y, float& z, float& a, float& b, float& c);

		float* getJointPosition();
		void getJointPosition(float& j1, float& j2, float& j3, float& j4, float& j5, float& j6, float& j7);

		float getAxisTqAct(int axis);
		float* getAxisAct();
		void getAxisAct(float& j1, float& j2, float& j3, float& j4, float& j5, float& j6, float& j7);

		float getAxisTqEst(int axis);
		float* getAxisEst();
		void getAxisEst(float& j1, float& j2, float& j3, float& j4, float& j5, float& j6, float& j7);

		float* getTcpEst();

		float* getMsrJointVel();
		float* getMsrJointAcc();

		bool getFriFrmBool(int id);
		void setFriToBool(int id, bool value);
		bool getFriToBool(int id);

		float getFriFrmReal(int id);
		void setFriToReal(int id, double value);
		float getFriToReal(int id);

		int getFriFrmInt(int id);
		void setFriToInt(int id, int value);
		int getFriToInt(int id);

		IOPortState getIOPortState();

		int getControlScheme();
		int getState();
		int getQuality();
		bool getPower() const;
		bool isSpringRelaxed();

		virtual ~FRIDriver();

		std::set<std::string> getMutableParameters() const;
		void updateParameters();

		void lockDevice();
		void unlockDevice();

		bool loadDataChanged();
		bool controlSchemeChanged();

		int getKRCModeError() const;
		bool getKRCModeFinished() const;
		void setKRCModeFinished(bool state);

		bool setKRCDesiredControlScheme(int scheme);
		bool setKRCToolTCP(const KDL::Frame& tcp);
		bool setKRCToolCOM(const KDL::Frame& com);
		bool setKRCToolMOI(const KDL::Vector& moi);
		bool setKRCToolMass(double mass);


		int getJointCount() const;
		int getJointError(int joint);
		robotarm::JointPositionError checkJointPosition(int joint, double position);
		double getMeasuredJointPosition(int joint);
		double getMeasuredJointVelocity(int joint);
		double getMeasuredJointAcceleration(int joint);

		virtual void setToolCOM(KDL::Vector com, int axis);
		virtual void setToolMass(double mass, int axis);
		virtual void setToolMOI(KDL::Vector moi, int axis);
		virtual bool getToolFinished(int axis) const;
		virtual int getToolError(int axis) const;

		void setDigitalOut(int port, bool value);
		bool getDigitalOut(int port);
		bool getDigitalIn(int port);
		void setAnalogOut(int port, double value);
		double getAnalogOut(int port);
		double getAnalogIn(int port);

		virtual unsigned int getNumDigitalIn() const;
		virtual unsigned int getNumDigitalOut() const;
		virtual unsigned int getNumAnalogIn() const;
		virtual unsigned int getNumAnalogOut() const;

		void setEStop(bool value);
		RPI::DeviceState getDeviceState() const;

		double getMaximumAcceleration(int joint) const;
		double getMaximumVelocity(int joint) const;

		void getLinkLengths(double& l0, double& l1, double& l2,  double& l3, double& l4, double& l5, double& l6, double& l7);
	private:
		FRIDriver(std::string name, RPI::parameter_t parameters);
		void readioconfig(int portcount, RPI::parameter_t parameters, std::string prefix, std::map<int, std::string>& portlist);

		RPI::DeviceState deviceState;
		//void KRCModeSwitchLoopHandler();
		void KRCMonitorModeLoopHandler();
		int getKRCControlScheme();

		int monitorModeState;

		int id_handshake, id_communication;

		enum krc_switch_state
		{
			INACTIVE, REQUESTED, ACKNOWLEDGED, CONFIRMED
		};
		krc_switch_state krc_switch_state;

		int krc_desired_controlscheme, krc_mode_error;
		bool krc_mode_finished;

		double krc_desired_mass, krc_current_mass;
		KDL::Vector krc_desired_moi, krc_current_moi;
		KDL::Frame krc_desired_tcp, krc_current_tcp, krc_desired_com, krc_current_com;
		bool krc_loadDataValid;

		friRemote* fri;
		float joints[LBR_MNJ], jntstiff[LBR_MNJ], jntdamp[LBR_MNJ], jntaddtrq[LBR_MNJ];
		float cartpos[FRI_CART_FRM_DIM], cartstiff[FRI_CART_VEC], cartdamp[FRI_CART_VEC], cartaddtcpft[FRI_CART_VEC];
		float jointvel[LBR_MNJ], jointacc[LBR_MNJ];

		float old_msrjoint[LBR_MNJ];
		float msrjoint_v[LBR_MNJ];

		float old_msrjoint_v[LBR_MNJ];
		float msrjoint_a[LBR_MNJ];
		int failed;

		RTT::os::TimeService::nsecs lastNsecs;
		RTT::os::Mutex deviceMutex;

		std::map<int, std::string> IPortsDig;
		std::map<int, std::string> OPortsDig;
		std::map<int, std::string> IPortsAn;
		std::map<int, std::string> OPortsAn;

		RPI::RoundRobinLog<double>* dumpMsrPos;
		RPI::RoundRobinLog<double>* dumpMsrVel;
		RPI::RoundRobinLog<double>* dumpFriPos;


		bool isEStop;
	};


}

#endif /* FRI_HPP_ */
