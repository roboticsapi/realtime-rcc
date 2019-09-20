/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef IIWAFRI_HPP
#define IIWAFRI_HPP

#include <string.h>
#include <stdio.h>
#include <math.h>

#include <rtt/Activity.hpp>
#include <rtt/TaskContext.hpp>

#include "frisdk/friClientApplication.h"
#include "frisdk/friLBRClient.h"
#include "frisdk/friClientData.h"

#include "UdpConnection.hpp"
#include "../device/IiwaDevice.hpp"
#include <libs/framework/io/interface/IOInterface.hpp>
#include <libs/framework/robotarm/device/CyclicPositionRobotArm.hpp>
#include <libs/framework/robotarm/interface/RobotArmInterface.hpp>
#include <libs/framework/armkinematics/interface/ArmKinematicsInterface.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#ifndef DEFAULT_FRI_PORTID
#define DEFAULT_FRI_PORTID 30200
#endif

#ifndef DEFAULT_CONTROL_MODE_PORTID
#define DEFAULT_CONTROL_MODE_PORTID 30000
#endif

#ifndef IIWA_FRI_CYCLE_TIME
#define IIWA_FRI_CYCLE_TIME 0.002
#endif

#ifndef IIWA_CONTROL_MODE_PACKET_SIZE
#define IIWA_CONTROL_MODE_PACKET_SIZE 49
#endif

#ifndef IIWA_CONTROL_MODE_HEARTBEAT_CYCLE_TIME
#define IIWA_CONTROL_MODE_HEARTBEAT_CYCLE_TIME 5
#endif

#define LBR_MNJ 7				/*!< Number of Managed Joints for Light Weight Robot */
#define FRI_USER_SIZE 16   		/*!< Number of Userdefined data, which are exchanged to KRL Interpreter  */
#define FRI_CART_FRM_DIM 12     /*!< Number of data within a 3x4 Cartesian frame - homogenous matrix */
#define FRI_CART_VEC  6			/*!< Number of data for a Cartesian vector */

namespace kuka_iiwa
{
	using namespace RPI;
	using namespace std;
	using namespace KUKA::FRI;

	const std::string dev_lbr_joint = "kuka_iiwa_fri";
	const int iiwa_fri_axiscount = 7;

	struct IIWA_DATA
	{
		// Current axis position
		double pos_cur[iiwa_fri_axiscount];

		// Current Ipo position
		double ipo_cur[iiwa_fri_axiscount];

		// calculated values
		double vel_cur[iiwa_fri_axiscount];
		double acc_cur[iiwa_fri_axiscount];

		//current torque values
		double torque_cur[iiwa_fri_axiscount];

		// offset values for FRI start
		double pos_offset[iiwa_fri_axiscount];

		// FRI information
		ESessionState session_state;
		EConnectionQuality conn_quality;
		ESafetyState safety_state;
		EOperationMode op_mode;
		EDriveState drive_state;
		EControlMode control_mode;

		RTT::os::TimeService::nsecs timestamp;
	};


	class FRIDriver: public RTT::TaskContext,
			public LBRClient,
			public IiwaDevice,
			public robotarm::CyclicPositionRobotArm

	{
	public:
		FRIDriver(std::string name, RPI::parameter_t parameters);
		static FRIDriver* createDevice(std::string name, RPI::parameter_t parameters);
		virtual ~FRIDriver();

		// Interface TaskContext
		bool startHook();
		bool configureHook();
		void updateHook();

		//Interface LBRClient
		virtual void onStateChange(ESessionState oldState, ESessionState newState);
		virtual void command();
		virtual void monitor();
		virtual void waitForCommand();

		// Interface Cyclic position
		double getMaximumAcceleration(int joint) const;
		double getMaximumVelocity(int joint) const;

		// Interface RobotArm
		int getJointError(int joint);
		int getJointCount() const;
		robotarm::JointPositionError checkJointPosition(int joint, double position);
		double getMeasuredJointPosition(int joint);
		double getMeasuredJointVelocity(int joint);
		double getMeasuredJointAcceleration(int joint);
		virtual void setToolCOM(KDL::Vector com, int axis);
		virtual void setToolMass(double mass, int axis);
		virtual void setToolMOI(KDL::Vector moi, int axis);
		virtual bool getToolFinished(int axis) const;
		virtual int getToolError(int axis) const;

		// Interface Device
		std::set<std::string> getMutableParameters() const;
		void updateParameters();
		RPI::DeviceState getDeviceState() const;
		void setEStop(bool estop);

		void setJointConfig(int axis, double _max_vel, double _max_acc, double minj, double maxj);
		int getControlScheme();
		int getState();
		int getQuality();
		bool getPower() const;
		bool isSpringRelaxed();

		float getAxisTqAct(int axis);
		float getAxisTqEst(int axis);
		float* getTcpEst();

		bool loadDataChanged();
		bool controlSchemeChanged();

		int getKRCControlScheme();
		bool setKRCDesiredControlScheme(int scheme);
		int getKRCModeError() const;
		bool getKRCModeFinished() const;
		void setKRCModeFinished(bool state);
		bool setKRCToolTCP(const KDL::Frame& tcp);
		bool setKRCToolCOM(const KDL::Frame& com);
		bool setKRCToolMOI(const KDL::Vector& moi);
		bool setKRCToolMass(double mass);

		void setJntImpStiffness(float stiff, int axis);
		void setJntImpDamping(float damp, int axis);
		void setJntImpAddTorque(float trq, int axis);
		void setCartImpStiffness(float x, float y, float z, float a, float b, float c);
		void setCartImpDamping(float x, float y, float z, float a, float b, float c);
		void setCartImpAddTcpFT(float x, float y, float z, float a, float b, float c);
		void getCartPosition(float& x, float& y, float& z, float& a, float& b, float& c);

		void setCartSinImpAmplitude(float x, float y, float z, float a, float b, float c);
		void setCartSinImpFrequency(float x, float y, float z, float a, float b, float c);
		void getCartSinImpAmplitude(float& x, float& y, float& z, float& a, float& b, float& c);
		void getCartSinImpFrequency(float& x, float& y, float& z, float& a, float& b, float& c);

		float getJntImpStiffness(int axis);
		float getJntImpDamping(int axis);
		float getJntImpAddTorque(int axis);
		void getCartImpStiffness(float& x, float& y, float& z, float& a, float& b, float& c);
		void getCartImpDamping(float& x, float& y, float& z, float& a, float& b, float& c);
		void getCartImpAddTcpFT(float& x, float& y, float& z, float& a, float& b, float& c);

		unsigned int getNumDigitalIn() const;
		unsigned int getNumDigitalOut() const;
		unsigned int getNumAnalogIn() const;
		unsigned int getNumAnalogOut() const;
		void setDigitalOut(int port, bool value);
		bool getDigitalOut(int port);
		bool getDigitalIn(int port);
		void setAnalogOut(int port, double value);
		double getAnalogOut(int port);
		double getAnalogIn(int port);
		float* getMsrJointVel();
		float* getMsrJointAcc();


	private:
		void sendControlMode();
		void sendHeartbeat();
		void receiveControlModeHeartbeat();
		void receiveControlModeConfirmation();
		void sendConnectionCloseMessage();

		ClientApplication* app;
		UdpConnection connection;
		RPI::DeviceInstanceT<ethernet::EthernetServerUDPInterface> udp;

		bool isEstop, isStart;

		std::string deviceName;
		std::string udpDeviceName;
		std::string udpControlModeDevice;
		int fri_port, control_mode_port;

		int krc_mode_error, krc_desired_controlscheme;
		int current_mode;
		bool krc_mode_finished, newControlMode, newControlModeConfirmed;
		enum krc_switch_state
		{
			INACTIVE, REQUESTED, ACKNOWLEDGED, CONFIRMED
		};
		krc_switch_state krc_switch_state;

		//struct for connection information
		IIWA_DATA data;

		// Tool configuration
		double toolmass, toolmassold, krc_desired_mass;
		KDL::Vector toolcom, toolcomold, toolmoi;
		KDL::Vector krc_desired_moi, krc_current_moi;
		KDL::Frame krc_desired_tcp, krc_current_tcp, krc_desired_com, krc_current_com;
		bool toolswitching;
		float joints[LBR_MNJ], jntstiff[LBR_MNJ], jntdamp[LBR_MNJ], jntaddtrq[LBR_MNJ];
		float cartpos[FRI_CART_FRM_DIM], cartstiff[FRI_CART_VEC], cartdamp[FRI_CART_VEC], cartaddtcpft[FRI_CART_VEC];
		float jointvel[LBR_MNJ], jointacc[LBR_MNJ];
		float cartsinamp[FRI_CART_VEC], cartsinfreq[FRI_CART_VEC];

		DeviceState deviceState;

		struct sockaddr_in _controllerAddr; //!< the controller's socket address

		std::map<int, std::string> IPortsDig;
		std::map<int, std::string> OPortsDig;
		std::map<int, std::string> IPortsAn;
		std::map<int, std::string> OPortsAn;

		RTT::os::TimeService::nsecs controlModeTime;

		//RPI::RoundRobinLog<double>* dumpMsrPos;

	protected:
		std::vector<double> max_vel, max_acc, minj, maxj;
	};

}
#endif /* IIWAFRI_HPP_ */
