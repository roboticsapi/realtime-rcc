/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef RSI_DRIVER_HPP_
#define RSI_DRIVER_HPP_

#include <rcc/Device.hpp>
#include <libs/framework/robotarm/device/CyclicPositionRobotArm.hpp>
#include <libs/framework/ethernet/interface/EthernetServerUDPInterface.hpp>
#include <rcc/DeviceInstanceT.hpp>

#include <rtt/TaskContext.hpp>

#include "../device/KR_Controller.hpp"

namespace kuka_kr
{
	const unsigned int kr_rsi_axiscount = 12;
	const unsigned int kr_rsi_axiscount_a = 6;

	const unsigned int kr_rsi_digin = 4;
	const unsigned int kr_rsi_digout = 4;

	struct KR_to_SI_factors {
		double conv_factor[12];
	};

	struct RSI_Recv_Data
	{
		// Current axis position
		double pos_cur[kr_rsi_axiscount];
		// Commanded axis position
		//double axis_cmd[kr_rsi_axiscount];
		// Current motor current
		double i_cur[kr_rsi_axiscount];

		// IPO counter, needs to be mirrored to KRC
		long long ipoc;

		// Status value from KRC
		// status 0: Initialize
		// status 1: run mode
		int status;

		bool pactive;

		// offset values for RSI start
		double pos_offset[kr_rsi_axiscount];

		// calculated values
		double vel_cur[kr_rsi_axiscount];
		double acc_cur[kr_rsi_axiscount];

		RTT::os::TimeService::nsecs timestamp;

		bool digin[kr_rsi_digin];
		bool digout[kr_rsi_digout];
	};

	class RTT_EXPORT KR_Controller_RSI: public robotarm::CyclicPositionRobotArm, public RTT::TaskContext, public virtual KR_Controller
	{
	public:
		KR_Controller_RSI(const std::string& name, const std::string& ethdevice, const KR_to_SI_factors& conv_factors);
		virtual ~KR_Controller_RSI();

		// Interface Device
		virtual void updateParameters();
		virtual std::set<std::string> getMutableParameters() const;
		virtual void setEStop(bool estop);
		virtual RPI::DeviceState getDeviceState() const;

		// RobotArm interface
		virtual int getJointError(int joint);

		virtual double getMeasuredJointPosition(int joint);

		virtual double getMeasuredJointVelocity(int joint);

		virtual double getMeasuredJointAcceleration(int joint);

		virtual void setToolCOM(KDL::Vector com, int axis);
		virtual void setToolMOI(KDL::Vector moi, int axis);
		virtual void setToolMass(double mass, int axis);
		virtual bool getToolFinished(int axis) const;
		virtual int getToolError(int axis) const;

		// Cyclic position
		virtual double getMaximumAcceleration(int joint) const;
		virtual double getMaximumVelocity(int joint) const;

		virtual int getJointCount() const;
		virtual robotarm::JointPositionError checkJointPosition(int joint, double position);

		// TaskContext
		bool configureHook();
		void updateHook();

		// Configuration of maxvel and maxacc
		virtual void setJointConfig(int axis, double max_vel, double max_acc, double minj, double maxj);
		virtual void setInitialToolConfig(double mass, KDL::Vector com);

		std::list<RPI::CrashDumper*> getCrashDumpers() const;

		RSI_Recv_Data* getRecvData();
	private:
		void parseKRData();
		void createKRData();
		void calculate_offset();
		void reset_position();

		bool isEstop;

		std::string ethdevicename;
		RPI::DeviceInstanceT<ethernet::EthernetServerUDPInterface> udp;

		char *recvbuffer, *sendbuffer;
		ssize_t sendsize;
		std::string xml_name[kr_rsi_axiscount];

		struct RSI_Recv_Data recv_data;

		bool stop_rsi;
		bool connection_alive;

		// Tool configuration
		double toolmass, toolmassold;
		KDL::Vector toolcom, toolcomold;
		bool toolswitching;

		// Velocities for Cyclic Position Interface
		std::vector<double> max_vel, max_acc, minj, maxj;

		RPI::RoundRobinLog<double>* dumpMsrPos;

		KR_to_SI_factors conv_factors;
	};

} /* namespace kuka_kr */

#endif /* RSI_DRIVER_HPP_ */
