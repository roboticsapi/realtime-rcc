/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "Fri.hpp"
#include <rcc/DeviceFactory.hpp>
#include <rtt/extras/PeriodicActivity.hpp>

namespace kuka_iiwa
{
	FRIDriver::FRIDriver(std::string name, RPI::parameter_t parameters) :
			TaskContext("FRI_" + name, PreOperational), IiwaDevice(name, parameters), deviceState(
					RPI::DeviceState::OFFLINE), robotarm::CyclicPositionRobotArm(7, 50)
	{
		isEstop = false;
		isStart = true;
		newControlMode = false;
		newControlModeConfirmed = true;
		toolswitching = false;

		toolcomold = toolcom = KDL::Vector(0, 0, 0);
		// Set tool to 0 - must be overridden
		toolmassold = toolmass = 1;

		krc_desired_controlscheme = 1;	// standard control mode is position mode
		krc_desired_mass = 0;
		krc_mode_error = 0;
		krc_mode_finished = true;
		krc_switch_state = INACTIVE;
		current_mode = 1;

		//get parameters
		deviceName = name;
		fri_port = getParameterT<int>("fri_port", DEFAULT_FRI_PORTID);
		control_mode_port = getParameterT<int>("control_mode_port",DEFAULT_CONTROL_MODE_PORTID);
		udpDeviceName = getParameter("fri_device_name");
		udpControlModeDevice = (const char*) getParameter("control_mode_device_name").c_str();

		_controllerAddr.sin_family = AF_INET;
		_controllerAddr.sin_port = control_mode_port;
		//_controllerAddr.sin_addr.s_addr = (in_addr_t)*"172.31.1.103";

		//resize and set values
		//dumpMsrPos = new RPI::RoundRobinLog<double>[kr_rsi_axiscount];
		max_vel.resize(iiwa_fri_axiscount);
		max_acc.resize(iiwa_fri_axiscount);
		minj.resize(iiwa_fri_axiscount);
		maxj.resize(iiwa_fri_axiscount);
		for (int i = 0; i < iiwa_fri_axiscount; ++i)
		{
			maxj[i] = max_vel[i] = max_acc[i] = std::numeric_limits<double>::max();
			minj[i] = -std::numeric_limits<double>::max();
		}

		//initialize ClientApplication with an UdpConnection and FriDriver Object itself as LBRClient
		//the ClientApplication controls the FRI connection and the functions which are called
		app = new ClientApplication(connection, *this);

		controlModeTime = RTT::os::TimeService::Instance()->getNSecs() / 1e9;

		this->setActivity(new RTT::Activity(RTT::os::HighestPriority, IIWA_FRI_CYCLE_TIME));
	}

	FRIDriver::~FRIDriver()
	{
		// send connection close message and wait till confirmation is received
		sendConnectionCloseMessage();
		while(!newControlModeConfirmed){
			receiveControlModeConfirmation();
		}

		// then close all sockets and connections and delete arrays
		app->disconnect();
		connection.close();
		udp->close_socket();

		delete (app);

		stop();
		cleanup();

		RTT::log(RTT::Info) << "FRI Driver of Device "<< deviceName <<" shut down!" << RTT::endlog();
	}

	bool FRIDriver::startHook()
	{
		return true;
	}
	bool FRIDriver::configureHook()
	{
		//try to connect to the fri application on the robot control
		if (!app->connect(fri_port, udpDeviceName.c_str()))
		{
			RTT::log(RTT::Critical) << "FRI Device " << udpDeviceName << " over UDP on port " << fri_port << " not found"
					<< RTT::endlog();
			return false;
		} else
		{
			RTT::log(RTT::Info) << "FRI Device " << udpDeviceName << " on port " << fri_port << " connected" << RTT::endlog();
		}

		if (!udp.fetchInstance(udpControlModeDevice) || !udp->initialize_socket())
		{
			RTT::log(RTT::Critical) << "UDP Device " << udpControlModeDevice << " on port " << control_mode_port << " not found"
					<< RTT::endlog();
			return false;
		}

		return true;
	}

	void FRIDriver::updateHook()
	{
		// if a new control mode is not confirmed yet try to receive it
		if (!newControlModeConfirmed)
		{
			receiveControlModeConfirmation();
		}
		// try to receive a heartbeat message in every cycle
		receiveControlModeHeartbeat();

		// send a heartbeat message every IIWA_CONTROL_MODE_HEARTBEAT_CYCLE_TIME seconds to keep UDP connection alive
		RTT::os::TimeService::Seconds new_secs = RTT::os::TimeService::Instance()->getNSecs() / 1e9;;
		if (new_secs - controlModeTime >= IIWA_CONTROL_MODE_HEARTBEAT_CYCLE_TIME)
		{
			sendHeartbeat();
			controlModeTime = new_secs;
		}

		// if new control mode is set -> send message to robot control
		if (newControlMode)
		{
			sendControlMode();
			controlModeTime = new_secs;
		}

		// do step in every cycle
		app->step();

		//check if spring is relaxed in every cycle
		isSpringRelaxed();
	}

	void FRIDriver::onStateChange(ESessionState oldState, ESessionState newState)
	{
		data.session_state = newState;

		// print current state on console
		switch (newState)
		{
		case COMMANDING_ACTIVE:
			RTT::log(RTT::Info) << "IIWA in new State: COMMANDING_ACTIVE" << RTT::endlog();
			break;
		case COMMANDING_WAIT:
			RTT::log(RTT::Info) << "IIWA in new State: COMMANDING_WAIT" << RTT::endlog();
			break;
		case IDLE:
			deviceState = DeviceState::OFFLINE;
			RTT::log(RTT::Info) << "IIWA in new State: IDLE" << RTT::endlog();
			break;
		case MONITORING_READY:
			RTT::log(RTT::Info) << "IIWA in new State: COMMANDING_WAIT" << RTT::endlog();
			break;
		case MONITORING_WAIT:
			RTT::log(RTT::Info) << "IIWA in new State: MONITORING_WAIT" << RTT::endlog();
			break;
		}
	}

	//method is called in state COMMANDING_ACTIVE
	void FRIDriver::command()
	{
		//save old values
		double pos_old[iiwa_fri_axiscount];
		memcpy(pos_old, data.pos_cur, iiwa_fri_axiscount * sizeof(double));

		//get new values
		if (robotState().getIpoJointPosition() != NULL)
			memcpy(data.ipo_cur, robotState().getIpoJointPosition(), iiwa_fri_axiscount * sizeof(double));
		if (robotState().getMeasuredJointPosition() != NULL)
			memcpy(data.pos_cur, robotState().getMeasuredJointPosition(), iiwa_fri_axiscount * sizeof(double));
		if (robotState().getMeasuredTorque() != NULL)
			memcpy(data.torque_cur, robotState().getMeasuredTorque(), iiwa_fri_axiscount * sizeof(double));

		// calculate velocity and acceleration
		RTT::os::TimeService::nsecs last_nsecs, new_nsecs, delta_nsecs;
		last_nsecs = data.timestamp;
		data.timestamp = new_nsecs = RTT::os::TimeService::Instance()->getNSecs();
		delta_nsecs = new_nsecs - last_nsecs;

		for (int i = 0; i < iiwa_fri_axiscount; ++i)
		{
			double new_vel = (data.pos_cur[i] - pos_old[i]) / (delta_nsecs / 1.0e9);
			double new_acc = (new_vel - data.vel_cur[i]) / (delta_nsecs / 1.0e9);

			data.vel_cur[i] = new_vel;
			data.acc_cur[i] = new_acc;
		}

		//save information
		data.conn_quality = robotState().getConnectionQuality();
		data.safety_state = robotState().getSafetyState();
		data.op_mode = robotState().getOperationMode();
		data.drive_state = robotState().getDriveState();
		data.control_mode = robotState().getControlMode();
		//current_mode = (IIWA_CTRL) data.control_mode;

		deviceState = DeviceState::OPERATIONAL;

		//get new positions from RCC and set them on IIWA
		double cmdPos[7];
		getValuesToCommand(cmdPos, NULL);
		robotCommand().setJointPosition(cmdPos);
	}

	//method is called in state MONITORING_READY and MONITORING_WAIT
	void FRIDriver::monitor()
	{
		LBRClient::monitor();

		//get new values
		if (robotState().getIpoJointPosition() != NULL)
			memcpy(data.ipo_cur, robotState().getIpoJointPosition(), 7 * sizeof(double));
		if (robotState().getMeasuredJointPosition() != NULL)
			memcpy(data.pos_cur, robotState().getMeasuredJointPosition(), 7 * sizeof(double));
		if (robotState().getMeasuredTorque() != NULL)
			memcpy(data.torque_cur, robotState().getMeasuredTorque(), 7 * sizeof(double));

		//save information
		data.conn_quality = robotState().getConnectionQuality();
		data.safety_state = robotState().getSafetyState();
		data.op_mode = robotState().getOperationMode();
		data.drive_state = robotState().getDriveState();

		if (data.session_state == ESessionState::MONITORING_READY || data.drive_state == EDriveState::OFF
				|| data.safety_state != ESafetyState::NORMAL_OPERATION)
		{

			for (int i = 0; i < iiwa_fri_axiscount; i++)
			{
				setJointPositionStatic(i, data.pos_cur[i]);
			}
			deviceState = RPI::DeviceState::SAFE_OPERATIONAL;
		} else
		{
			deviceState = RPI::DeviceState::OPERATIONAL;
		}
	}

	//receive current Heartbeats and Error information
	void FRIDriver::receiveControlModeHeartbeat()
	{
		char rcvbuffer[1];
		ssize_t received;

		//receive heartbeat from Sunrise controller
		int sockAddrSize = sizeof(struct sockaddr_in);
		received = udp->recvfrom(rcvbuffer, sizeof(rcvbuffer), MSG_DONTWAIT, (struct sockaddr *) &_controllerAddr,
				(socklen_t *) &sockAddrSize);
		if (received >= 0)
		{
			isStart = false;
		}
	}

	void FRIDriver::sendControlMode()
	{
		// union variable for control mode parameter transmission
		union
		{
			double d[IIWA_CONTROL_MODE_PACKET_SIZE];
			char buffer[sizeof(double) * IIWA_CONTROL_MODE_PACKET_SIZE]; // 296 = 37 * sizeof(double) -> 37 double parameters (1 contolMode, 14 joint parameters, 12 cartesian parameters)
		} myUnion;

		//send desired control mode
		switch (krc_desired_controlscheme)
		{
		case 1:
			//Pos Control Mode
			myUnion.d[0] = 0.0;
			break;
		case 2:
			//Cart Imp Mode
			myUnion.d[0] = 1.0;
			break;
		case 3:
			//Joint Imp Mode
			myUnion.d[0] = 2.0;
			break;
		case 4:
			//Cart Sin Imp Mode
			//RTT::log(RTT::Info) <<  "\n\nCART SIN IMP MODE" << cartsinamp[0] << " " << cartsinfreq[0]<< RTT::endlog();
			myUnion.d[0] = 3.0;
			break;
		default:
			myUnion.d[0] = 0.0;
			break;
		}

		// copy damping, stiffness and tool data for transmission into union variable
		for (int i = 1; i < IIWA_CONTROL_MODE_PACKET_SIZE; ++i)
		{
			if (i < 8)
			{
				myUnion.d[i] = jntstiff[i - 1];
			} else if (i < 15)
			{
				myUnion.d[i] = jntdamp[i - 8];
			} else if (i < 21)
			{
				myUnion.d[i] = cartstiff[i - 15];
			} else if (i < 27)
			{
				myUnion.d[i] = cartdamp[i - 21];
			} else if (i < 33)
			{
				myUnion.d[i] = cartsinamp[i - 27];
			} else if (i < 39)
			{
				myUnion.d[i] = cartsinfreq[i - 33];
			}else if (i < 40)
			{
				//RTT::log(RTT::Info) << "ToolMass: "<< krc_desired_mass  << RTT::endlog();
				myUnion.d[i] = krc_desired_mass;
			} else if (i < 43)
			{
				//RTT::log(RTT::Info) << "COM Vector: " << krc_desired_com.p.data[i - 28] << RTT::endlog();
				//RTT::log(RTT::Info) << "TCP Vector: "<< krc_desired_tcp.p.data[i-28]  << RTT::endlog();
				myUnion.d[i] = krc_desired_com.p.data[i - 40];
			} else if (i < 46)
			{
				//RTT::log(RTT::Info) << "COM Rot: " << krc_desired_com.M.GetRot().data[i - 31] << RTT::endlog();
				//RTT::log(RTT::Info) << "tcp Rot: "<< krc_desired_tcp.M.GetRot().data[i-31]  << RTT::endlog();
				myUnion.d[i] = krc_desired_com.M.GetRot().data[i - 43];
			} else
			{
				//RTT::log(RTT::Info) << "MOI: " << krc_desired_moi.data[i - 34] << RTT::endlog();
				myUnion.d[i] = krc_desired_moi.data[i - 46];
			}
		}

		if (udp.getDevice() != NULL)
		{
			int sent = udp->sendto(const_cast<char*>(myUnion.buffer), sizeof(myUnion.buffer), 0,
					(struct sockaddr *) &_controllerAddr, sizeof(_controllerAddr));
			newControlMode = false;
			newControlModeConfirmed = false;
		} else
		{
			RTT::log(RTT::Error) << "Error while sending Control Mode - UDP not open!" << RTT::endlog();
		}

	}
	void FRIDriver::sendHeartbeat()
	{
		union
		{
			double d[IIWA_CONTROL_MODE_PACKET_SIZE];
			char buffer[sizeof(double) * IIWA_CONTROL_MODE_PACKET_SIZE];
		} myUnion;
		myUnion.d[0] = krc_desired_controlscheme - 1;

		if (udp.getDevice() != NULL)
		{
			int sent = udp->sendto(const_cast<char*>(myUnion.buffer), sizeof(myUnion.buffer), 0, (struct sockaddr *) &_controllerAddr,
					sizeof(_controllerAddr));
		} else
		{
			RTT::log(RTT::Error) << "Error while sending Control Mode - UDP not open!" << RTT::endlog();
		}
	}
	void FRIDriver::receiveControlModeConfirmation()
	{
		//receive response
		int sockAddrSize = sizeof(struct sockaddr_in);
		ssize_t received;
		char rcvbuffer[1];

		received = udp->recvfrom(rcvbuffer, sizeof(rcvbuffer), MSG_DONTWAIT, (struct sockaddr *) &_controllerAddr,
				(socklen_t *) &sockAddrSize);
		if (received >= 0)
		{
			newControlModeConfirmed = true;
			krc_mode_finished = true;
		}
	}

	void FRIDriver::sendConnectionCloseMessage(){
		union
		{
			double d[IIWA_CONTROL_MODE_PACKET_SIZE];
			char buffer[sizeof(double) * IIWA_CONTROL_MODE_PACKET_SIZE];
		} myUnion;
		myUnion.d[0] = 255.0;

		if (udp.getDevice() != NULL)
		{
			int sent = udp->sendto(const_cast<char*>(myUnion.buffer), sizeof(myUnion.buffer), 0, (struct sockaddr *) &_controllerAddr,
					sizeof(_controllerAddr));
			newControlModeConfirmed = false;
			RTT::log(RTT::Info) << "Connection Close sent!" << RTT::endlog();

		} else
		{
			RTT::log(RTT::Error) << "Error while sending Connection close - UDP not open!" << RTT::endlog();
		}
	}

	//method is called in state COMMANDING_WAIT
	void FRIDriver::waitForCommand()
	{
		LBRClient::waitForCommand();
	}

	double FRIDriver::getMaximumAcceleration(int joint) const
	{
		return max_acc[joint];
	}
	double FRIDriver::getMaximumVelocity(int joint) const
	{
		return max_vel[joint];
	}

	int FRIDriver::getJointError(int joint)
	{
		if (isEstop)
			return 1;
		return 0;
	}
	int FRIDriver::getJointCount() const
	{
		return iiwa_fri_axiscount;
	}

	robotarm::JointPositionError FRIDriver::checkJointPosition(int joint, double position)
	{
		if (position != position)
			return robotarm::JP_INVALID;

		if (joint < 0 || joint >= iiwa_fri_axiscount)
			return robotarm::JP_INVALID;

		if (joint >= minj.size() || joint >= maxj.size())
			return robotarm::JP_OK;

		if (position < minj[joint] || position > maxj[joint])
			return robotarm::JP_OUTOFRANGE;

		return robotarm::JP_OK;
	}
	double FRIDriver::getMeasuredJointPosition(int joint)
	{
		//std::cout << data.pos_cur[joint] << std::endl;
		if (joint >= 0 && joint < iiwa_fri_axiscount)
			return data.pos_cur[joint];
		return 0;
	}
	double FRIDriver::getMeasuredJointVelocity(int joint)
	{
		if (joint >= 0 && joint < iiwa_fri_axiscount)
			return data.vel_cur[joint];
		return 0;
	}
	double FRIDriver::getMeasuredJointAcceleration(int joint)
	{
		if (joint >= 0 && joint < iiwa_fri_axiscount)
			return data.acc_cur[joint];
		return 0;
	}
	void FRIDriver::setToolCOM(KDL::Vector com, int axis)
	{
		toolcom = com;
		toolswitching = true;
	}
	void FRIDriver::setToolMass(double mass, int axis)
	{
		toolmass = mass;
		toolswitching = true;
	}
	void FRIDriver::setToolMOI(KDL::Vector moi, int axis)
	{
		toolmoi = moi;
		// not yet supported
		return;
	}
	bool FRIDriver::getToolFinished(int axis) const
	{
		return !toolswitching;
	}
	int FRIDriver::getToolError(int axis) const
	{
		return 0;
	}

	std::set<std::string> FRIDriver::getMutableParameters() const
	{
		return std::set<std::string>();
	}
	void FRIDriver::updateParameters()
	{

	}

	RPI::DeviceState FRIDriver::getDeviceState() const
	{
		return deviceState;
	}

	void FRIDriver::setEStop(bool estop)
	{
		this->isEstop = estop;
	}

	void FRIDriver::setJointConfig(int axis, double max_vel, double max_acc, double minj, double maxj)
	{
		if (axis < 0 || axis >= iiwa_fri_axiscount)
			return;

		this->max_vel[axis] = max_vel;
		this->max_acc[axis] = max_acc;
		this->minj[axis] = minj;
		this->maxj[axis] = maxj;

	}

	bool FRIDriver::setKRCDesiredControlScheme(int scheme)
	{
		if (scheme != krc_desired_controlscheme)
		{
			krc_desired_controlscheme = scheme;
			newControlMode = true;
			return true;
		} else
		{
			krc_mode_finished = true;
			newControlModeConfirmed = true;
			return false;
		}
	}

	int FRIDriver::getKRCModeError() const
	{
		return krc_mode_error;
	}

	bool FRIDriver::getKRCModeFinished() const
	{
		return krc_mode_finished;
	}

	void FRIDriver::setKRCModeFinished(bool state)
	{
		krc_mode_finished = state;
		if(!newControlMode && newControlModeConfirmed)
			krc_mode_finished = true;
	}
	bool FRIDriver::setKRCToolTCP(const KDL::Frame& tcp)
	{
		if (krc_desired_tcp != tcp)
		{
			krc_desired_tcp = tcp;
			return true;
		} else
		{
			return false;
		}
	}

	bool FRIDriver::setKRCToolCOM(const KDL::Frame& com)
	{

		if (krc_desired_com != com)
		{
			RTT::log(RTT::Info) << "!!!! setKRCToolCOM: " << com.p.data[1] << RTT::endlog();
			krc_desired_com = com;
			return true;
		} else
		{
			return false;
		}
	}

	bool FRIDriver::setKRCToolMOI(const KDL::Vector& moi)
	{
		if (krc_desired_moi != moi)
		{
			krc_desired_moi = moi;
			return true;
		} else
		{
			return false;
		}
	}

	bool FRIDriver::setKRCToolMass(double mass)
	{
		if (krc_desired_mass != mass)
		{
			//RTT::log(RTT::Info) << "setKRCToolMass: "<< mass  << RTT::endlog();
			krc_desired_mass = mass;
			return true;
		} else
		{
			return false;
		}
	}

	int FRIDriver::getControlScheme()
	{
		if (app == NULL)
			return 1;
		return (int) data.control_mode; //(IIWA_CTRL) fri->getCurrentControlScheme();
	}
	int FRIDriver::getState()
	{
		if (app == NULL)
			return 0;
		return (int) data.drive_state;
	}
	int FRIDriver::getQuality()
	{
		if (app == 0)
			return 0;
		return (int) data.conn_quality;
	}
	bool FRIDriver::getPower() const
	{
		if (app == 0)
			return false;
		return true;
	}
	bool FRIDriver::isSpringRelaxed()
	{
		bool relaxed = true;
		for (int i = 0; i < LBR_MNJ; i++)
		{
			relaxed &= fabs(getMeasuredJointPosition(i) - getCommandedJointPosition(i)) < 0.01;
		}
		if(!relaxed){
			krc_mode_error = 3;
		}
		else{
			krc_mode_error = 0;
		}
		return relaxed;
	}
	float FRIDriver::getAxisTqAct(int axis)
	{
		return 0.0;
	}
	bool FRIDriver::loadDataChanged()
	{
		return false;
	}
	bool FRIDriver::controlSchemeChanged()
	{
		return false;
	}

	void FRIDriver::setDigitalOut(int port, bool value)
	{
	}
	bool FRIDriver::getDigitalOut(int port)
	{
		return false;
	}
	bool FRIDriver::getDigitalIn(int port)
	{
		return false;
	}
	void FRIDriver::setAnalogOut(int port, double value)
	{

	}
	double FRIDriver::getAnalogOut(int port)
	{
		return 0.0;
	}
	double FRIDriver::getAnalogIn(int port)
	{
		return 0.0;
	}

	unsigned int FRIDriver::getNumDigitalIn() const
	{
		return 1;
	}
	unsigned int FRIDriver::getNumDigitalOut() const
	{
		return 1;
	}
	unsigned int FRIDriver::getNumAnalogIn() const
	{
		return 1;
	}
	unsigned int FRIDriver::getNumAnalogOut() const
	{
		return 1;
	}

	int FRIDriver::getKRCControlScheme()
	{
		int newstrategy = 0;
		switch (current_mode)
		{
		case 1:
			newstrategy = 10;
			break;
		case 2:
			newstrategy = 20;
			break;
		case 3:
			newstrategy = 30;
			break;
		default:
			// illegal new strategy, abort update
			current_mode = this->getControlScheme();
			break;
		}
		return newstrategy;
	}

	FRIDriver* FRIDriver::createDevice(string name, parameter_t parameters)
	{
		FRIDriver* ret = new FRIDriver(name, parameters);
		ret->configure();
		ret->start();
		return ret;
	}
	void FRIDriver::setJntImpStiffness(float stiff, int axis)
	{
		if (axis >= 0 && axis < LBR_MNJ)
			jntstiff[axis] = stiff;
	}

	void FRIDriver::setJntImpDamping(float damp, int axis)
	{
		if (axis >= 0 && axis < LBR_MNJ)
			jntdamp[axis] = damp;
	}

	void FRIDriver::setJntImpAddTorque(float trq, int axis)
	{
		if (axis >= 0 && axis < LBR_MNJ)
			jntaddtrq[axis] = trq;
	}

	void FRIDriver::setCartImpStiffness(float x, float y, float z, float a, float b, float c)
	{
		cartstiff[0] = x;
		cartstiff[1] = y;
		cartstiff[2] = z;
		cartstiff[3] = a;
		cartstiff[4] = b;
		cartstiff[5] = c;
	}

	void FRIDriver::setCartImpDamping(float x, float y, float z, float a, float b, float c)
	{
		cartdamp[0] = x;
		cartdamp[1] = y;
		cartdamp[2] = z;
		cartdamp[3] = a;
		cartdamp[4] = b;
		cartdamp[5] = c;
	}

	void FRIDriver::setCartImpAddTcpFT(float x, float y, float z, float a, float b, float c)
	{
		cartaddtcpft[0] = x;
		cartaddtcpft[1] = y;
		cartaddtcpft[2] = z;
		cartaddtcpft[3] = a;
		cartaddtcpft[4] = b;
		cartaddtcpft[5] = c;
	}

	void FRIDriver::setCartSinImpAmplitude(float x, float y, float z, float a, float b, float c){
		cartsinamp[0] = x;
		cartsinamp[1] = y;
		cartsinamp[2] = z;
		cartsinamp[3] = a;
		cartsinamp[4] = b;
		cartsinamp[5] = c;
	}
	void FRIDriver::setCartSinImpFrequency(float x, float y, float z, float a, float b, float c){
		cartsinfreq[0] = x;
		cartsinfreq[1] = y;
		cartsinfreq[2] = z;
		cartsinfreq[3] = a;
		cartsinfreq[4] = b;
		cartsinfreq[5] = c;
	}

	float FRIDriver::getJntImpStiffness(int axis)
	{
		return 0.0;
	}
	float FRIDriver::getJntImpDamping(int axis)
	{
		return 0.0;
	}
	float FRIDriver::getJntImpAddTorque(int axis)
	{
		return 0.0;
	}
	void FRIDriver::getCartPosition(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		double* pos = data.pos_cur; //fri->getMsrCartPosition();
		x = pos[3];
		y = pos[7];
		z = pos[11];
		double ra, rb, rc;
		KDL::Rotation rot(pos[0], pos[1], pos[2], pos[4], pos[5], pos[6], pos[8], pos[9], pos[10]);
		rot.GetRPY(rc, rb, ra);
		a = ra;
		b = rb;
		c = rc;
	}
	void FRIDriver::getCartImpStiffness(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartstiff[0];
		y = cartstiff[1];
		z = cartstiff[2];
		a = cartstiff[3];
		b = cartstiff[4];
		c = cartstiff[5];
	}
	void FRIDriver::getCartImpDamping(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartdamp[0];
		y = cartdamp[1];
		z = cartdamp[2];
		a = cartdamp[3];
		b = cartdamp[4];
		c = cartdamp[5];
	}
	void FRIDriver::getCartImpAddTcpFT(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartaddtcpft[0];
		y = cartaddtcpft[1];
		z = cartaddtcpft[2];
		a = cartaddtcpft[3];
		b = cartaddtcpft[4];
		c = cartaddtcpft[5];
	}

	void FRIDriver::getCartSinImpAmplitude(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartsinamp[0];
		y = cartsinamp[1];
		z = cartsinamp[2];
		a = cartsinamp[3];
		b = cartsinamp[4];
		c = cartsinamp[5];
	}

	void FRIDriver::getCartSinImpFrequency(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		x = cartsinfreq[0];
		y = cartsinfreq[1];
		z = cartsinfreq[2];
		a = cartsinfreq[3];
		b = cartsinfreq[4];
		c = cartsinfreq[5];
	}

	float FRIDriver::getAxisTqEst(int axis)
	{
		if (axis < 0 || axis >= LBR_MNJ)
			return 0;
		//return fri->getMsrEstExtJntTrq()[axis];
		return 0.0;
	}

	float* FRIDriver::getTcpEst()
	{
		//		return fri->getMsrExtTcpFT();
		return NULL;
	}

	float* FRIDriver::getMsrJointVel()
	{
		//return msrjoint_v;
		return (float*) data.vel_cur;
	}

	float* FRIDriver::getMsrJointAcc()
	{
		//return msrjoint_a;
		return (float*) data.acc_cur;
	}
}
