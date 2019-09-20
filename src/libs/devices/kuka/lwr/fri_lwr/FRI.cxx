/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "FRI.hpp"
#include <boost/algorithm/string.hpp>
#include <rtt/extras/PeriodicActivity.hpp>
#include <rcc/DeviceFactory.hpp>

#ifndef RTNET
#define FRI_CYCLE_TIME 0.02
#else
#define FRI_CYCLE_TIME 0.00005
#endif

namespace kuka_lwr
{
	using namespace RPI;
	using namespace std;

	FRIDriver::FRIDriver(string name, parameter_t parameters) :
			TaskContext("FRI_" + name, PreOperational), robotarm::CyclicPositionRobotArm(7, 50), LwrDevice(name,
					parameters), deviceMutex(), isEStop(false), deviceState(RPI::DeviceState::OFFLINE)
	{
		string ip;
		int port = 0;
		// preinitialize values, configuration might not be set
		id_handshake = 15;
		id_communication = 14;
		parameter_t::iterator fres;

		// aquire typekits
		TypeKit* inttk = TypeKits::getInstance()->getTypeKit<int>();
		TypeKit* doubletk = TypeKits::getInstance()->getTypeKit<double>();
		TypeKit* frametk = TypeKits::getInstance()->getTypeKit<KDL::Frame>();
		TypeKit* vectortk = TypeKits::getInstance()->getTypeKit<KDL::Vector>();

		fres = parameters.find("ip");
		if (fres != parameters.end())
			ip = fres->second;

		fres = parameters.find("port");
		if (fres != parameters.end())
			inttk->fromString(&port, fres->second);

		fres = parameters.find("handshake");
		if (fres != parameters.end())
			inttk->fromString(&id_handshake, fres->second);

		fres = parameters.find("communication");
		if (fres != parameters.end())
			inttk->fromString(&id_communication, fres->second);

		fres = parameters.find("mass");
		if (fres != parameters.end())
			doubletk->fromString(&krc_desired_mass, fres->second);

		fres = parameters.find("tcp");
		if (fres != parameters.end())
			frametk->fromString(&krc_desired_tcp, fres->second);
		fres = parameters.find("com");
		if (fres != parameters.end())
			frametk->fromString(&krc_desired_com, fres->second);
		fres = parameters.find("moi");
		if (fres != parameters.end())
			vectortk->fromString(&krc_desired_moi, fres->second);

		// Inputs configured?
		fres = parameters.find("iportsdig");
		if (fres != parameters.end())
		{
			readioconfig(atoi(fres->second.c_str()), parameters, "iportdig", IPortsDig);
		}
		fres = parameters.find("iportsan");
		if (fres != parameters.end())
		{
			readioconfig(atoi(fres->second.c_str()), parameters, "iportan", IPortsAn);
		}

		// Outputs configured?
		fres = parameters.find("oportsdig");
		if (fres != parameters.end())
		{
			readioconfig(atoi(fres->second.c_str()), parameters, "oportdig", OPortsDig);
		}
		fres = parameters.find("oportsan");
		if (fres != parameters.end())
		{
			readioconfig(atoi(fres->second.c_str()), parameters, "oportan", OPortsAn);
		}

		// initialize values
		for (int i = 0; i < LBR_MNJ; i++)
		{
			joints[i] = 0;
			old_msrjoint[i] = 0;
			old_msrjoint_v[i] = 0;
			msrjoint_a[i] = 0;
			msrjoint_v[i] = 0;

			jntstiff[i] = 1000;
			jntdamp[i] = 0.7;
			jntaddtrq[i] = 0;

			jointvel[i] = 0;
			jointacc[i] = 0;
		}

		cartstiff[0] = cartstiff[1] = cartstiff[2] = 2000;
		cartstiff[3] = cartstiff[4] = cartstiff[5] = 200;
		for (int i = 0; i < FRI_CART_VEC; i++)
		{
			cartdamp[i] = 0.7;
			cartpos[i] = 0; // should never be actually used, kinematic should produce new values from joints[]
			cartaddtcpft[i] = 0;
		}

		this->setActivity(new RTT::extras::PeriodicActivity(RTT::os::HighestPriority, FRI_CYCLE_TIME));
//		this->setActivity(new RTT::NonPeriodicActivity(RTT::os::HighestPriority));
		friRemote* remote = new friRemote(port);

		fri = remote;
		failed = 0;

		krc_switch_state = INACTIVE;
		krc_desired_controlscheme = fri->getCurrentControlScheme();
		krc_mode_error = 0;
		krc_mode_finished = true;

		krc_loadDataValid = false;

		monitorModeState = 0;

		dumpMsrPos = new RPI::RoundRobinLog<double>[LBR_MNJ];
		dumpMsrVel = new RPI::RoundRobinLog<double>[LBR_MNJ];
		dumpFriPos = new RPI::RoundRobinLog<double>[LBR_MNJ];

		for (const auto& dumper : CyclicPositionRobotArm::getCrashDumpers())
			addCrashDumper(dumper);

		for (int i = 0; i < LBR_MNJ; i++)
		{
			std::stringstream joint;
			joint << "Joint " << i;
			dumpMsrPos[i].setName(joint.str() + "|Position|Measured");
			addCrashDumper(&dumpMsrPos[i]);
			dumpMsrVel[i].setName(joint.str() + "|Velocity|Measured");
			addCrashDumper(&dumpMsrVel[i]);
			dumpFriPos[i].setName(joint.str() + "|Position|FRI");
			addCrashDumper(&dumpFriPos[i]);
		}

	}

	double FRIDriver::getMaximumAcceleration(int joint) const
	{
		return 4;
	}

	double FRIDriver::getMaximumVelocity(int joint) const
	{
		return 112.5 / 180.0 * KDL::PI;
	}

	void FRIDriver::readioconfig(int portcount, parameter_t parameters, string prefix, map<int, string>& portlist)
	{
		parameter_t::iterator fres;
		for (int i = 0; i < portcount; i++)
		{
			stringstream sname;
			string ioname;

			sname << prefix << i << "name";
			fres = parameters.find(sname.str());
			if (fres != parameters.end())
				ioname = fres->second;
			else
				ioname = "unnamed";

			portlist[i] = ioname;
		}
	}

	FRIDriver::~FRIDriver()
	{
		delete fri;
		delete[] dumpMsrPos;
		dumpMsrPos = 0;
		delete[] dumpMsrVel;
		dumpMsrVel = 0;
		delete[] dumpFriPos;
		dumpFriPos = 0;
		fri = 0;
	}

	set<string> FRIDriver::getMutableParameters() const
	{
		return set<string>();
	}

	void FRIDriver::updateParameters()
	{

	}

	IOPortState FRIDriver::getIOPortState()
	{
		IOPortState state;
		if (!fri)
			return state;

		for (unsigned int i = 0; i < IPortsDig.size(); i++)
		{
			IOPortStateDig istate;
			istate.name = IPortsDig[i];
			istate.value = fri->getFrmKRLBool(i);
			state.ioDigIn.push_back(istate);
		}
		for (unsigned int i = 0; i < IPortsAn.size(); i++)
		{
			IOPortStateAn istate;
			istate.name = IPortsAn[i];
			istate.value = fri->getFrmKRLReal(i);
			state.ioAnIn.push_back(istate);
		}
		for (unsigned int i = 0; i < OPortsDig.size(); i++)
		{
			IOPortStateDig istate;
			istate.name = OPortsDig[i];
			istate.value = fri->getToKRLBool(i);
			state.ioDigOut.push_back(istate);
		}
		for (unsigned int i = 0; i < OPortsAn.size(); i++)
		{
			IOPortStateAn istate;
			istate.name = OPortsAn[i];
			istate.value = fri->getToKRLReal(i);
			state.ioAnOut.push_back(istate);
		}

		return state;
	}

	FRIDriver* FRIDriver::createDevice(string name, parameter_t parameters)
	{
		FRIDriver* ret = new FRIDriver(name, parameters);
		ret->configure();
		ret->start();
		return ret;
	}

	bool FRIDriver::checkJointPosition(float j, int axis)
	{
		if (j != j) // value is NaN
			return false;
		if (axis % 2 == 0)
		{ // even axes can move +- 170°
			if (j > KDL::PI / 180 * 170 || j < -KDL::PI / 180 * 170)
				return false;
		} else
		{ // odd axes only allow +-120°
			if (j > KDL::PI / 180 * 120 || j < -KDL::PI / 180 * 120)
				return false;
		}
		return true;
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

	float FRIDriver::getJntImpStiffness(int axis)
	{
		if (axis >= 0 && axis < LBR_MNJ)
			return jntstiff[axis];
		return 0;
	}
	float FRIDriver::getJntImpDamping(int axis)
	{
		if (axis >= 0 && axis < LBR_MNJ)
			return jntdamp[axis];
		return 0;
	}
	float FRIDriver::getJntImpAddTorque(int axis)
	{
		if (axis >= 0 && axis < LBR_MNJ)
			return jntaddtrq[axis];
		return 0;
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

	void FRIDriver::getCartPosition(float& x, float& y, float& z, float& a, float& b, float& c)
	{
		float* pos = fri->getMsrCartPosition();
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

	float* FRIDriver::getJointPosition()
	{
		return fri->getMsrMsrJntPosition();
	}
	void FRIDriver::getJointPosition(float& j1, float& j2, float& j3, float& j4, float& j5, float& j6, float& j7)
	{
		fri->getMsrMsrJntPosition(j1, j2, j3, j4, j5, j6, j7);
	}

	float FRIDriver::getAxisTqAct(int axis)
	{
		if (axis < 0 || axis >= LBR_MNJ)
			return 0;
		return fri->getMsrJntTrq()[axis];
	}

	float* FRIDriver::getAxisAct()
	{
		return fri->getMsrJntTrq();
	}

	float FRIDriver::getAxisTqEst(int axis)
	{
		if (axis < 0 || axis >= LBR_MNJ)
			return 0;
		return fri->getMsrEstExtJntTrq()[axis];
	}

	void FRIDriver::getAxisEst(float& j1, float& j2, float& j3, float& j4, float& j5, float& j6, float& j7)
	{
		fri->getMsrEstExtJntTrq(j1, j2, j3, j4, j5, j6, j7);
	}

	void FRIDriver::getAxisAct(float& j1, float& j2, float& j3, float& j4, float& j5, float& j6, float& j7)
	{
		fri->getMsrJntTrq(j1, j2, j3, j4, j5, j6, j7);
	}

	float* FRIDriver::getAxisEst()
	{
		return fri->getMsrEstExtJntTrq();
	}

	float* FRIDriver::getTcpEst()
	{
		return fri->getMsrExtTcpFT();
	}

	float* FRIDriver::getMsrJointVel()
	{
		return msrjoint_v;
	}

	float* FRIDriver::getMsrJointAcc()
	{
		return msrjoint_a;
	}

	bool FRIDriver::getFriFrmBool(int id)
	{
		if (fri == 0)
			return false;
		return fri->getFrmKRLBool(id);
	}

	float FRIDriver::getFriFrmReal(int id)
	{
		if (fri == 0)
			return false;
		return fri->getFrmKRLReal(id);
	}

	int FRIDriver::getFriFrmInt(int id)
	{
		if (fri == 0)
			return false;
		return fri->getFrmKRLInt(id);
	}

	bool FRIDriver::getFriToBool(int id)
	{
		if (fri == 0)
			return false;
		return fri->getToKRLBool(id);
	}

	float FRIDriver::getFriToReal(int id)
	{
		if (fri == 0)
			return false;
		return fri->getToKRLReal(id);
	}

	int FRIDriver::getFriToInt(int id)
	{
		if (fri == 0)
			return false;
		return fri->getToKRLInt(id);
	}

	int FRIDriver::getControlScheme()
	{
		if (fri == 0)
			return 1;
		return (int) fri->getCurrentControlScheme();
	}
	int FRIDriver::getState()
	{
		if (fri == 0)
			return 0;
		return (int) fri->getState();
	}
	int FRIDriver::getQuality()
	{
		if (fri == 0)
			return 0;
		return (int) fri->getQuality();
	}

	bool FRIDriver::getPower() const
	{
		if (fri == 0)
			return false;

		// we do not drive in EStop case
//		if(isEStop) return false;

		return fri->isPowerOn();
	}

	void FRIDriver::setEStop(bool value)
	{
		isEStop = value;
	}

	bool FRIDriver::isSpringRelaxed()
	{
		bool relaxed = true;
		for (int i = 0; i < LBR_MNJ; i++)
		{
			relaxed &= fabs(getMeasuredJointPosition(i) - getCommandedJointPosition(i)) < 0.01;
		}
		return relaxed;
	}

	void FRIDriver::setFriToBool(int id, bool value)
	{
		if (fri != 0)
			fri->setToKRLBool(id, value);
	}

	void FRIDriver::setFriToReal(int id, double value)
	{
		if (fri != 0)
			fri->setToKRLReal(id, value);
	}

	void FRIDriver::setFriToInt(int id, int value)
	{
		if (fri != 0)
			fri->setToKRLInt(id, value);
	}

	bool FRIDriver::configureHook()
	{
		float j[7] =
		{ 0, 0, 0, 0, 0, 0, 0 };
		fri->doPositionControl(j, false);
		return true;
	}

	bool FRIDriver::startHook()
	{
		return true;
	}

	void FRIDriver::stopHook()
	{
	}

	void FRIDriver::cleanupHook()
	{
	}

	void FRIDriver::updateHook()
	{
		if (fri != 0)
		{
			// prohibit updates on FRI structures
			lockDevice();

			if (fri->doReceiveData() == 0)
			{

				double cmdPos[7];
				getValuesToCommand(cmdPos, NULL);

				{
					// snapshot current RPI values

					// keine Freigabe, gemessene Werte auf kommandierte Werte spiegeln
					if (!getPower() || (getState() != FRI_STATE_CMD))
					{
						float* measured = getJointPosition();
						for (int i = 0; i < LBR_MNJ; i++)
						{
							cmdPos[i] = measured[i];
							setJointPositionStatic(i, cmdPos[i]);
						}
						deviceState = RPI::DeviceState::SAFE_OPERATIONAL;
					} else
					{
						deviceState = RPI::DeviceState::OPERATIONAL;
					}

					// if we are in monitor mode, and did not request so ourselves,
					// mark all load data as invalid
					if ((getState() != FRI_STATE_CMD) && krc_mode_finished)
					{
						krc_loadDataValid = false;
					}
				}

				float curFriJoints[7];
				for (int i = 0; i < 7; i++)
				{
					curFriJoints[i] = cmdPos[i];
					dumpFriPos[i].put(cmdPos[i]);
				}

				// command joints, positions etc
				switch (fri->getCurrentControlScheme())
				{
				case FRI_CTRL_POSITION:
					fri->doPositionControl(&curFriJoints[0], false);
					break;
				case FRI_CTRL_JNT_IMP:
					fri->doJntImpedanceControl(&curFriJoints[0], &jntstiff[0], &jntdamp[0], &jntaddtrq[0], false);
					break;
				case FRI_CTRL_CART_IMP:
					fri->doCartesianImpedanceControl(NULL, &cartstiff[0], &cartdamp[0], NULL, &curFriJoints[0], false);
					break;
				case FRI_CTRL_OTHER:
					// should not happen, do nothing
					break;
				}

				fri->doSendData();

				RTT::os::TimeService::nsecs time = RTT::os::TimeService::Instance()->getNSecs(lastNsecs);
				double cycleTime = time / 1e9;
				// Update velocities and accelerations
				for (int i = 0; i < 7; i++)
				{
					msrjoint_v[i] = (fri->getMsrMsrJntPosition()[i] - old_msrjoint[i]) / cycleTime;
					old_msrjoint[i] = fri->getMsrMsrJntPosition()[i];

					msrjoint_a[i] = (msrjoint_v[i] - old_msrjoint_v[i]) / cycleTime;
					old_msrjoint_v[i] = msrjoint_v[i];

					dumpMsrPos[i].put(getMeasuredJointPosition(i));
					dumpMsrVel[i].put(getMeasuredJointVelocity(i));
				}
				lastNsecs = RTT::os::TimeService::Instance()->getNSecs();
				failed = 0;
			} else
			{
				failed++;
				if (failed > 1.0 / FRI_CYCLE_TIME)
				{
					fri->clearMsrBuffer();
					deviceState = RPI::DeviceState::OFFLINE;
					failed = 0;
				}
			}

			// perform communication with KRL
			KRCMonitorModeLoopHandler();

			// reenable updates
			unlockDevice();
		}
	}

	/**
	 * Periodic update handler which cares about all switching actions necessary while being in
	 * monitor mode
	 */
	void FRIDriver::KRCMonitorModeLoopHandler()
	{
		switch (krc_switch_state)
		{
		// idle case
		case INACTIVE:
			// check if something needs updating
			if (loadDataChanged() || controlSchemeChanged())
			{
				// check whether power is enabled, stop otherwise
				if (!this->getPower())
				{
					krc_mode_error = 2;
				} else if (!this->isSpringRelaxed())
					krc_mode_error = 3;
				else
				{
					int newstrategy = getKRCControlScheme();
					if (newstrategy != 0)
					{
						krc_mode_finished = false;
						monitorModeState = 1;
						// send handshake
						setFriToBool(id_handshake, true);

						// send first data packet
						setFriToInt(id_communication, getKRCControlScheme());
						setFriToInt(id_handshake, monitorModeState);
						krc_switch_state = REQUESTED;
					}
				}
			} else
			{
				// nothing to do for us
				krc_mode_error = 0;
				krc_mode_finished = true;
			}
			break;
		case REQUESTED:
			if (getFriFrmBool(id_handshake))
			{
				krc_switch_state = ACKNOWLEDGED;
			} else
			{
				// Timeout ...
			}
			break;
		case ACKNOWLEDGED:
			if (getFriFrmBool(id_handshake))
			{
				// check if KRL acknowledged
				if (getFriFrmInt(id_handshake) == monitorModeState)
				{
					double tcpa, tcpb, tcpc, coma, comb, comc;

					krc_desired_tcp.M.GetRPY(tcpc, tcpb, tcpa);
					krc_desired_com.M.GetRPY(comc, comb, coma);

					monitorModeState++;
					switch (monitorModeState)
					{
					case 2:
						setFriToReal(id_communication, krc_desired_tcp.p.x() * 1000);
						break;
					case 3:
						setFriToReal(id_communication, krc_desired_tcp.p.y() * 1000);
						break;
					case 4:
						setFriToReal(id_communication, krc_desired_tcp.p.z() * 1000);
						break;
					case 5:
						setFriToReal(id_communication, tcpa * 180 / KDL::PI);
						break;
					case 6:
						setFriToReal(id_communication, tcpb * 180 / KDL::PI);
						break;
					case 7:
						setFriToReal(id_communication, tcpc * 180 / KDL::PI);
						break;
					case 8:
						setFriToReal(id_communication, krc_desired_mass);
						break;
					case 9:
						setFriToReal(id_communication, krc_desired_com.p.x() * 1000);
						break;
					case 10:
						setFriToReal(id_communication, krc_desired_com.p.y() * 1000);
						break;
					case 11:
						setFriToReal(id_communication, krc_desired_com.p.z() * 1000);
						break;
					case 12:
						setFriToReal(id_communication, coma * 180 / KDL::PI);
						break;
					case 13:
						setFriToReal(id_communication, comb * 180 / KDL::PI);
						break;
					case 14:
						setFriToReal(id_communication, comc * 180 / KDL::PI);
						break;
					case 15:
						setFriToReal(id_communication, krc_desired_moi.x());
						break;
					case 16:
						setFriToReal(id_communication, krc_desired_moi.y());
						break;
					case 17:
						setFriToReal(id_communication, krc_desired_moi.z());
						break;
					case 18:
						// not a real case, but KRL has acknowledged last transmission
						setFriToBool(id_handshake, false);
						krc_switch_state = CONFIRMED;
					}
					setFriToInt(id_handshake, monitorModeState);
				} else
				{
					// in else case just wait (aka do nothing), either a new value will be acknowledged, or protocol will be
					// aborted
				}
			} else
			{
				// Protocol was aborted
				krc_switch_state = INACTIVE;
				krc_mode_error = 0;
				krc_mode_finished = true;
			}
			break;
		case CONFIRMED:
			// wait for KRL to reinitialize command mode
			if (!getFriFrmBool(id_handshake))
			{
				krc_switch_state = INACTIVE;
				krc_mode_finished = true;
				krc_mode_error = 0;
				krc_current_com = krc_desired_com;
				krc_current_mass = krc_desired_mass;
				krc_current_moi = krc_desired_moi;
				krc_current_tcp = krc_desired_tcp;
				krc_loadDataValid = true;
			}
			break;
		}
	}

	int FRIDriver::getKRCControlScheme()
	{
		int newstrategy = 0;
		switch (krc_desired_controlscheme)
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
			krc_desired_controlscheme = this->getControlScheme();
			break;
		}
		return newstrategy;
	}

	bool FRIDriver::loadDataChanged()
	{
		if (!krc_loadDataValid || krc_desired_com != krc_current_com || krc_desired_mass != krc_current_mass
				|| krc_desired_moi != krc_current_moi || krc_desired_tcp != krc_current_tcp)
			return true;
		else
			return false;
	}

	bool FRIDriver::controlSchemeChanged()
	{
		return this->getControlScheme() != krc_desired_controlscheme;
	}

	void FRIDriver::lockDevice()
	{
		deviceMutex.lock();
	}

	void FRIDriver::unlockDevice()
	{
		deviceMutex.unlock();
	}

	// RobotArm interface
	int FRIDriver::getJointCount() const
	{
		return 7;
	}
	int FRIDriver::getJointError(int joint)
	{
		return getPower() ? 0 : 1;
	}
	robotarm::JointPositionError FRIDriver::checkJointPosition(int joint, double position)
	{
		return checkJointPosition(position, joint) ? robotarm::JP_OK : robotarm::JP_INVALID;
	}
	double FRIDriver::getMeasuredJointPosition(int joint)
	{
		return getJointPosition()[joint];
	}
	double FRIDriver::getMeasuredJointVelocity(int joint)
	{
		return getMsrJointVel()[joint];
	}
	double FRIDriver::getMeasuredJointAcceleration(int joint)
	{
		return getMsrJointAcc()[joint];
	}

	void FRIDriver::setToolMOI(KDL::Vector moi, int axis)
	{
		if (axis != 6)
			return;

		setKRCToolMOI(moi);
		setKRCModeFinished(false);
	}

	void FRIDriver::setToolCOM(KDL::Vector com, int axis)
	{
		if (axis != 6)
			return;

		KDL::Frame frame;
		frame.p = com;

		setKRCToolCOM(frame);
		setKRCModeFinished(false);
	}

	void FRIDriver::setToolMass(double mass, int axis)
	{
		if (axis != 6)
			return;

		setKRCToolMass(mass);
		setKRCModeFinished(false);
	}

	bool FRIDriver::getToolFinished(int axis) const
	{
		if (axis != 6)
			return true;

		return getKRCModeFinished();
	}

	int FRIDriver::getToolError(int axis) const
	{
		if (axis != 6)
			return 0;

		return getKRCModeError();
	}

	// IODriver interface
	void FRIDriver::setDigitalOut(int port, bool value)
	{
		this->setFriToBool(port, value);
	}
	bool FRIDriver::getDigitalOut(int port)
	{
		return this->getFriToBool(port);
	}
	bool FRIDriver::getDigitalIn(int port)
	{
		return this->getFriFrmBool(port);
	}
	void FRIDriver::setAnalogOut(int port, double value)
	{
		this->setFriToReal(port, value);
	}
	double FRIDriver::getAnalogOut(int port)
	{
		return this->getFriToReal(port);
	}
	double FRIDriver::getAnalogIn(int port)
	{
		return this->getFriFrmReal(port);
	}

	unsigned int FRIDriver::getNumDigitalIn() const
	{
		return IPortsDig.size();
	}
	unsigned int FRIDriver::getNumDigitalOut() const
	{
		return OPortsDig.size();
	}
	unsigned int FRIDriver::getNumAnalogIn() const
	{
		return IPortsAn.size();
	}
	unsigned int FRIDriver::getNumAnalogOut() const
	{
		return OPortsAn.size();
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
	}

	bool FRIDriver::setKRCDesiredControlScheme(int scheme)
	{
		if (scheme != krc_desired_controlscheme)
		{
			krc_desired_controlscheme = scheme;
			return true;
		} else
		{
			return false;
		}
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
			krc_desired_mass = mass;
			return true;
		} else
		{
			return false;
		}
	}

	RPI::DeviceState FRIDriver::getDeviceState() const
	{
		if (fri == 0)
			return RPI::DeviceState::OFFLINE;
		if (isEStop)
			return RPI::DeviceState::SAFE_OPERATIONAL;
		return deviceState;
	}

}

