/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include <rtt/Activity.hpp>
#include <rcc/rapidxml/rapidxml.hpp>
#include "KR_Controller_RSI.hpp"
#include <cstdlib>
#include <iomanip>

namespace kuka_kr
{
	const unsigned int buffersize = 2000;

	KR_Controller_RSI::KR_Controller_RSI(const std::string& name, const std::string& ethdevice,
			const KR_to_SI_factors& conv_factors) :
			CyclicPositionRobotArm(kr_rsi_axiscount, 50), TaskContext(name, PreOperational), Device(name,
					RPI::parameter_t())
	{
		sendsize = 0;

		this->ethdevicename = ethdevice;
		this->conv_factors = conv_factors;

		// Initialize receive buffer with NULL bytes
		recvbuffer = new char[buffersize + 1];
		memset(recvbuffer, 0, buffersize + 1);
		sendbuffer = new char[buffersize];
		memset(sendbuffer, 0, buffersize);

		for (int i = 0; i < kr_rsi_axiscount; i++)
		{
			if (i < kr_rsi_axiscount_a)
				xml_name[i] = std::string("A").append(std::to_string(i + 1));
			else
				xml_name[i] = std::string("E").append(std::to_string((i - kr_rsi_axiscount_a) + 1));
		}

		memset(&recv_data, 0, sizeof(recv_data));

		stop_rsi = false;
		connection_alive = false;

		isEstop = false;

		toolswitching = false;
		toolcomold = toolcom = KDL::Vector(0, 0, 0);

		// Set tool to 0 - must be overridden by KR_Arm_RSI
		toolmassold = toolmass = 1;

		// Set max values for cyclic position handler, will be overridden by Arm or ExtAxis
		max_vel.resize(kr_rsi_axiscount);
		max_acc.resize(kr_rsi_axiscount);
		minj.resize(kr_rsi_axiscount);
		maxj.resize(kr_rsi_axiscount);
		for (int i = 0; i < kr_rsi_axiscount; ++i)
		{
			maxj[i] = max_vel[i] = max_acc[i] = std::numeric_limits<double>::max();
			minj[i] = -std::numeric_limits<double>::max();
		}

		dumpMsrPos = new RPI::RoundRobinLog<double>[kr_rsi_axiscount];

		// initialize values
		for (int i = 0; i < kr_rsi_axiscount; i++)
		{
			std::stringstream joint;
			joint << "Joint " << i;
			dumpMsrPos[i].setName(joint.str() + "|Position|Measured");
		}

		for (const auto& dumper : CyclicPositionRobotArm::getCrashDumpers())
			addCrashDumper(dumper);

		this->setActivity(new RTT::Activity(RTT::os::HighestPriority, 0.001));

	}

	std::list<RPI::CrashDumper*> KR_Controller_RSI::getCrashDumpers() const
	{
		auto superdumpers = CyclicPositionRobotArm::getCrashDumpers();
		for (int i = 0; i < kr_rsi_axiscount; ++i)
			superdumpers.push_back(&dumpMsrPos[i]);

		return superdumpers;
	}

	KR_Controller_RSI::~KR_Controller_RSI()
	{
		if (udp.getDevice())
		{
			udp->close_socket();
		}

		delete[] recvbuffer;
		delete[] sendbuffer;

		delete[] dumpMsrPos;
	}

	bool KR_Controller_RSI::configureHook()
	{
		if (!udp.fetchInstance(ethdevicename))
		{
			RTT::log(RTT::Critical) << "RSI Device " << ethdevicename << " not found" << RTT::endlog();
			return false;
		}

		if (!udp->initialize_socket())
		{
			RTT::log(RTT::Critical) << "Could not initialize UDP socket " << ethdevicename << RTT::endlog();
			return false;
		}

		return true;
	}

	void KR_Controller_RSI::updateHook()
	{
		struct sockaddr_in udp_client;
		socklen_t udp_client_size = sizeof(udp_client);

		ssize_t received = udp->recvfrom(recvbuffer, buffersize, MSG_DONTWAIT, (struct sockaddr*) &udp_client,
				&udp_client_size);

		if (received >= 0)
		{
			// add terminiating 0 byte
			recvbuffer[received] = 0;

			// std::cout << recvbuffer << std::endl;

			double pos_old[kr_rsi_axiscount];
			memcpy(pos_old, recv_data.pos_cur, sizeof(pos_old));

			// Read data from XML
			parseKRData();

			for (int i = 0; i < kr_rsi_axiscount; ++i)
				dumpMsrPos[i].put(recv_data.pos_cur[i]);

			if (recv_data.status == 0)
			{
				// Initialize phase
				calculate_offset();

				// save tool data - switching not completed until status 1 reached
				toolmassold = toolmass;
				toolcomold = toolcom;

				// Toggle
				stop_rsi = !stop_rsi;

			} else if (!recv_data.pactive)
			{
				// mirror received position
				reset_position();
			} else
			{
				if (toolswitching && (toolmass != toolmassold || toolcom != toolcomold))
				{
					// Toggle stop bit to fall back to status 0
					stop_rsi = !stop_rsi;
				} else
				{
					// either no toolswitch was required, or status 0 just finished
					toolswitching = false;
					stop_rsi = false;
				}
			}

			// calculate velocity and acceleration
			RTT::os::TimeService::nsecs last_nsecs, new_nsecs, delta_nsecs;
			last_nsecs = recv_data.timestamp;
			recv_data.timestamp = new_nsecs = RTT::os::TimeService::Instance()->getNSecs();
			delta_nsecs = new_nsecs - last_nsecs;

			for (int i = 0; i < kr_rsi_axiscount; ++i)
			{
				double new_vel = (recv_data.pos_cur[i] - pos_old[i]) / (delta_nsecs / 1.0e9);
				double new_acc = (new_vel - recv_data.vel_cur[i]) / (delta_nsecs / 1.0e9);

				recv_data.vel_cur[i] = new_vel;
				recv_data.acc_cur[i] = new_acc;
			}

			connection_alive = true;

			// Create XML anwser
			createKRData();

			if (sendsize > 0)
			{
				udp->sendto(sendbuffer, sendsize, 0, (struct sockaddr*) &udp_client, udp_client_size);
			}
		} else
		{
			// Nothing received for 50 ms
			if (connection_alive && (RTT::os::TimeService::Instance()->getNSecs(recv_data.timestamp) > (50 * 1e6)))
			{
				connection_alive = false;
				reset_position();
			}
		}

	}

	void KR_Controller_RSI::parseKRData()
	{
		rapidxml::xml_document<> doc;
		doc.parse<0>(recvbuffer);

		rapidxml::xml_node<> *rob = doc.first_node("Rob");
		if (!rob)
			return;

		// Current position
		rapidxml::xml_node<> *p_cur = rob->first_node("AIPos");
		if (!p_cur)
			return; //calculate_offset();
		for (int i = 0; i < kr_rsi_axiscount_a; i++)
		{
			rapidxml::xml_attribute<> *aval = p_cur->first_attribute(xml_name[i].c_str());
			if (aval)
				recv_data.pos_cur[i] = atof(aval->value()) * conv_factors.conv_factor[i];
		}

		p_cur = rob->first_node("EIPos");
		if (!p_cur)
			return;
		for (int i = 0; i < kr_rsi_axiscount_a; i++)
		{
			rapidxml::xml_attribute<> *aval = p_cur->first_attribute(xml_name[i + 6].c_str());
			if (aval)
			{
				recv_data.pos_cur[i + kr_rsi_axiscount_a] = atof(aval->value())
						* conv_factors.conv_factor[i + kr_rsi_axiscount_a];
//				std::cout << "Setting EIPos " << std::endl;
			}
		}

		rapidxml::xml_node<>* ipoc = rob->first_node("IPOC");
		if (!ipoc)
			return;

		recv_data.ipoc = atoi(ipoc->value());

		rapidxml::xml_node<>* status = rob->first_node("Status");
		if (!status)
			return;

		recv_data.status = atoi(status->value());

		rapidxml::xml_node<>* pactive = rob->first_node("ProState_R");
		if (!pactive)
			return;

		recv_data.pactive = (atoi(pactive->value()) == 3) ? true : false;

		rapidxml::xml_node<>* digin = rob->first_node("DigIn");
		if (digin)
		{
			for (unsigned int i = 0; i < kr_rsi_digin; ++i)
			{
				std::string name = "D";
				name.append(std::to_string(i+1));
				rapidxml::xml_attribute<>* digio = digin->first_attribute(name.c_str());
				if (digio) {
					recv_data.digin[i] = atoi(digio->value()) == 1 ? true : false;
				}
			}
		}

	}

	void KR_Controller_RSI::createKRData()
	{
		std::stringstream res;
		res << std::setprecision(8);
		res << "<Sen Type=\"ImFree\">";

		if (recv_data.status == 0)
		{
			res << "<EStr>RealtimeRCC in calibration mode</EStr>";
		} else
		{
			res << "<EStr>RealtimeRCC running</EStr>";
		}
		res << "<AKorr ";
		double cmd[kr_rsi_axiscount];
		getValuesToCommand(RTT::os::TimeService::Instance()->getNSecs() + 0.006 * 1e9, cmd, 0);
		for (int i = 0; i < kr_rsi_axiscount_a; i++)
		{
			res << xml_name[i] << "=\"" << ((cmd[i] - recv_data.pos_offset[i]) / conv_factors.conv_factor[i]) << "\" ";
			//std::cout << "CMD " << i << " " << cmd[i] << " off " << recv_data.pos_offset[i] << std::endl;
		}

		res << "/><EKorr ";
		for (int i = 0; i < kr_rsi_axiscount_a; i++)
		{
			res << xml_name[i + 6] << "=\""
					<< ((cmd[i + kr_rsi_axiscount_a] - recv_data.pos_offset[i + kr_rsi_axiscount_a])
							/ conv_factors.conv_factor[i + kr_rsi_axiscount_a]) << "\" ";

		}
		res << "/>";

		// Stop bit
		res << "<Stop>" << (stop_rsi ? "1" : "0") << "</Stop>";

		// Tool
		// TODO: A, B, C, Necessary?

		res << "<TOOL X=\"" << toolcom.x() * 1000.0 << "\" Y=\"" << toolcom.y() * 1000.0 << "\" Z=\""
				<< toolcom.z() * 1000.0 << "\" ";
		res << "A=\"0\" B=\"0\" C=\"0\" ";
		res << "M=\"" << toolmass << "\"/>";

		// Digital I/O
		res << "<DigOut ";
		for (unsigned int i = 0; i < kr_rsi_digout; ++i)
		{
			res << "D" << (i + 1) << "=\"" << (recv_data.digout[i] ? "1" : "0") << "\" ";
		}
		res << "/>";

		// IPOC
		res << "<IPOC>" << recv_data.ipoc << "</IPOC>";
		res << "</Sen>";

		sendsize = res.str().size();
		if (sendsize <= buffersize)
			strcpy(sendbuffer, res.str().c_str());
		else
			sendsize = 0;

//		std::cout << res.str() << std::endl;
	}

	void KR_Controller_RSI::calculate_offset()
	{
		// set commanded joint positions to current configuration and save offset

		for (int i = 0; i < kr_rsi_axiscount; ++i)
		{
			setJointPositionStatic(i, recv_data.pos_cur[i]);
			recv_data.pos_offset[i] = recv_data.pos_cur[i];
//			std::cout << "Off" << i << ": " <<recv_data.pos_offset[i]  << " " ;
		}
//		std::cout << std::endl;
	}

	void KR_Controller_RSI::reset_position()
	{
		for (int i = 0; i < kr_rsi_axiscount; ++i)
		{
			setJointPositionStatic(i, recv_data.pos_cur[i]);
		}
	}

	int KR_Controller_RSI::getJointError(int joint)
	{
		if (isEstop)
			return 1;
		if (!((recv_data.status == 1) && recv_data.pactive))
			return 2;
		return 0;
	}

	double KR_Controller_RSI::getMeasuredJointPosition(int joint)
	{
		if (joint >= 0 && joint < kr_rsi_axiscount)
			return recv_data.pos_cur[joint];
		return 0;
	}

	double KR_Controller_RSI::getMeasuredJointVelocity(int joint)
	{
		if (joint >= 0 && joint < kr_rsi_axiscount)
			return recv_data.vel_cur[joint];
		return 0;
	}

	double KR_Controller_RSI::getMeasuredJointAcceleration(int joint)
	{
		if (joint >= 0 && joint < kr_rsi_axiscount)
			return recv_data.acc_cur[joint];
		return 0;
	}

	void KR_Controller_RSI::setToolCOM(KDL::Vector com, int axis)
	{
		toolcom = com;
		toolswitching = true;
	}

	void KR_Controller_RSI::setToolMOI(KDL::Vector moi, int axis)
	{
		// not yet supported
		return;
	}

	void KR_Controller_RSI::setToolMass(double mass, int axis)
	{
		toolmass = mass;
		toolswitching = true;
	}

	bool KR_Controller_RSI::getToolFinished(int axis) const
	{
		return !toolswitching;
	}

	int KR_Controller_RSI::getToolError(int axis) const
	{
		return 0;
	}

	double KR_Controller_RSI::getMaximumAcceleration(int joint) const
	{
		if (joint < 0 || joint >= max_acc.size())
			return 0;
		return max_acc[joint];
	}

	double KR_Controller_RSI::getMaximumVelocity(int joint) const
	{
		if (joint < 0 || joint >= max_vel.size())
			return 0;
		return max_vel[joint];
	}

	RPI::DeviceState KR_Controller_RSI::getDeviceState() const
	{
		if (!connection_alive)
			return RPI::DeviceState::OFFLINE;

		if ((recv_data.status == 1) && recv_data.pactive && !isEstop)
			return RPI::DeviceState::OPERATIONAL;
		else
			return RPI::DeviceState::SAFE_OPERATIONAL;
	}

	void KR_Controller_RSI::updateParameters()
	{

	}

	void KR_Controller_RSI::setEStop(bool estop)
	{
		this->isEstop = estop;

		// TODO: EStop beruecksichtigen
	}

	std::set<std::string> KR_Controller_RSI::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	int KR_Controller_RSI::getJointCount() const
	{
		return kr_rsi_axiscount;
	}

	robotarm::JointPositionError KR_Controller_RSI::checkJointPosition(int joint, double position)
	{
		if (position != position)
			return robotarm::JP_INVALID;

		if (joint < 0 || joint >= kr_rsi_axiscount)
			return robotarm::JP_INVALID;

		if (joint >= minj.size() || joint >= maxj.size())
			return robotarm::JP_OK;

		if (position < minj[joint] || position > maxj[joint])
			return robotarm::JP_OUTOFRANGE;

		return robotarm::JP_OK;
	}

	void KR_Controller_RSI::setJointConfig(int axis, double _max_vel, double _max_acc, double minj, double maxj)
	{
		if (axis < 0 || axis >= kr_rsi_axiscount)
			return;

		max_vel[axis] = _max_vel;
		max_acc[axis] = _max_acc;
		this->minj[axis] = minj;
		this->maxj[axis] = maxj;

	}

	void KR_Controller_RSI::setInitialToolConfig(double mass, KDL::Vector com)
	{
		// Set initial tool configuration
		toolmass = toolmassold = mass;
		toolcom = toolcomold = com;
		// Switching is not active, tool data will always be set on first start of RSI program
		toolswitching = false;
	}

	RSI_Recv_Data* KR_Controller_RSI::getRecvData()
	{
		return &recv_data;
	}

}
/* namespace kuka_kr */

