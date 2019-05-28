/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef YBAXIS_HPP_
#define YBAXIS_HPP_

#include <string>
#include <boost/units/unit.hpp>
#include <boost/units/systems/si.hpp>

#include "EthercatSlave.hpp"
#include "EthercatSlaveMailboxMsg.hpp"
#include "EthercatSlaveMsg.hpp"

namespace kuka_youbot
{

//#define SLEEP_MILLISEC(millisec) boost::this_thread::sleep(boost::posix_time::milliseconds((millisec)));
#define SLEEP_MILLISEC(millisec) usleep(millisec * 1000)

	class ybAxis
	{
	public:
		ybAxis(YoubotEthercatSlave* ethercatSlave, int axis, int gearRatio, float torqueConstant);
		virtual ~ybAxis();
		bool retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message);
		void setJointRoundsPerMinute(int JointRoundsPerMinute);
		void setJointVelocity(float vel);
		void setJointCurrent(float current);
		void setJointTorque(float torque);
		void setEncoderToZero();
		bool setValueToMotorContoller(const YouBotSlaveMailboxMsg& mailboxMsg);
		float getJointSensedAngle();
		float getJointVelocity();
		void setJointAngle(float angle);
		void parseError(uint32 error);
		std::string getJointFirmware();
		bool setJointParameter(int typeNumber, int value);
		int getJointParameter(int typeNumber);
		int getJointError();
		boost::units::quantity<boost::units::si::current> getSensedCurrent();
		boost::units::quantity<boost::units::si::torque> getJointSensedTorque();

		void TMCL_Axis_MVP(bool absolute, int position);
		bool TMCL_Axis_SAP(int parameter, int value);
		int TMCL_Axis_GAP(int parameter);
		int TMCL_Axis_GGP(int parameter, int bank);
		bool TMCL_Axis_SGP(int parameter, int bank, int value);
		void TMCL_Gripper_MST(int bar);
		void TMCL_Gripper_MVP(int bar, bool absolute, int position);
		bool TMCL_Gripper_SAP(int bar, int parameter, int value);
		int TMCL_Gripper_GAP(int bar, int parameter);
		void TMCL_ProcessData(int value, int controllerMode);

		int getJointSensedRoundsPerMinute();

		bool IsCalibrated();
		void sendIsCalibratedMessage();

		void initialize();

		// Gripper functionality
		bool calibrateGripperBlocking(int maxEncoderValueGripper);

		bool sendGripperPosition(const float gripperPosition, const int motorNr) const;
		bool receiveGripperMessage() const;

		bool sendGripperFinished(const int motorNr) const;
		bool receiveGripperFinished(bool& finished) const;

	private:
		int axis; // BEGINNS WITH 1 !
		int gearRatio;
		float torqueConstant;
		int encoderTicksPerRound;
		YoubotEthercatSlave* ethercatSlave;

		YouBotSlaveMailboxMsg createMailboxMsg(const int module, const int command, const int type, const int motorBank,
				const int value) const;

	};
}
#endif /* YBAXIS_HPP	_ */
