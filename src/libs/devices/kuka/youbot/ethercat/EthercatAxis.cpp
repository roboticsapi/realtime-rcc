/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "EthercatAxis.hpp"
#include <rtt/Logger.hpp>
#include "kdl/utilities/utility.h"
#include "EthercatProtocolDefinitions.hpp"
#include <boost/units/systems/angle/gradians.hpp>

using namespace std;
using namespace boost::units;

namespace kuka_youbot
{

	ybAxis::ybAxis(YoubotEthercatSlave* ethercatSlave, int axis, int gearRatio, float torqueConstant)
	{
		this->axis = axis;
		this->gearRatio = gearRatio;
		this->torqueConstant = torqueConstant;

		this->ethercatSlave = ethercatSlave;

		encoderTicksPerRound = 4000;
	}

	ybAxis::~ybAxis()
	{
	}

	// blocking read / write
	bool ybAxis::retrieveValueFromMotorContoller(YouBotSlaveMailboxMsg& message)
	{
		int sleepTime = 1; //ms
		while (!ethercatSlave->setMailboxMsgBuffer(message))
			SLEEP_MILLISEC(sleepTime);

		while (!ethercatSlave->getMailboxMsgBuffer(message))
			SLEEP_MILLISEC(sleepTime);

		// std::cout << "cmd " << (int)message.stctInput.commandNumber << ", ec " << (int)message.stctOutput.commandNumber << ", val " << (int)message.stctInput.value << ", status " << (int)message.stctInput.status << std::endl;
		return (message.stctOutput.commandNumber == message.stctInput.commandNumber
				&& message.stctInput.status == NO_ERROR);
	}

	bool ybAxis::setValueToMotorContoller(const YouBotSlaveMailboxMsg& mailboxMsg)
	{
		YouBotSlaveMailboxMsg message = mailboxMsg;
		const int sleepTime = 1; //ms
		while (!ethercatSlave->setMailboxMsgBuffer(message))
		{
			SLEEP_MILLISEC(sleepTime);
		}

		while (!ethercatSlave->getMailboxMsgBuffer(message))
		{
			SLEEP_MILLISEC(sleepTime);
		}

		return (message.stctOutput.commandNumber == message.stctInput.commandNumber
				&& message.stctOutput.value == message.stctInput.value && message.stctInput.status == NO_ERROR);
	}

	// create mailbox message
	YouBotSlaveMailboxMsg ybAxis::createMailboxMsg(const int module, const int command, const int type, const int motorBank, const int value) const
	{
		YouBotSlaveMailboxMsg ret;
		ret.stctOutput.moduleAddress = module;
		ret.stctOutput.commandNumber = command;
		ret.stctOutput.typeNumber = type; //move gripper relative
		ret.stctOutput.value = value;
		ret.stctOutput.motorNumber = motorBank;
		return ret;
	}

	// TMCL commands
	void ybAxis::TMCL_Axis_MVP(bool absolute, int position) {
		setValueToMotorContoller(createMailboxMsg(DRIVE, MVP, absolute?0:1, 0, position));
	}
	int ybAxis::TMCL_Axis_GAP(int parameter) {
		YouBotSlaveMailboxMsg gap = createMailboxMsg(DRIVE, GAP, parameter, 0, 0);
		retrieveValueFromMotorContoller(gap);
		return gap.stctInput.value;
	}
	int ybAxis::TMCL_Axis_GGP(int parameter, int bank) {
		YouBotSlaveMailboxMsg ggp = createMailboxMsg(DRIVE, GGP, parameter, bank, 0);
		retrieveValueFromMotorContoller(ggp);
		return ggp.stctInput.value;
	}
	bool ybAxis::TMCL_Axis_SAP(int parameter, int value)  {
		return setValueToMotorContoller(createMailboxMsg(DRIVE, SAP, parameter, 0, value));
	}
	bool ybAxis::TMCL_Axis_SGP(int parameter, int bank, int value) {
		return setValueToMotorContoller(createMailboxMsg(DRIVE, SGP, parameter, bank, value));
	}

	void ybAxis::TMCL_Gripper_MST(int bar) {
		setValueToMotorContoller(createMailboxMsg(GRIPPER, MST, 0, bar, 0));
	}
	void ybAxis::TMCL_Gripper_MVP(int bar, bool absolute, int position) {
		setValueToMotorContoller(createMailboxMsg(GRIPPER, MVP, absolute ? 0 : 1, bar, position));
	}
	bool ybAxis::TMCL_Gripper_SAP(int bar, int parameter, int value) {
		return setValueToMotorContoller(createMailboxMsg(GRIPPER, SAP, parameter, bar, value));
	}
	int ybAxis::TMCL_Gripper_GAP(int bar, int parameter) {
		YouBotSlaveMailboxMsg gap = createMailboxMsg(GRIPPER, GAP, parameter, bar, 0);
		retrieveValueFromMotorContoller(gap);
		return gap.stctInput.value;
	}
	void ybAxis::TMCL_ProcessData(int value, int controllerMode) {
		YouBotSlaveMsg messageBuffer;
		messageBuffer.stctOutput.controllerMode = controllerMode;
		messageBuffer.stctOutput.value = value;
		ethercatSlave->setMsgBuffer(messageBuffer);
	}

///gets the motor current of one joint which have been measured by a hal sensor
///@param axis the axis which should be sensed
	quantity<si::current> ybAxis::getSensedCurrent()
	{
		YouBotSlaveMsg messageBuffer;
		messageBuffer = ethercatSlave->getMsgBuffer();
		double current = messageBuffer.stctInput.actualCurrent;
		quantity<si::current> sensedcurrent = current / 1000.0 * si::ampere;

		return sensedcurrent;
	}

///sets the velocity in round per minute to one joint
///@param JointRoundsPerMinute the setpoint velocity
///@param axis the axis
	void ybAxis::setJointRoundsPerMinute(int JointRoundsPerMinute)
	{
		YouBotSlaveMsg messageBuffer;
		messageBuffer = ethercatSlave->getMsgBuffer();
		parseError(messageBuffer.stctInput.errorFlags);
		messageBuffer.stctOutput.controllerMode = VELOCITY_CONTROL;
		messageBuffer.stctOutput.value = JointRoundsPerMinute;

		ethercatSlave->setMsgBuffer(messageBuffer);
	}

	void ybAxis::setJointVelocity(float vel)
	{
		setJointRoundsPerMinute(vel * 60 / 2 / KDL::PI * gearRatio);
	}

///commands a current to one joint
///@param current the commanded current in A
	void ybAxis::setJointCurrent(float current)
	{
		// Bouml preserved body begin 000955F1
		YouBotSlaveMsg messageBuffer;
		messageBuffer = ethercatSlave->getMsgBuffer();
		parseError(messageBuffer.stctInput.errorFlags);

		messageBuffer.stctOutput.controllerMode = CURRENT_MODE;
		messageBuffer.stctOutput.value = (int32) (current * 1000.0);

		if(messageBuffer.stctOutput.value > 4000)
			messageBuffer.stctOutput.value = 4000;

		if(messageBuffer.stctOutput.value < -4000)
			messageBuffer.stctOutput.value = -4000;

		if(messageBuffer.stctOutput.value != messageBuffer.stctOutput.value)
			messageBuffer.stctOutput.value = 0;
//		std::cout << "current"<< messageBuffer.stctOutput.value<<std::endl;
		ethercatSlave->setMsgBuffer(messageBuffer);
	}

///command a torque to one joint
///@param torque the commanded torque in Nm
	void ybAxis::setJointTorque(float torque)
	{
		float current;
		current = torque/(gearRatio*torqueConstant);
		setJointCurrent(current);
	}

/// set the encoder values of the joint to zero. This postion will be the new reference.
///@param axis the axis
	void ybAxis::setEncoderToZero()
	{
		YouBotSlaveMsg messageBuffer;
		messageBuffer.stctOutput.controllerMode = SET_POSITION_TO_REFERENCE;
		messageBuffer.stctOutput.value = 0;

		ethercatSlave->setMsgBuffer(messageBuffer);
	}

	std::string ybAxis::getJointFirmware()
	{
		YouBotSlaveMailboxMsg FirmwareVersion;
		FirmwareVersion.stctOutput.commandNumber = FIRMWARE_VERSION;
		FirmwareVersion.stctOutput.moduleAddress = DRIVE;
		FirmwareVersion.stctOutput.typeNumber = 0;
		FirmwareVersion.stctOutput.value = 0;
		retrieveValueFromMotorContoller(FirmwareVersion);
		char version[9];
		version[0] = FirmwareVersion.stctInput.replyAddress;
		version[1] = FirmwareVersion.stctInput.moduleAddress;
		version[2] = FirmwareVersion.stctInput.status;
		version[3] = FirmwareVersion.stctInput.commandNumber;
		version[4] = FirmwareVersion.stctInput.value >> 24;
		version[5] = FirmwareVersion.stctInput.value >> 16;
		version[6] = FirmwareVersion.stctInput.value >> 8;
		version[7] = FirmwareVersion.stctInput.value & 0xff;
		version[8] = 0;
		string ver = version;
		return ver;

	}

	int ybAxis::getJointParameter(int typeNumber)
	{
		return TMCL_Axis_GAP(typeNumber);
	}

	bool ybAxis::setJointParameter(int typeNumber, int value)
	{
		return TMCL_Axis_SAP(typeNumber, value);
	}

///gets the position or angle of one joint which have been calculated from the actual encoder value
///@param axis the axis
	float ybAxis::getJointSensedAngle()
	{
		YouBotSlaveMsg messageBuffer;
		messageBuffer = ethercatSlave->getMsgBuffer();
		parseError(messageBuffer.stctInput.errorFlags);

		float angle = ((double) messageBuffer.stctInput.actualPosition / encoderTicksPerRound)
				* ((double) 1 / gearRatio) * (2.0 * M_PI); //* radian;

		return angle;
	}

///gets the motor torque of one joint which have been calculated from the current
///@param axis the axis
	quantity<si::torque> ybAxis::getJointSensedTorque()
	{

		quantity<si::current> sensedCurrent = getSensedCurrent();
		quantity<si::torque> torque = ((sensedCurrent.value() * torqueConstant) * gearRatio) * si::newton_meter;
		return torque;

	}

//Winkel im Bogenmaß
///commands angle to one joint
///@param angle the command position
///@param axis	the joint
	void ybAxis::setJointAngle(float angle)
	{
		YouBotSlaveMsg messageBuffer;
		messageBuffer = ethercatSlave->getMsgBuffer();
		parseError(messageBuffer.stctInput.errorFlags);
		messageBuffer.stctOutput.controllerMode = POSITION_CONTROL;
		quantity<si::plane_angle> angle2;
		angle2 = angle * si::radian;
		messageBuffer.stctOutput.value = (int32) round(
				(angle * ((double) encoderTicksPerRound / (2.0 * M_PI))) / ((double) 1 / gearRatio));

		ethercatSlave->setMsgBuffer(messageBuffer);
	}

///gets the velocity in round per minute of one joint
///@param axis the axis
	int ybAxis::getJointSensedRoundsPerMinute()
	{
		YouBotSlaveMsg messageBuffer;
		messageBuffer = ethercatSlave->getMsgBuffer();
		int rpm;
		rpm = messageBuffer.stctInput.actualVelocity;
		return rpm;
	}

	float ybAxis::getJointVelocity()
	{
		float vel = getJointSensedRoundsPerMinute();
		return vel * KDL::PI * 2 / 60 / gearRatio;

	}

///gets the error status of a joint
///@param axis the axis
	int ybAxis::getJointError()
	{
		YouBotSlaveMsg messageBuffer;
		messageBuffer = ethercatSlave->getMsgBuffer();
		int error;
		error = messageBuffer.stctInput.errorFlags;
		return error;
	}

	void ybAxis::parseError(uint32 error)
	{
		//RTT::log(RTT::Info) << "status " << error << RTT::endlog();

		if (error & OVER_CURRENT)
		{
			RTT::log(RTT::Info) << "over current " << axis << RTT::endlog();
		}

		if (error & UNDER_VOLTAGE)
		{
			RTT::log(RTT::Info) << "under voltage " << axis << RTT::endlog();
		}

		if (error & OVER_VOLTAGE)
		{
			RTT::log(RTT::Info) << "over voltage " << axis << RTT::endlog();
		}

		if (error & OVER_TEMPERATURE)
		{
			RTT::log(RTT::Info) << "over temperature " << axis << RTT::endlog();
		}

		if (error & MOTOR_HALTED)
		{
			// RTT:log(RTT::Info) <<  "is halted"  << RTT::endlog();
		}

		if (error & HALL_SENSOR_ERROR)
		{
			RTT::log(RTT::Info) << "hall sensor problem " << axis << RTT::endlog();
		}

		if (error & ENCODER_ERROR)
		{
			RTT::log(RTT::Info) << "encoder problem " << axis << RTT::endlog();
		}

		if (error & INITIALIZATION_ERROR)
		{
			RTT::log(RTT::Info) << "initialization problem " << axis << RTT::endlog();
		}

		if (error & PWM_MODE_ACTIVE)
		{
			// RTT::log(RTT::Info) << "has PWM mode active " << axis << RTT::endlog();
		}

		if (error & VELOCITY_MODE)
		{
			// RTT::log(RTT::Info) <<  "has velocity mode active"  << RTT::endlog();
		}

		if (error & POSITION_MODE)
		{
			// RTT::log(RTT::Info) << "has position mode active " << axis << RTT::endlog();
		}

		if (error & TORQUE_MODE)
		{
			// RTT::log(RTT::Info) << "has torque mode active " << axis << RTT::endlog();
		}

		if (error & EMERGENCY_STOP)
		{
			RTT::log(RTT::Info) << "emergency stop active " << axis << RTT::endlog();
		}

		if (error & FREERUNNING)
		{
			RTT::log(RTT::Info) << "has freerunning active " << axis << RTT::endlog();
		}

		if (error & POSITION_REACHED)
		{
			// RTT::log(RTT::Info) << "has position reached " << axis << RTT::endlog();
		}

		if (!(error & INITIALIZED))
		{
			RTT::log(RTT::Info) << "not initialized " << axis << RTT::endlog();
		}

		if (error & TIMEOUT)
		{
			RTT::log(RTT::Info) << "exceeded timeout " << axis << RTT::endlog();
		}

		if (error & I2T_EXCEEDED)
		{
			RTT::log(RTT::Info) << "exceeded I2t " << axis << RTT::endlog();
		}
	}

	bool ybAxis::IsCalibrated()
	{
		const char index = 16; // Parameter 0 to 15 of bank 2 are password protected
		return TMCL_Axis_GGP(index, USER_VARIABLE_BANK) == 1;
	}

	void ybAxis::sendIsCalibratedMessage()
	{
		const char index = 16; // Parameter 0 to 15 of bank 2 are password protected
		setEncoderToZero();
		TMCL_Axis_SGP(index, USER_VARIABLE_BANK, 1);
	}

	bool ybAxis::calibrateGripperBlocking(int maxEncoderValueGripper)
	{
		if (axis != 5)
			return false;

		unsigned int maxenc = maxEncoderValueGripper;
		TMCL_Gripper_MVP(0, false, -maxenc);
		TMCL_Gripper_MVP(1, false, -maxenc);

		//open gripper
		bool targetReachedBar1 = false, targetReachedBar2 = false;

		for (int i = 0; i < 40; i++)
		{
			if(!targetReachedBar1 && TMCL_Gripper_GAP(0, 8) == 1) targetReachedBar1 = true;
			if(!targetReachedBar2 && TMCL_Gripper_GAP(1, 8) == 1) targetReachedBar2 = true;
			if (targetReachedBar1 && targetReachedBar2) break;
			usleep(100000);
		}

		TMCL_Gripper_MVP(0, false, maxenc);
		TMCL_Gripper_MVP(1, false, maxenc);

		targetReachedBar1 = false;
		targetReachedBar2 = false;

		for (int i = 0; i < 40; i++)
		{
			if(!targetReachedBar1 && TMCL_Gripper_GAP(0, 8) == 1) targetReachedBar1 = true;
			if(!targetReachedBar2 && TMCL_Gripper_GAP(1, 8) == 1) targetReachedBar2 = true;
			if (targetReachedBar1 && targetReachedBar2) break;
			usleep(100000);
		}

		//stop Gripper motor
		TMCL_Gripper_MST(0);
		TMCL_Gripper_MST(1);

		// set pose to zero as reference
		TMCL_Gripper_SAP(0, 1, 0);
		TMCL_Gripper_SAP(1, 1, 0);
		return true;
	}


	bool ybAxis::sendGripperPosition(const float gripperPostion, const int motorNr) const
	{
		return ethercatSlave->setMailboxMsgBuffer(createMailboxMsg(GRIPPER, MVP, 0, motorNr, -gripperPostion));
	}

	bool ybAxis::receiveGripperMessage() const
	{
		YouBotSlaveMailboxMsg outMessage;
		return ethercatSlave->getMailboxMsgBuffer(outMessage);
	}

	bool ybAxis::sendGripperFinished(const int motorNr) const
	{
		return ethercatSlave->setMailboxMsgBuffer(createMailboxMsg(GRIPPER, GAP, 8, motorNr, 0));
	}

	bool ybAxis::receiveGripperFinished(bool& finished) const
	{
		YouBotSlaveMailboxMsg outMessage;
		bool successful;
		successful = ethercatSlave->getMailboxMsgBuffer(outMessage);
		if(successful)
			finished = outMessage.stctInput.value == 1;
		else
			finished = false;

		return successful;
	}

	void ybAxis::initialize()
	{
		//RTT::log(RTT::Info) << "Axis " << axis << " Firmware " << getJointFirmware() << RTT::endlog();

		// Clear Motor Controller Timeout Flag
		if(!TMCL_Axis_SAP(MOTOR_CONTROLLER_TIMEOUT_FLAG, 1))  {
			RTT::log(RTT::Error) << "Could not clear Motor Controller Timeout Flag, retry!" << RTT::endlog();
			return;
		}

		// Initialize BLDC
		if(!TMCL_Axis_GAP(INITIALIZE_BLDC)) {
			RTT::log(RTT::Error) << "Initializing axis " << axis << "." << RTT::endlog();
			TMCL_ProcessData(-100, VELOCITY_CONTROL);
//			TMCL_ProcessData(0, INITIALIZE);
		}

	}
}
