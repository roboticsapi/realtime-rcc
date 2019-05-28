/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "youBotArmEthercat.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/units/cmath.hpp>

#include <rtt/Activity.hpp>
#include <kdl/utilities/utility.h>

#include <rcc/DeviceFactory.hpp>
#include "EthercatProtocolDefinitions.hpp"
#include <kdl/chainfksolvervel_recursive.hpp>

namespace kuka_youbot
{
	using namespace std;
	using namespace RPI;
	using namespace boost::units;

	youBotArmEthercat::youBotArmEthercat(string name, parameter_t parameters) :
			TaskContext("youbot_arm_" + name, PreOperational), Device(name, parameters), CyclicPositionRobotArm(5, 50),
			jointMutex(), master(getParameter("ethercatdevice")),
			isEStop(false), isInit(false),
			maxGripperDistance(0.023), gripperOffset(0), maxEncoderValueGripper(67000)
	{
		armNr = getParameterT<double>("arm", 0);

		dumpMsrPos = new RPI::RoundRobinLog<double>[YOUBOT_NUMAXIS];
		dumpMsrVel= new RPI::RoundRobinLog<double>[YOUBOT_NUMAXIS];
		dumpECVel = new RPI::RoundRobinLog<double>[YOUBOT_NUMAXIS];
		dumpECTorque = new RPI::RoundRobinLog<double>[YOUBOT_NUMAXIS];

		// initialize values
		for (int i = 0; i < YOUBOT_NUMAXIS; i++)
		{
			std::stringstream joint; joint<<"Joint " << i;
			dumpMsrPos[i].setName(joint.str() + "|Position|Measured");
			dumpMsrVel[i].setName(joint.str() + "|Velocity|Measured");
			dumpECVel[i].setName(joint.str() + "|Velocity|Ethercat");
			dumpECTorque[i].setName(joint.str() + "|Torque|Ethercat");
			addCrashDumper(&dumpMsrPos[i]);
			addCrashDumper(&dumpMsrVel[i]);
			addCrashDumper(&dumpECVel[i]);
			addCrashDumper(&dumpECTorque[i]);

			msrPos[i] = 0;
			msrVel[i] = 0;
			msrAcc[i] = 0;

			msrError[i] = 0;
		}

		//Der youBot gibt beim Winkelmessen den Wert ab der Kalibrierungsposition. Um die Werte auf
		//die Homeposition umzurechnen muss man JointAngleToZero addieren
		JointAngleToZero[0] = 2.949606435870417;
		JointAngleToZero[1] = 1.1344640137963142;
		JointAngleToZero[2] = -2.5481807079117621;
		JointAngleToZero[3] = 1.7889624832941878;
		JointAngleToZero[4] = 2.9146998508305306;

		minAngle[0] = -169.0 /180*KDL::PI;
		maxAngle[0] = 169.0 /180*KDL::PI;
		minAngle[1] = -65.0 /180*KDL::PI;
		maxAngle[1] = 90.0 /180*KDL::PI;
		minAngle[2] = -151.0 /180*KDL::PI;
		maxAngle[2] = 146.0 /180*KDL::PI;
		minAngle[3] = -102.5 /180*KDL::PI;
		maxAngle[3] = 102.5 /180*KDL::PI;
		minAngle[4] = -167.5 /180*KDL::PI;
		maxAngle[4] = 167.5 /180*KDL::PI;

		jointDirection[0] = -1;
		jointDirection[1] = -1;
		jointDirection[2] = 1;
		jointDirection[3] = -1;
		jointDirection[4] = -1;

		for (int i = 0; i < YOUBOT_NUMAXIS; i++)  {
			slaves[i] = new YoubotEthercatSlave(this);
			armAxis[i] = 0;
		}

		isInit = false;
		startInit = false;

		this->setActivity(new RTT::Activity(RTT::os::LowestPriority, 0, 0, name));

		// setup controller
		positionController = new youBotArmPositionController(&datasource,this);
		jointImpedanceController = new youBotArmJointImpedanceController(&datasource,this,jointDirection);

		// store for index access
		armController[0] = positionController;
		armController[1] = jointImpedanceController;

		// default controller = position
		selectedController = 0;

		for(const auto& dumper: CyclicPositionRobotArm::getCrashDumpers())
			addCrashDumper(dumper);

		master->addDevice(this);

	}

	void youBotArmEthercat::startupDevice()
	{
		int gearRatio[YOUBOT_NUMAXIS] = { 156, 156, 100, 71, 71 };
		float torqueConstant[YOUBOT_NUMAXIS] = { 0.0335, 0.0335, 0.0335, 0.051, 0.049 }; // Nm/A
		std::string slaveName = "TMCM-1610";

		int startId = armNr * 5;
		int id = 0;
		for(const auto& slave: master->getSlaveNames())
		{
			if(slaveName == slave.slaveName) {
				if(id >= startId + 5) break;
				if(id >= startId) {
					delete armAxis[id - startId];
					armAxis[id - startId] = new ybAxis(slaves[id - startId], (id - startId) + 1, gearRatio[id], torqueConstant[id]);
					master->addSlave(slave.slaveID, slaves[id - startId]);
				}
				id++;
			}
		}
		startInit = isInit = false;
		if(id - startId != 5) {
			for(int i=startId; i < id; i++)
			{
				master->removeSlave(slaves[i - startId]);
			}
		}
	}

	void youBotArmEthercat::updateDevice()
	{
		if (!isInit) {
			return;
		}

		RTT::os::MutexLock lock(deviceMutex);
		double pos[5], vel[5];
		double cycleTime;
		getValuesToCommand(pos, vel, cycleTime);
		datasource.setCycleTime(cycleTime);
		for (int i = 0; i < YOUBOT_NUMAXIS; i++)
		{
			//sensed Angle und target Angle Kalibrier System
			msrPos[i] = jointDirection[i] * (armAxis[i]->getJointSensedAngle() - jointDirection[i] * JointAngleToZero[i]);
			msrVel[i] = jointDirection[i] * armAxis[i]->getJointVelocity();
			msrError[i] = armAxis[i]->getJointError();

			dumpMsrPos[i].put(msrPos[i]);
			dumpMsrVel[i].put(msrVel[i]);

			datasource.setCommandedJointPosition(i, pos[i]);
			datasource.setMeasuredJointPosition(i, msrPos[i]);
			datasource.setCommandedJointVelocity(i, vel[i]);
			datasource.setMeasuredJointVelocity(i, msrVel[i]);
		}

		// let controller do the job
		armController[selectedController]->updateHook();
	}

	void youBotArmEthercat::shutdownDevice()
	{
		isInit = startInit = false;
		for(int i=0; i<5; i++) {
			if(armAxis[i] != 0) armAxis[i]->setJointRoundsPerMinute(0);
		}
		for(int i=0; i<5; i++) {
			master->removeSlave(slaves[i]);
		}

	}

	void youBotArmEthercat::inSafeOp(int slaveno)
	{
		master->doSlaveOp(slaveno);

		bool allop = true;
		for (int i=0; i<5; i++)
		{
			allop &= slaves[i]->isinOp();
		}

		if(allop)
		{
			startInit = true;
			trigger();
		}
	}


	youBotArmEthercat::~youBotArmEthercat()
	{
		for (int i = 0; i < YOUBOT_NUMAXIS; i++)
		{
			delete armAxis[i]; armAxis[i] = 0;
			master->removeSlave(slaves[i]);
			delete slaves[i]; slaves[i] = 0;
		}

		delete[] dumpMsrPos; dumpMsrPos = 0;
		delete[] dumpMsrVel; dumpMsrVel = 0;
		delete[] dumpECVel; dumpECVel = 0;
		delete[] dumpECTorque; dumpECTorque = 0;

		delete positionController;	positionController = 0;
		delete jointImpedanceController; jointImpedanceController = 0;

		master->removeDevice(this);
	}

	bool youBotArmEthercat::configureHook()
	{
		return true;
	}

	bool youBotArmEthercat::startHook()
	{
		return true;
	}

	void youBotArmEthercat::stopHook()
	{
		return;
	}

	void youBotArmEthercat::cleanupHook()
	{
		return;
	}

	void youBotArmEthercat::updateHook()
	{
		if(startInit)
		{
			isInit = initEtherCat();
			startInit = false;
		}

	}

	void youBotArmEthercat::updateParameters()
	{
		return;
	}

	set<string> youBotArmEthercat::getMutableParameters() const
	{
		return set<string>();
	}

	youBotArmEthercat* youBotArmEthercat::createDevice(string name, parameter_t parameters)
	{
		youBotArmEthercat* ret = new youBotArmEthercat(name, parameters);
		ret->configure();
		ret->start();
		return ret;
	}

	double youBotArmEthercat::getMaximumAcceleration(int joint) const {
		switch(joint) {
		case 0: case 1: return 1.25;
		case 2: return 2;
		case 3: case 4: return 2.5;
		}
		return 1;
	}

	double youBotArmEthercat::getMaximumVelocity(int joint) const {
		return 1.57;
	}


	// set controller to selected index
	void youBotArmEthercat::setControllerIndex(int controllerIndex)
	{
		// security check
		if (!checkControllerIndex(controllerIndex)) return;

		// set selected controller
		this->selectedController = controllerIndex;
	}


	// check if contrller index is valid
	bool youBotArmEthercat::checkControllerIndex(int controllerIndex)
	{
		// check
		if (controllerIndex < 0 || controllerIndex > 1) return false;
		else return true;
	}


	// set stiffness for joint impedance controller
	void youBotArmEthercat::setControllerJointImpedanceParameters(int joint, float stiffness,float damping, float maxTorque)
	{
		// security check
		if(!checkControllerJointImpedanceParameters(joint,stiffness,damping,maxTorque))return;

		// set stiffness
		jointImpedanceController->setStiffness(joint,stiffness);

		// set damping
		jointImpedanceController->setDampingConstant(joint,damping);

		// set maximum torque
		jointImpedanceController->setMaximumTorque(joint, maxTorque);
	}


	// check impedance parameters
	bool youBotArmEthercat::checkControllerJointImpedanceParameters(int joint, float stiffness,float damping, float maxTorque)
	{
		// no negative stiffness
		if (stiffness < 0)
			return false;

		// damping must be in 0..1 range
		if (damping < 0 || damping > 1)
			return false;

		// joint index 0...4
		if(joint < 0 || joint > 4) return false;

		// parameters are valid
		else
			return true;
	}


	// set additional torque for joint impedance controller
	void youBotArmEthercat::setControllerJointImpedanceAddionalTorque(int joint, float torque)
	{
		// security check
		if (!checkControllerJointImpedanceAdditionalTorque(joint,torque)) return;

		// set torque to controller
		jointImpedanceController->setAdditionalTorque(joint,torque);
	}

	// check additional torque for joint impedance controller
	bool youBotArmEthercat::checkControllerJointImpedanceAdditionalTorque(int joint, float torque)
	{
		// joint index 0...4
		if(joint < 0 || joint > 4) return false;

		return true;
	}


	// set gain of given joint in position controller
	void youBotArmEthercat::setControllerPositionGainConstant(int joint, float gain)
	{
		// set gain
		positionController->setP(joint,gain);
	}

	void youBotArmEthercat::getJointFirmware(std::string& j1, std::string& j2, std::string& j3, std::string& j4, std::string& j5)
	{
		if (!isInit) {
			j1 = j2 = j3 = j4 = j5 = "offline";
			return;
		}
		j1 = armAxis[0]->getJointFirmware();
		j2 = armAxis[1]->getJointFirmware();
		j3 = armAxis[2]->getJointFirmware();
		j4 = armAxis[3]->getJointFirmware();
		j5 = armAxis[4]->getJointFirmware();
	}

	void youBotArmEthercat::getJointParameter(int& j1, int& j2, int& j3, int& j4, int& j5, int ParamNumber)
	{
		if (!isInit) {
			j1 = j2 = j3 = j4 = j5 = 0;
			return;
		}
		j1 = armAxis[0]->getJointParameter(ParamNumber);
		j2 = armAxis[1]->getJointParameter(ParamNumber);
		j3 = armAxis[2]->getJointParameter(ParamNumber);
		j4 = armAxis[3]->getJointParameter(ParamNumber);
		j5 = armAxis[4]->getJointParameter(ParamNumber);
	}

	bool youBotArmEthercat::getJointError(int axis, int bit)
	{
		int ErrorFlag[18] =
		{ OVER_CURRENT, UNDER_VOLTAGE, OVER_VOLTAGE, OVER_TEMPERATURE, MOTOR_HALTED, HALL_SENSOR_ERROR, 0, 0,
				PWM_MODE_ACTIVE, VELOCITY_MODE, POSITION_MODE, TORQUE_MODE, 0, 0, POSITION_REACHED, INITIALIZED,
				TIMEOUT, I2T_EXCEEDED };

		bool ret = (msrError[axis - 1] & ErrorFlag[bit]);
		return ret;
	}

	void youBotArmEthercat::setEStop(bool estop)
	{
		isEStop = estop;
	}

	bool youBotArmEthercat::initEtherCat()
	{
		try {
			initArm();
			calibrateArm();
			startArm();
			RTT::log(RTT::Info) << "Arm successfully initialized." << RTT::endlog();

		} catch (...) {
			RTT::log(RTT::Info) << "Arm could not be created!" << RTT::endlog();
			return false;
		}
		return true;
	}

	void youBotArmEthercat::initArm()
	{
		for(int i=0; i < 5; i++) {
			if(armAxis[i]->getJointFirmware() != "1610V200") {
				RTT::log(RTT::Error) << "Unsupported firmware in axis " << i + 1 << ": " << armAxis[i]->getJointFirmware() << RTT::endlog();
				throw std::runtime_error("Unsupported firmware.");
			}
		}

		for(int i = 0; i < 5; i++) {
			armAxis[i]->initialize();
		}
		SLEEP_MILLISEC(200);

		// check for the next 5 sec if the joints are commutated
		bool isCommutated[5] = {0, 0, 0, 0, 0};
		bool allCommutated = false;
		for (int u = 1; u <= 5000 && !allCommutated; u++) {
			allCommutated = true;
			for (unsigned int i = 0; i < 5; i++) {
				if(armAxis[i]->getJointError() & INITIALIZED) {
					isCommutated[i] = true;
					armAxis[i]->setJointRoundsPerMinute(0);
				}
				allCommutated &= isCommutated[i];
			}
			SLEEP_MILLISEC(1);
		}

		for (int i = 0; i < 5; i++) {
			if (isCommutated[i] == true) {
				RTT::log(RTT::Info) << "Axis " << i + 1 << " is initialized!" << RTT::endlog();
			} else {
				RTT::log(RTT::Error) << "Axis " << i + 1 << " is not initialized, power off the arm and try again!" << RTT::endlog();
				throw std::runtime_error("Could not initialize! Power off the arm and try again!");
			}
		}
	}

	//Calibrate all manipulator joints
	void youBotArmEthercat::calibrateArm()
	{
		bool doCalibration[5];
		bool finished[5] = { 0, 0, 0, 0, 0 };

		for (int i = 0; i < 5; i++) {
			doCalibration[i] = !(armAxis[i]->IsCalibrated());
			if (doCalibration[i] == false) {
				RTT::log(RTT::Warning) << "Axis #" << i << " already calibrated!" << RTT::endlog();
			}
		}

		//move the joints slowly in calibration direction
		const int rpm[5] = { 156, 156, 99, 70, 70 };
		for (int i = 0; i < 5; i++) {
			if (doCalibration[i] == true) {
				armAxis[i]->setJointRoundsPerMinute(rpm[i]);
			} else {
				finished[i] = true;
			}
		}

		const double maxCurrent[5] = { 1.0, 0.5, 1.0, 0.5, 0.2 };
		bool allDone = false;
		//turn till a max current is reached;
		while (!allDone)
		{
			allDone = true;
			for(int i = 0; i < 5; i++) {
				if (abs(armAxis[i]->getSensedCurrent()) > maxCurrent[i] * si::ampere) {
					armAxis[i]->setJointCurrent(0);
					finished[i] = true;
				}
				allDone &= finished[i];
			}
			SLEEP_MILLISEC(1);
		}

		SLEEP_MILLISEC(100);

		for (int i = 0; i < 5; i++) {
			if (doCalibration[i] == true) {
				armAxis[i]->sendIsCalibratedMessage();
			}
		}
		SLEEP_MILLISEC(100);

	}

	void youBotArmEthercat::startArm()
	{
		RTT::log(RTT::Info) << "Starting arm..." << RTT::endlog();

		// disable ramp
		for (int i = 0; i < 5; i++) {
			armAxis[i]->setJointParameter(146, 0);
		}

		for (int i = 0; i < 5; i++) {
			msrPos[i] = jointDirection[i] * (armAxis[i]->getJointSensedAngle() - jointDirection[i] * JointAngleToZero[i]);
			if(msrPos[i] < minAngle[i]) msrPos[i] = minAngle[i];
			if(msrPos[i] > maxAngle[i]) msrPos[i] = maxAngle[i];
			setJointPositionStatic(i, msrPos[i]);
		}
		resetTimeHistory();
	}

	//Calibrate the gripper
	void youBotArmEthercat::calibrateGripperBlocking()
	{
		armAxis[4]->calibrateGripperBlocking(maxEncoderValueGripper);
		resetTimeHistory();
	}

	// set Torque to given joint
	void youBotArmEthercat::setTorque(int joint, double torque)
	{
		// security check
		if(joint < 0 || joint > YOUBOT_NUMAXIS-1) return;

		// don't allow torques towards end stops when closer than 5° before end stop
		double eps = 5.0 /180*KDL::PI;

		if(msrPos[joint] > maxAngle[joint]-eps && torque > 0) {
			torque = 0;
		}

		if(msrPos[joint] < minAngle[joint]+eps && torque < 0) {
			torque = 0;
		}

		dumpECTorque[joint].put(torque);
		armAxis[joint]->setJointTorque(torque * jointDirection[joint]);
	}

	// set Velocity to given joint
	void youBotArmEthercat::setVelocity(int joint, double velocity)
	{
		// security check
		if(joint < 0 || joint > YOUBOT_NUMAXIS-1) return;

		dumpECVel[joint].put(velocity);
		armAxis[joint]->setJointVelocity(velocity*jointDirection[joint]);
	}


	//set the gripper position
	//@param value the position
	bool youBotArmEthercat::setGripperPositionBlocking(float gripperPosition)
	{
		int pos = (int) ((gripperPosition + gripperOffset) / maxGripperDistance * maxEncoderValueGripper);

		RTT::log(RTT::Info) << "Moving gripper. " << RTT::endlog();
		while (!armAxis[4]->sendGripperPosition(pos, 0)) SLEEP_MILLISEC(10);
		while (!armAxis[4]->receiveGripperMessage()) SLEEP_MILLISEC(10);
		while (!armAxis[4]->sendGripperPosition(pos, 1)) SLEEP_MILLISEC(10);
		while (!armAxis[4]->receiveGripperMessage()) SLEEP_MILLISEC(10);

		bool finished = false;
		while(!finished) {
			while(!armAxis[4]->sendGripperFinished(0));
			while(!armAxis[4]->receiveGripperFinished(finished));
			SLEEP_MILLISEC(50);
		}
		finished = false;
		while(!finished) {
			while(!armAxis[4]->sendGripperFinished(1));
			while(!armAxis[4]->receiveGripperFinished(finished));
			SLEEP_MILLISEC(50);
		}
		RTT::log(RTT::Info) << "Gripper finished. " << RTT::endlog();

		return true;
	}

	RPI::DeviceState youBotArmEthercat::getDeviceState() const
	{
		return isInit ? (isEStop ? RPI::DeviceState::SAFE_OPERATIONAL : RPI::DeviceState::OPERATIONAL) : RPI::DeviceState::OFFLINE;
	}

	// RobotArmDriver implementation
	int youBotArmEthercat::getJointCount() const {
		return 5;
	}

	int youBotArmEthercat::getJointError(int joint) {
		return isEStop ? 1 : isInit ? 0 : 2;
	}

	robotarm::JointPositionError youBotArmEthercat::checkJointPosition(int joint, double position) {
		if(position != position) return robotarm::JP_INVALID;

		if(joint < 0 || joint >= YOUBOT_NUMAXIS)
			return robotarm::JP_INVALID;

		if(position > maxAngle[joint] || position  < minAngle[joint])
			return robotarm::JP_OUTOFRANGE;

		return robotarm::JP_OK;
	}

	double youBotArmEthercat::getMeasuredJointPosition(int joint) {
		return msrPos[joint];
	}

	double youBotArmEthercat::getMeasuredJointVelocity(int joint) {
		return msrVel[joint];
	}

	double youBotArmEthercat::getMeasuredJointAcceleration(int joint) {
		return msrAcc[joint];
	}

	void youBotArmEthercat::setToolCOM(KDL::Vector com, int axis)
	{
		if (axis!= 4) return;

		jointImpedanceController->setToolCOM(com);
	}

	void youBotArmEthercat::setToolMOI(KDL::Vector moi, int axis)
	{
		if (axis != 4) return;

		jointImpedanceController->setToolMOI(moi);
	}

	void youBotArmEthercat::setToolMass(double mass, int axis)
	{
		if (axis!= 4) return;

		jointImpedanceController->setToolMass(mass);
	}

	bool youBotArmEthercat::getToolFinished(int axis) const
	{
		return true;
	}

	int youBotArmEthercat::getToolError(int axis) const
	{
		return 0;
	}

	KDL::Frame youBotArmEthercat::Kin(const RPI::Array<double>& joints) {
		KDL::Frame pos;
		kin.ybKin(joints.get(0), joints.get(1), joints.get(2), joints.get(3), joints.get(4), pos);
		return pos;
	}

	void youBotArmEthercat::InvKin(const RPI::Array<double>& hintjoints, const KDL::Frame& position, RPI::Array<double>& resultJoints) {
		for(int i=0; i<5; i++) resultJoints[i] = hintjoints.get(i);
//		invkin.invKin(position, resultJoints[0], resultJoints[1], resultJoints[2], resultJoints[3], resultJoints[4]);
		kin.ybInvKin(position, resultJoints[0], resultJoints[1], resultJoints[2], resultJoints[3], resultJoints[4]);
	}

} //End of namespace yb
