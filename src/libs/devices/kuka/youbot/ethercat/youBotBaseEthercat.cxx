/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "youBotBaseEthercat.hpp"
#include "EthercatProtocolDefinitions.hpp"
#include <rtt/Activity.hpp>

namespace kuka_youbot
{
	using namespace std;

	youBotBaseEthercat::youBotBaseEthercat(std::string name, RPI::parameter_t parameters) :
			TaskContext("youbot_base_" + name), RPI::Device(name, parameters), isEStop(false),
			master(getParameter("ethercatdevice")),
			isInit(false), startInit(false),
			msrPos(), msrVel(), rpiVel(), rpiTime(0), ecTime(0)
	{
		baseNr = getParameterT<double>("base", 0);

		IDX_FRONTLEFT = 0;
		IDX_FRONTRIGHT = 1;
		IDX_REARLEFT = 2;
		IDX_REARRIGHT = 3;
		direction[IDX_FRONTLEFT] = -1;
		direction[IDX_FRONTRIGHT] = 1;
		direction[IDX_REARLEFT] = -1;
		direction[IDX_REARRIGHT] = 1;


		for (int i = 0; i < 4; i++)  {
			slaves[i] = new YoubotEthercatSlave(this);
			baseAxes[i] = 0;
		}


		// set up crash dumpers
		dumpMsrPos = new RPI::RoundRobinLog<double>[3];
		dumpCmdPos = new RPI::RoundRobinLog<double>[3];
		dumpMsrVel = new RPI::RoundRobinLog<double>[3];
		dumpCmdVel = new RPI::RoundRobinLog<double>[3];
		dumpRpiVel = new RPI::RoundRobinLog<double>[3];
		dumpMsrWheelPos = new RPI::RoundRobinLog<double>[4];
		dumpMsrWheelVel= new RPI::RoundRobinLog<double>[4];
		dumpCmdWheelVel = new RPI::RoundRobinLog<double>[4];

		for (int i = 0; i < 4; i++)
		{
			std::stringstream wheel; wheel << "Wheel " << i;
			dumpMsrWheelPos[i].setName(wheel.str() + "|Position|Measured");
			dumpMsrWheelVel[i].setName(wheel.str() + "|Velocity|Measured");
			dumpCmdWheelVel[i].setName(wheel.str() + "|Velocity|Commanded");
			addCrashDumper(&dumpMsrWheelPos[i]);
			addCrashDumper(&dumpMsrWheelVel[i]);
			addCrashDumper(&dumpCmdWheelVel[i]);
		}

		for(int i=0; i<3; i++) {
			std::string component = i==0?"X":(i==1?"Y":"Yaw");
			dumpMsrPos[i].setName(component + "|Position|Measured");
			dumpCmdPos[i].setName(component + "|Position|Commanded");
			dumpMsrVel[i].setName(component + "|Velocity|Measured");
			dumpCmdVel[i].setName(component + "|Velocity|Commanded");
			dumpRpiVel[i].setName(component + "|Velocity|RPI");

			addCrashDumper(&dumpMsrPos[i]);
			addCrashDumper(&dumpCmdPos[i]);
			addCrashDumper(&dumpMsrVel[i]);
			addCrashDumper(&dumpCmdVel[i]);
			addCrashDumper(&dumpRpiVel[i]);
		}

		for(const auto& cd: CartesianPositionDevice::getCrashDumpers()) {
			addCrashDumper(cd);
		}
		this->setActivity(new RTT::Activity(RTT::os::LowestPriority, 0, 0, name));
		master->addDevice(this);
	}


	void youBotBaseEthercat::startupDevice()
	{
		std::string slaveName = "TMCM-1632";
		int startId = baseNr * 4;
		int id = 0;
		for(const auto& slave: master->getSlaveNames())
		{
			if(slaveName == slave.slaveName) {
				if(id >= startId + 4) break;
				if(id >= startId) {
					delete baseAxes[id - startId];
					baseAxes[id - startId] = new ybAxis(slaves[id - startId], id - startId + 1, 9405 / 364, 0.0335);
					master->addSlave(slave.slaveID, slaves[id - startId]);
				}
				id++;
			}
		}
		if(id - startId == 4) {
			startInit = isInit = false;

		} else {
			startInit = isInit = false;
			for(int i=startId; i < id; i++)
				master->removeSlave(slaves[i - startId]);

		}
	}

	void youBotBaseEthercat::shutdownDevice()
	{
		isInit = false;
		for(int i=0; i<5; i++)
		{
			master->removeSlave(slaves[i]);
		}
	}

	void youBotBaseEthercat::updateDevice()
	{
		if(!isInit)
			return;

		RTT::os::TimeService::nsecs now = RTT::os::TimeService::Instance()->getNSecs();

		// calculate position
		{
			double x, y, yaw;
			double fl = baseAxes[IDX_FRONTLEFT]->getJointSensedAngle() * direction[IDX_FRONTLEFT];
			double fr = baseAxes[IDX_FRONTRIGHT]->getJointSensedAngle() * direction[IDX_FRONTRIGHT];
			double rl = baseAxes[IDX_REARLEFT]->getJointSensedAngle() * direction[IDX_REARLEFT];
			double rr = baseAxes[IDX_REARRIGHT]->getJointSensedAngle() * direction[IDX_REARRIGHT];

			dumpMsrWheelPos[0].put(fl);
			dumpMsrWheelPos[1].put(fr);
			dumpMsrWheelPos[2].put(rl);
			dumpMsrWheelPos[3].put(rr);

			kin.posKin(
					fl - wheelPos[IDX_FRONTLEFT],
					fr - wheelPos[IDX_FRONTRIGHT],
					rl - wheelPos[IDX_REARLEFT],
					rr - wheelPos[IDX_REARRIGHT],
					x, y, yaw);

			msrPos = msrPos * KDL::Frame(KDL::Rotation::RotZ(yaw), KDL::Vector(x, y, 0));
			getPosition(x, y, yaw);
			dumpMsrPos[0].put(x);
			dumpMsrPos[1].put(y);
			dumpMsrPos[2].put(yaw);

			wheelPos[IDX_FRONTLEFT] = fl;
			wheelPos[IDX_FRONTRIGHT] = fr;
			wheelPos[IDX_REARLEFT] = rl;
			wheelPos[IDX_REARRIGHT] = rr;
		}

		// calculate velocity
		{
			double x, y, yaw;
			double fl = baseAxes[IDX_FRONTLEFT]->getJointVelocity() * direction[IDX_FRONTLEFT];
			double fr = baseAxes[IDX_FRONTRIGHT]->getJointVelocity() * direction[IDX_FRONTRIGHT];
			double rl = baseAxes[IDX_REARLEFT]->getJointVelocity() * direction[IDX_REARLEFT];
			double rr = baseAxes[IDX_REARRIGHT]->getJointVelocity() * direction[IDX_REARRIGHT];

			double msrVelX, msrVelY, msrVelYaw;
			kin.velKin(fl, fr, rl, rr, msrVelX, msrVelY, msrVelYaw);
			msrVel = KDL::Twist(KDL::Vector(msrVelX, msrVelY, 0), KDL::Vector(0, 0, msrVelYaw));

			dumpMsrVel[0].put(msrVelX);
			dumpMsrVel[1].put(msrVelY);
			dumpMsrVel[2].put(msrVelYaw);

			dumpMsrWheelVel[0].put(fl);
			dumpMsrWheelVel[1].put(fr);
			dumpMsrWheelVel[2].put(rl);
			dumpMsrWheelVel[3].put(rr);
		}

		// do control

		double velX, velY, velYaw;

		// when estop is active, reset goal and velocity
		if(isEStop) {
			rpiVel = KDL::Twist::Zero();
			initPosition(msrPos, KDL::Twist::Zero());
		}

		if(cmdTime > rpiTime) {
			// position control mode
			KDL::Frame goal = getCommandedPosition();
			KDL::Twist ff = getCommandedVelocity();
			if (RTT::os::TimeService::Instance()->getNSecs(cmdTime) > 0.2*1e9)
			{
				// no value for 0.2s -> assume standstill
				ff = KDL::Twist::Zero();
			}

			KDL::Frame err = getMeasuredPosition().Inverse()*goal;
			double errX, errY, errYaw;
			errX = err.p.x();
			errY = err.p.y();
			errYaw = err.M.GetRot().z();

			double px = 3, py = 3, pyaw = 3;
			velX = ff.vel.x() + errX * px;
			velY = ff.vel.y() + errY * py;
			velYaw = ff.rot.z() + errYaw * pyaw;

			dumpCmdPos[0].put(goal.p.x());
			dumpCmdPos[1].put(goal.p.y());
			dumpCmdPos[2].put(goal.M.GetRot().z());

		} else {
			// velocity control mode
			if (RTT::os::TimeService::Instance()->getNSecs(rpiTime) > 0.2*1e9)
			{
				// no value for 0.2s -> assume standstill
				rpiVel = KDL::Twist::Zero();
			}

			KDL::Frame pos = getCommandedPosition();
			pos = KDL::addDelta(pos, pos.M.Inverse(rpiVel), (now - ecTime) / 1e9);
			initPosition(pos, rpiVel);

			velX = rpiVel.vel.x();
			velY = rpiVel.vel.y();
			velYaw = rpiVel.rot.z();
		}

		// handle e-stop
		if(isEStop) {
			velX = 0;
			velY = 0;
			velYaw = 0;
		}

		// limit maximum velocity
		double maxX = 0.8, maxY = 0.8, maxYaw = 0.8;
		double factor = 1;
		factor = std::max(factor, velX / maxX);
		factor = std::max(factor, velY / maxY);
		factor = std::max(factor, velYaw / maxYaw);
		velX = velX / factor;
		velY = velY / factor;
		velYaw = velYaw / factor;

		// command velocity
		{
			double fl, fr, rl, rr;
			kin.velInvKin(velX, velY, velYaw, fl, fr, rl, rr);
			baseAxes[IDX_FRONTLEFT]->setJointVelocity(fl * direction[IDX_FRONTLEFT]);
			baseAxes[IDX_FRONTRIGHT]->setJointVelocity(fr * direction[IDX_FRONTRIGHT]);
			baseAxes[IDX_REARLEFT]->setJointVelocity(rl * direction[IDX_REARLEFT]);
			baseAxes[IDX_REARRIGHT]->setJointVelocity(rr * direction[IDX_REARRIGHT]);

			dumpCmdVel[0].put(velX);
			dumpCmdVel[1].put(velY);
			dumpCmdVel[2].put(velYaw);

			dumpCmdWheelVel[0].put(fl);
			dumpCmdWheelVel[1].put(fr);
			dumpCmdWheelVel[2].put(rl);
			dumpCmdWheelVel[3].put(rr);
		}

		ecTime = now;
	}

	void youBotBaseEthercat::inSafeOp(int slaveno)
	{
		master->doSlaveOp(slaveno);

		bool allop = true;
		for (int i=0; i<4; i++)
		{
			allop &= slaves[i]->isinOp();
		}

		if(allop)
		{
			startInit = true;
			trigger();
		}
	}

	youBotBaseEthercat::~youBotBaseEthercat()
	{
		this->stop();

		for(int i=0; i<4; i++) {
			delete baseAxes[i]; baseAxes[i] = 0;
			master->removeSlave(slaves[i]);
			delete slaves[i]; slaves[i] = 0;
		}
		master->removeDevice(this);
	}

	youBotBaseEthercat* youBotBaseEthercat::createDevice(std::string name, RPI::parameter_t parameters)
	{
		youBotBaseEthercat* ret = new youBotBaseEthercat(name, parameters);
		ret->configure();
		ret->start();
		return ret;
	}

	void youBotBaseEthercat::updateParameters()
	{

	}

	void youBotBaseEthercat::setEStop(bool estop)
	{
		this->isEStop = estop;
	}

	std::set<std::string> youBotBaseEthercat::getMutableParameters() const
	{
		return std::set<std::string>();
	}

	bool youBotBaseEthercat::startHook()
	{
		return true;
	}

	bool youBotBaseEthercat::configureHook()
	{
		return true;
	}

	void youBotBaseEthercat::initialize()
	{
		for(int i=0; i < 4; i++) {
			if(baseAxes[i]->getJointFirmware() != "1632V200") {
				RTT::log(RTT::Error) << "Unsupported firmware in axis " << i + 1 << ": " << baseAxes[i]->getJointFirmware() << RTT::endlog();
				throw std::runtime_error("Unsupported firmware.");
			}
		}


		for (int i = 0; i < 4; i++)
		{
			baseAxes[i]->initialize();
		}

		std::vector<bool> isCommutated;
		isCommutated.assign(4, false);
		unsigned int u = 0;

		// check for the next 5 sec if the joints are commutated
		for (u = 1; u <= 5000; u++)
		{
			for (unsigned int i = 0; i < 4; i++)
			{
				if (baseAxes[i]->getJointError() & INITIALIZED) {
					isCommutated[i] = true;
					baseAxes[i]->setJointRoundsPerMinute(0);
				}
			}
			if (isCommutated[0] && isCommutated[1] && isCommutated[2] && isCommutated[3])
			{
				break;
			}
			SLEEP_MILLISEC(1);
		}

		for (int i = 0; i < 4; i++)
		{
			if (isCommutated[i] == true)
				RTT::log(RTT::Info) << "Axis " << i + 1 << " is initialized!" << RTT::endlog();
			else
			{
				RTT::log(RTT::Error) << "Axis " << i + 1 << " is not initialized, power off the base and try again!" << RTT::endlog();
				throw std::runtime_error("Could not initialize! Power off the base and try again!");
			}
		}

		SLEEP_MILLISEC(50);

		for (int i = 0; i < 4; i++)
		{
			if (!baseAxes[i]->IsCalibrated())
			{
				baseAxes[i]->sendIsCalibratedMessage();
				RTT::log() << "Axis " << i << " calibrated" << RTT::endlog();

			}
			baseAxes[i]->setEncoderToZero();
			wheelPos[i] = 0;
		}
		SLEEP_MILLISEC(100);

		isInit = true;
	}

	void youBotBaseEthercat::updateHook()
	{
		if(startInit)
		{
			startInit = false;
			initialize();
		}
	}

	bool youBotBaseEthercat::setVelocity(double forward, double left, double yaw)
	{
		bool success = false;
		if (isEStop)
		{
			rpiVel = KDL::Twist::Zero();
		} else
		{
			rpiVel = KDL::Twist(KDL::Vector(forward, left, 0), KDL::Vector(0, 0, yaw));
			success = true;
		}

		rpiTime = RTT::os::TimeService::Instance()->getNSecs();
		dumpRpiVel[0].put(rpiVel.vel.x(), rpiTime);
		dumpRpiVel[1].put(rpiVel.vel.y(), rpiTime);
		dumpRpiVel[2].put(rpiVel.rot.z(), rpiTime);
		return success;
	}

	void youBotBaseEthercat::getPosition(double& x, double& y, double& yaw)
	{
		x = msrPos.p.x();
		y = msrPos.p.y();
		double r, p;
		yaw = msrPos.M.GetRot().z();
	}

	void youBotBaseEthercat::getVelocity(double& x, double& y, double& yaw)
	{
		x = msrVel.vel.x();
		y = msrVel.vel.y();
		yaw = msrVel.rot.z();
	}

	int youBotBaseEthercat::getBaseError()
	{
		return isEStop ? 1 : isInit ? 0 : 2;
	}

	int youBotBaseEthercat::checkBaseVelocity(KDL::Twist velocity)
	{
		return 0;
	}
	void youBotBaseEthercat::setBaseVelocity(KDL::Twist velocity)
	{
		setVelocity(velocity.vel.x(), velocity.vel.y(), velocity.rot.z());
	}
	KDL::Frame youBotBaseEthercat::getMeasuredBasePosition()
	{
		double x, y, yaw;
		getPosition(x, y, yaw);
		return KDL::Frame(KDL::Rotation::RotZ(yaw), KDL::Vector(x, y, 0));
	}

	KDL::Twist youBotBaseEthercat::getMeasuredBaseVelocity() {
		double x, y, yaw;
		getVelocity(x, y, yaw);
		return KDL::Twist(KDL::Vector(x, y, 0), KDL::Vector(0, 0, yaw));
	}

	KDL::Twist youBotBaseEthercat::getCommandedBaseVelocity() {
		if(rpiTime > cmdTime)
		{
			return rpiVel;
		}
		else
		{
			return getCommandedVelocity();
		}
	}

	RPI::DeviceState youBotBaseEthercat::getDeviceState() const {
		return isInit ? (isEStop ? RPI::DeviceState::SAFE_OPERATIONAL : RPI::DeviceState::OPERATIONAL) : RPI::DeviceState::OFFLINE;
	}

	int youBotBaseEthercat::getWheelCount() const
	{
		return 4;
	}
	double youBotBaseEthercat::getWheelPosition(int wheel) const
	{
		if(wheel < 0 || wheel > 4 || !isInit) return 0;
		return baseAxes[wheel]->getJointSensedAngle() * direction[wheel];
	}
	double youBotBaseEthercat::getWheelVelocity(int wheel) const
	{
		if(wheel < 0 || wheel > 4 || !isInit) return 0;
		return baseAxes[wheel]->getJointVelocity() * direction[wheel];
	}

	std::string youBotBaseEthercat::getRobotBaseResourceName()
	{
		return Device::getName();
	}


	int youBotBaseEthercat::getCartesianPositionDeviceError()
	{
		return getBaseError();
	}

	std::string youBotBaseEthercat::getCartesianPositionResourceName()
	{
		return getRobotBaseResourceName();
	}

	int youBotBaseEthercat::checkPosition(KDL::Frame position)
	{
		if(fabs(position.p.z()) > 1e-3 ||
				fabs(position.M.UnitZ().x()) > 1e-3  ||
				fabs(position.M.UnitZ().y()) > 1e-3) {
			return 1;
		}
		return 0;
	}

	KDL::Frame youBotBaseEthercat::getMeasuredPosition()
	{
		return getMeasuredBasePosition();
	}

	KDL::Twist youBotBaseEthercat::getMeasuredVelocity()
	{
		return getMeasuredBaseVelocity();
	}



} /* namespace youbot */
