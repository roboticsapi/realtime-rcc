/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CYCLICPOSITIONROBOTARM_HPP_
#define CYCLICPOSITIONROBOTARM_HPP_

#include <istream>
#include <list>
#include "../interface/RobotArmInterface.hpp"
#include "rcc/RoundRobinLog.hpp"
#include <rtt/rtt-config.h>

namespace robotarm
{

	class RTT_EXPORT CyclicPositionRobotArm: virtual public robotarm::RobotArmInterface
	{
	public:

		/**
		 * \brief Initialize a CyclicPositionRobotArm
		 * \param jointCount	number of joints
		 * \param smoothLength	amount of smoothing of the position:
		 * 				0  to disable interpolation (return the latest commanded position)
		 * 				1  to disable smoothing (return the interpolated position at the given time)
		 * 				n  interpolate smoothing the given time using a sliding average of n samples
		 */
		CyclicPositionRobotArm(int jointCount, int smoothLength);
		virtual ~CyclicPositionRobotArm();

		// methods implemented for RobotArmDriver
		virtual void setJointPosition(int joint, double position, RTT::os::TimeService::nsecs time);
		virtual void setJointPositionStatic(int joint, double position);
		virtual double getCommandedJointPosition(int joint);
		virtual double getCommandedJointVelocity(int joint);
		virtual double getCommandedJointAcceleration(int joint);

		/**
		 * \brief Retrieves the maximum acceleration of a joint
		 * \param joint		number of joint
		 * \return maximum acceleration in rad/s²
		 */
		virtual double getMaximumAcceleration(int joint) const = 0;

		/**
		 * \brief Retrieves the maximum velocity of a joint
		 * \param joint		number of joint
		 * \return maximum velocity in rad/s
		 */
		virtual double getMaximumVelocity(int joint) const = 0;

		virtual std::list<RPI::CrashDumper*> getCrashDumpers() const;

	protected:

		/**
		 * \brief Reset the time history (use when continuing after a pause)
		 */
		virtual void resetTimeHistory();

		/**
		 * \brief Retrieves the position and velocity to command "now"
		 * \param position	double array accepting the position (in rad)
		 * \param velocity	double array accepting the velocity (in rad/s)
		 */
		virtual void getValuesToCommand(double* position, double* velocity);

		/**
		 * \brief Retrieves the position and velocity to command "now"
		 * \param position	double array accepting the position (in rad)
		 * \param velocity	double array accepting the velocity (in rad/s)
		 * \param cycletime	cycle time (smoothed time since last call)
		 */
		virtual void getValuesToCommand(double* position, double* velocity, double& cycletime);

		/**
		 * \brief Retrieves the position and velocity to command at a given time near "now"
		 * \param ticks		time to calculate the position for
		 * \param position	double array accepting the position (in rad)
		 * \param velocity	double array accepting the velocity (in rad/s)
		 */
		virtual void getValuesToCommand(RTT::os::TimeService::nsecs currentTicks, double* position, double* velocity);

		/**
		 * \brief Retrieves the position and velocity to command at a given time near "now"
		 * \param ticks		time to calculate the position for
		 * \param position	double array accepting the position (in rad)
		 * \param velocity	double array accepting the velocity (in rad/s)
		 * \param cycletime	cycle time (smoothed time since last call)
		 **/
		virtual void getValuesToCommand(RTT::os::TimeService::nsecs currentTicks, double* position, double* velocity, double& cycletime);


	private:
		int jointCount;

		// data from RPI net
		double *RPIPos, *RPIVel, *RPIAcc;
		RTT::os::TimeService::nsecs *RPITicks;

		// data from RPI net at last CP cycle
		double *RPIPos2, *RPIPos1;
		RTT::os::TimeService::nsecs *RPITicks2, *RPITicks1;

		// data from last CP cycle
		double *CPPos, *CPVel;
		RTT::os::TimeService::nsecs lastCPTicks, lastRealTicks;
		RTT::os::TimeService::nsecs tickDiffSum, *tickDiffs;
		int tickDiffCount, tickDiffPos;

		RPI::RoundRobinLog<double>* dumpRPIPos;
		RPI::RoundRobinLog<double>* dumpRPIVel;
		RPI::RoundRobinLog<double>* dumpCmdPos;
		RPI::RoundRobinLog<double>* dumpCmdVel;
		RPI::RoundRobinLog<double>* dumpCmdAge;
		RPI::RoundRobinLog<double>* dumpCmdSmoothAge;

	};

} /* namespace robotarm */
#endif /* CYCLICPOSITIONROBOTARM_HPP_ */
