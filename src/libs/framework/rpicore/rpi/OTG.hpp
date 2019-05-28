/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef OTG_HPP_
#define OTG_HPP_

#include <rcc/Module.hpp>

namespace Core
{

	struct OTGData
	{
		double Acceleration;
		double Velocity;
		double Position;
	};

	class OTG: public RPI::ActiveModule
	{
	private:
		double samplingTime;

		double maxAcceleration;
		double maxVelocity;

		// variables to store values of last cycle, if no robot monitor is available
		double lastPosition, lastVelocity;
		double lastCmdPosition;

		double calculateDesiredAcceleration(double position, double velocity, double currentPosition,
				double currentVelocity);
		double sat(double x);
		int sign(double x);
	protected:
		RPI::InPort<double> inCurPos;
		RPI::InPort<double> inCurVel;
		RPI::InPort<double> inCurAcc;
		RPI::InPort<double> inDestPos;
		RPI::InPort<double> inDestVel;
		RPI::InPort<double> inMaxVel;
		RPI::InPort<double> inMaxAcc;

		RPI::OutPort<double> outPos;
		RPI::OutPort<double> outVel;
		RPI::OutPort<double> outAcc;

		RPI::Property<double> propMaxVel;
		RPI::Property<double> propMaxAcc;
	public:
		OTG(std::string name, RPI::Net* net);
		virtual ~OTG();

		OTGData calculate(double position, double velocity, double currentPosition, double currentVelocity,
				double currentAcceleration);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	};
}
#endif /* OTG_HPP_ */
