/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef OTG2_HPP_
#define OTG2_HPP_

#include <rcc/Module.hpp>

namespace Core
{

	class OTG2: public RPI::ActiveModule
	{
	private:
		double samplingTime;

		double curPos, curVel;
		double desPos, desVel;

		double calcAcceleration(double desPos, double desVel, double curPos, double curVel, double maxVel, double maxAcc);
		double calcVelocity(double curVel, double acc);
		double calcPosition(double curPos, double newVel, double curVel);
		
		double sat(double x);
		int sign(double x);
	protected:
		RPI::InPort<double> inCurPos;
		RPI::InPort<double> inCurVel;
		RPI::InPort<double> inDesPos;
		RPI::InPort<double> inMaxVel;
		RPI::InPort<double> inMaxAcc;

		RPI::OutPort<double> outPos;
		RPI::OutPort<double> outVel;
		RPI::OutPort<double> outAcc;

		RPI::Property<double> propMaxVel;
		RPI::Property<double> propMaxAcc;
	public:
		OTG2(std::string name, RPI::Net* net);
		virtual ~OTG2();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	};
}
#endif /* OTG2_HPP_ */
