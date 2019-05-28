/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "OTG2.hpp"
#include <rcc/Net.hpp>
#include <stdlib.h>
#include <math.h>

namespace Core
{
	using namespace RPI;
	using namespace std;

	OTG2::OTG2(std::string name, RPI::Net* net) :
			ActiveModule(name, net), samplingTime(0), curPos(0), curVel(0), desPos(0), desVel(0),
				inCurPos("inCurPos", this), inCurVel("inCurVel", this), inDesPos("inDesPos", this),
				inMaxVel("inMaxVel", this), inMaxAcc("inMaxAcc", this),
				outPos("outPos", this), outVel("outVel", this), outAcc("outAcc", this),
				propMaxVel("maxVel", "Maximum velocity planned by OTG"),
				propMaxAcc("maxAcc", "Maximum acceleration planned by OTG")
	{
		this->setDescription("Online trajectory generator for single axis with velocity and acceleration constraints.");
		this->ports()->addPort(&inCurPos, "Current position");
		this->ports()->addPort(&inCurVel, "Current velocity");
		this->ports()->addPort(&inDesPos, "Desired position");
		this->ports()->addPort(&inMaxVel, "Maximum velocity planned by OTG");
		this->ports()->addPort(&inMaxAcc, "Maximum acceleration planned by OTG");
		this->ports()->addPort(&outPos, "New position");
		this->ports()->addPort(&outVel, "New velocity");
		this->ports()->addPort(&outAcc, "New acceleration");
		this->properties()->addProperty(&propMaxVel);
		this->properties()->addProperty(&propMaxAcc);
	}

	OTG2::~OTG2()
	{
		// do nothing...
	}

	bool OTG2::configureHook()
	{
		samplingTime = inNet->getNetFrequency();

		return true;
	}

	bool OTG2::startHook()
	{
		return true;
	}

	void OTG2::updateHook()
	{
		if (active())
		{
			double maxVel = inMaxVel.Get(propMaxVel);
			double maxAcc = inMaxAcc.Get(propMaxAcc);

			if(maxAcc > 2 * maxVel / samplingTime) {
				maxAcc = 2 * maxVel / samplingTime;
			}

			if (inCurPos.connected())
			{
				curPos = inCurPos.Get();
			}

			if (inCurVel.connected())
			{
				curVel = inCurVel.Get();
			}

			double lastPos = desPos;
			desPos = inDesPos.Get();
			desVel = (desPos - lastPos) / samplingTime;

			double acc = calcAcceleration(desPos, desVel, curPos, curVel, maxVel, maxAcc);
			double newVel = calcVelocity(curVel, acc);
			double newPos = calcPosition(curPos, newVel, curVel);

			outPos.Set(newPos);
			outVel.Set(newVel);
			outAcc.Set(acc);

			curPos = newPos;
			curVel = newVel;
		}
	}

	void OTG2::stopHook()
	{
	}

	void OTG2::cleanupHook()
	{
	}

	double OTG2::calcAcceleration(double desPos, double desVel, double curPos, double curVel, double maxVel, double maxAcc)
	{
		double ek = (curPos - desPos) / maxAcc;
		double ek2 = (curVel - desVel) / maxAcc;

		double zk = (1 / samplingTime) * ((ek / samplingTime) + (ek2 / 2));
		double zk2 = ek2 / samplingTime;

		double m = floor((1 + sqrt(1 + 8 * fabs(zk))) / 2);
		double sigma = zk2 + zk / m + ((m - 1) / 2) * sign(zk);

		curVel = curVel * sign(sigma);
		double step = curVel + maxVel - samplingTime * maxAcc;

		if(step < 0) {
			maxAcc = (maxVel + curVel) / samplingTime;
			//step = 0;
		}
		return (-maxAcc) * sat(sigma); // * (1 + sign(step)) / 2;
	}

	double OTG2::calcVelocity(double curVel, double acc)
	{
		return curVel + samplingTime * acc;
	}

	double OTG2::calcPosition(double curPos, double newVel, double curVel)
	{
		return curPos + samplingTime / 2 * (newVel + curVel);
	}

	double OTG2::sat(double x)
	{
		if (x < -1)
			return -1;
		else if (x > 1)
			return 1;
		else
			return x;
	}

	int OTG2::sign(double x)
	{
		if (x < 0)
			return -1;
		else
			return 1;
	}
}
