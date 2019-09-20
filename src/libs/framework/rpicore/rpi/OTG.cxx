/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "OTG.hpp"
#include <rcc/Net.hpp>
#include <stdlib.h>
#include <math.h>

namespace Core
{
	using namespace RPI;
	using namespace std;

	OTG::OTG(std::string name, RPI::Net* net) :
			ActiveModule(name, net), samplingTime(0), maxAcceleration(0), maxVelocity(0), lastPosition(0), lastVelocity(
					0), lastCmdPosition(0), inCurPos("inCurPos", this), inCurVel("inCurVel", this), inCurAcc("inCurAcc",
					this), inDestPos("inDestPos", this), inDestVel("inDestVel", this), inMaxVel("inMaxVel", this), inMaxAcc(
					"inMaxAcc", this), outPos("outPos", this), outVel("outVel", this), outAcc("outAcc", this), propMaxVel(
					"maxVel", "Maximum velocity planned by OTG"), propMaxAcc("maxAcc",
					"Maximum acceleration planned by OTG")
	{
		this->setDescription("Online Trajectory Generator for single axis");
		this->ports()->addPort(&inCurPos, "current position");
		this->ports()->addPort(&inCurVel, "current velocity");
		this->ports()->addPort(&inCurAcc, "current acceleration");
		this->ports()->addPort(&inDestPos, "destination position");
		this->ports()->addPort(&inDestVel, "destination velocity");
		this->ports()->addPort(&inMaxVel, "maximum velocity planned by OTG");
		this->ports()->addPort(&inMaxAcc, "maximum acceleration planned by OTG");
		this->ports()->addPort(&outPos, "new position");
		this->ports()->addPort(&outVel, "new velocity");
		this->ports()->addPort(&outAcc, "new acceleration");
		this->properties()->addProperty(&propMaxVel);
		this->properties()->addProperty(&propMaxAcc);

	}

	OTG::~OTG()
	{
		// TODO Auto-generated destructor stub
	}

	bool OTG::configureHook()
	{
		samplingTime = inNet->getNetFrequency();
		lastPosition = numeric_limits<double>::quiet_NaN();
		lastVelocity = numeric_limits<double>::quiet_NaN();
		lastCmdPosition = 0;

		/*if (!inCurPosition.connected())
		 return false;*/

		return true;
	}

	bool OTG::startHook()
	{
		return true;
	}

	void OTG::updateHook()
	{

		if (active())
		{
			this->maxVelocity = inMaxVel.Get(propMaxVel);
			this->maxAcceleration = inMaxAcc.Get(propMaxAcc);

			// Workaround (aka Hack) to avoid the need of a robot monitor
			double curposition;
			double curvelocity = 0, curacceleration = 0;

			if (!inCurPos.connected())
			{
				curposition = lastCmdPosition;
			} else
			{
				curposition = inCurPos.Get();
			}

			if (inCurVel.connected())
			{
				curvelocity = inCurVel.Get();
			} else
			{
				if (lastPosition == lastPosition)
				{
					curvelocity = (curposition - lastPosition) / samplingTime;
				}
			}

			if(inCurAcc.connected())
			{
				curacceleration = inCurAcc.Get();
			} else
			{
				if (lastVelocity == lastVelocity)
				{
					curacceleration = (curvelocity - lastVelocity) / samplingTime;
				}
			}
			lastPosition = curposition;
			lastVelocity = curvelocity;

			OTGData data = calculate(inDestPos.Get(), inDestVel.Get(), curposition, curvelocity,
					curacceleration);

			outPos.Set(data.Position);
			outVel.Set(data.Velocity);
			outAcc.Set(data.Acceleration);

			lastCmdPosition = data.Position;
		}
	}

	void OTG::stopHook()
	{
	}

	void OTG::cleanupHook()
	{
	}

	double OTG::calculateDesiredAcceleration(double position, double velocity, double currentPosition,
			double currentVelocity)
	{

		double ek = (currentPosition - position) / maxAcceleration;
		double ek2 = (currentVelocity - velocity) / maxAcceleration;
		double zk = (1 / samplingTime) * ((ek / samplingTime) + (ek2 / 2));

		double m = floor((1 + sqrt(1 + 8 * fabs(zk))) / 2);

		double zk2 = ek2 / samplingTime;

		double sigma = zk2 + zk / m + ((m - 1) / 2) * sign(zk);

		double acc = (-maxAcceleration) * sat(sigma) * (1 + sign(currentVelocity * sign(sigma) + maxVelocity
				- samplingTime * maxAcceleration)) / 2;

		return acc;
	}

	OTGData OTG::calculate(double position, double velocity, double currentPosition, double currentVelocity,
			double currentAcceleration)
	{
		OTGData result;

		double resultAcc = calculateDesiredAcceleration(position, velocity, currentPosition, currentVelocity);
		double resultVel = currentVelocity + samplingTime * resultAcc;
		double resultPos = currentPosition + samplingTime * resultVel;

		result.Acceleration = resultAcc;
		result.Velocity = resultVel;
		result.Position = resultPos;

		return result;
	}

	double OTG::sat(double x)
	{
		if (x < -1)
			return -1;
		else if (x > 1)
			return 1;
		else
			return x;
	}

	int OTG::sign(double x)
	{
		if (x < 0)
			return -1;
		else if (x > 0)
			return 1;
		else
			return 0;
	}
}
