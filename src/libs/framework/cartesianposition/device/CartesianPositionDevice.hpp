/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CARTESIANPOSITIONDEVICE_HPP_
#define CARTESIANPOSITIONDEVICE_HPP_

#include "../interface/CartesianPositionInterface.hpp"
#include <rcc/RoundRobinLog.hpp>
#include <list>

namespace cartesianposition
{

	class RTT_EXPORT CartesianPositionDevice: public virtual cartesianposition::CartesianPositionInterface
	{
	public:
		CartesianPositionDevice();
		virtual ~CartesianPositionDevice();

		virtual void setPosition(KDL::Frame position, RTT::os::TimeService::nsecs time);
		virtual KDL::Frame getCommandedPosition();
		virtual KDL::Twist getCommandedVelocity();

	protected:

		virtual void initPosition(KDL::Frame position, KDL::Twist velocity);

		KDL::Twist cmdVel;
		KDL::Frame cmdPos;
		RTT::os::TimeService::nsecs cmdTime;

		virtual std::list<RPI::CrashDumper*> getCrashDumpers() const;

	private:
		RPI::RoundRobinLog<double>* dumpRPIPos;

	};

} /* namespace cartesianposition */

#endif /* CARTESIANPOSITIONDEVICE_HPP_ */
