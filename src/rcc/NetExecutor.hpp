/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef NETEXECUTOR_HPP_
#define NETEXECUTOR_HPP_

#include "NetExecutorFwd.hpp"
#include "NetFwd.hpp"

#include <rtt/os/Thread.hpp>

namespace RPI
{
	class NetExecutor: public RTT::os::Thread
	{
	public:
		NetExecutor(const std::string& name, double frequency, bool isRealtime);
		virtual ~NetExecutor();

		void step();
		bool initialize();
		void finalize();

		void finish();
		bool isFinished() const;

		void setNextNet(Net* net);

		RTT::os::TimeService::nsecs getTime() const;
		double getFrequency() const;
		bool isRealtime() const;
	private:
		Net *currentNet, *nextNet;
		bool finished, finishRequested;
		RTT::os::TimeService::nsecs time;

		double cycleFrequency;
		bool isRealtimeNet;
		int priority;
		unsigned int cpu;
	};
}
#endif /* NETEXECUTOR_HPP_ */
