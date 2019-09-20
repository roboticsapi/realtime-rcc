/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "NetExecutor.hpp"

#include <rtt/Activity.hpp>
#include <rtt/Logger.hpp>
#include "Net.hpp"
#include "Registry.hpp"

namespace RPI
{
	// we are just below maximum priority, which is reserved for tasks which may *never* fail,
	// such as FRI
	NetExecutor::NetExecutor(const std::string& name, double frequency, bool isRealtime) :
			RTT::os::Thread(ORO_SCHED_RT, 
				priority = (isRealtime ? RTT::os::HighestPriority - RTT::os::IncreasePriority : RTT::os::HighestPriority - 2 * RTT::os::IncreasePriority), 
				cycleFrequency = frequency, 
				1 << (cpu = Registry::getRegistry()->getNextCPU()), 
				name + "exe"),
			currentNet(0), nextNet(0), finished(false), finishRequested(false), time(0),
			isRealtimeNet(isRealtime)
	{
	}

	NetExecutor::~NetExecutor()
	{
	}

	bool NetExecutor::initialize()
	{
		RTT::Logger::In in(getName());
		RTT::log(RTT::Debug) << "Starting executor " << getName() << RTT::endlog();

		time = RTT::os::TimeService::Instance()->getNSecs();
		return true;
	}

	void NetExecutor::step()
	{
		//rt_task_set_mode(0, T_WARNSW, NULL);
		// don't do anything if we are supposed to stop
		if (!finished)
		{
			// there is a net to execute
			if (currentNet)
			{
				bool doexecute = true;
				while (doexecute)
				{
					bool normalrun = currentNet->update();
					doexecute = !normalrun && nextNet;
					if (doexecute)
					{
						currentNet = nextNet;
						nextNet = 0;
					}
				}
			}
			time += (getFrequency() * 1e9);

			if (finishRequested)
			{
				// We should terminate
				finished = true;
				stop();
			} else if (nextNet)
			{
				// update net pointer for next cycle
				currentNet = nextNet;
				nextNet = 0;
			}
		}
	}

	RTT::os::TimeService::nsecs NetExecutor::getTime() const
	{
		return time;
	}

	void NetExecutor::finalize()
	{

		RTT::Logger::In in(getName());

		Registry::getRegistry()->releaseCPU(cpu);

		RTT::log(RTT::Debug) << "Stopping executor " << getName() << RTT::endlog();
	}

	void NetExecutor::setNextNet(Net* net)
	{
		this->nextNet = net;
	}

	void NetExecutor::finish()
	{
		this->finishRequested = true;
	}

	bool NetExecutor::isFinished() const
	{
		return this->finished;
	}

	double NetExecutor::getFrequency() const
	{
		return this->cycleFrequency;
	}

	bool NetExecutor::isRealtime() const
	{
		return this->isRealtimeNet;
	}
}
