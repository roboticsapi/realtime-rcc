/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "NetExecutor.hpp"

#include <rtt/Activity.hpp>
#include "Net.hpp"
#include "Registry.hpp"

namespace RPI
{
	NetExecutor::NetExecutor(const std::string& name, double frequency, bool isRealtime) :
			RTT::TaskContext(name, PreOperational), currentNet(0), nextNet(0), finished(false), finishRequested(false),
			time(0), cycleFrequency(frequency), isRealtimeNet(isRealtime)
	{
		// we are just below maximum priority, which is reserved for tasks which may *never* fail,
		// such as FRI

		int priority = RTT::os::HighestPriority - RTT::os::IncreasePriority;

		if (!isRealtime)
			priority -= RTT::os::IncreasePriority;

		cpu = Registry::getRegistry()->getNextCPU();
		unsigned int cpumask = 1 << cpu;

		this->setActivity(new RTT::Activity(ORO_SCHED_RT, priority, frequency, cpumask, 0, "NetExecutor" + name));
	}

	NetExecutor::~NetExecutor()
	{
		Registry::getRegistry()->releaseCPU(cpu);
	}

	bool NetExecutor::configureHook()
	{
		return true;
	}

	bool NetExecutor::startHook()
	{
		time = RTT::os::TimeService::Instance()->getNSecs();

		return true;
	}

	void NetExecutor::updateHook()
	{
		//rt_task_set_mode(0, T_WARNSW, NULL);
		// don't do anything if we are supposed to stop
		if (!finished)
		{
			// there is a net to execute
			if (currentNet)
			{
				currentNet->update();
			}
			time += (getFrequency() * 1e9);

			if (finishRequested)
			{
				// We should terminate
				finished = true;
				stop();
			} else
			{
				// update net pointer for next cycle
				currentNet = nextNet;
			}
		}
	}

	RTT::os::TimeService::nsecs NetExecutor::getTime() const
	{
		return time;
	}

	void NetExecutor::stopHook()
	{

	}

	void NetExecutor::cleanupHook()
	{

	}

	void NetExecutor::setCurrentNet(Net* net)
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
