/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "NetExecutorLookup.hpp"
#include "Net.hpp"
#include "NetExecutor.hpp"
#include "Registry.hpp"
#include <algorithm>
#include <rtt/Logger.hpp>

namespace RPI
{
	using namespace std;

	NetExecutorLookup::NetExecutorLookup()
	{

	}

	NetExecutorLookup::~NetExecutorLookup()
	{

	}

	NetExecutor* NetExecutorLookup::getNetExecutor(Net* net)
	{
		RTT::os::MutexLock mutex(listmutex);

		// check if net already exists in table
		auto existing = find_if(table.begin(), table.end(), containsNet(net));
		if (existing != table.end()) {
			return existing->executor;
		}

		vector<string> resources = sortResources(net->getResourceNames());

		int num_net_resources = resources.size();

		int max_intersect_size = 0;

		auto found_it = table.end();
		vector<string> new_intersect;

		// Only search for existing executor if net requires at least a single resource
		if (num_net_resources > 0)
		{
			// search for best matching existing executor
			for (auto it = table.begin(); it != table.end(); ++it)
			{
				const auto& te = *it;
				vector<string> intersect(num_net_resources);
				auto intersect_it = set_intersection(te.resources.begin(), te.resources.end(), resources.begin(),
						resources.end(), intersect.begin());

				unsigned int new_size = intersect_it - intersect.begin();

				// look whether potential candidate has been found
				if ((new_size > 0) && (new_size > max_intersect_size))
				{
					intersect.resize(new_size);
					found_it = it;
					max_intersect_size = new_size;
					new_intersect = intersect;
				}

			}
		}

		// Did we find a suitable executor?
		if (found_it != table.end())
		{
			NetExecutorTable& te = *found_it;
			RTT::log(RTT::LoggerLevel::Info) << "Reusing executor " << te.executor->getName() << " for " << net->getName() << RTT::endlog();

			te.resources = new_intersect;
			te.nets.insert(net);

			return te.executor;
		} else
		{
			RTT::log(RTT::LoggerLevel::Info) << "Creating new executor for " << net->getName() << "." << RTT::endlog();

			NetExecutor* ne = new NetExecutor(net->getName(), net->getNetFrequency(), net->isRealtimeNet());

			ne->start();

			NetExecutorTable te;
			te.resources = resources;
			te.executor = ne;
			te.nets.insert(net);

			table.push_back(te);

			Registry::getRegistry()->addNetExecutor(ne);


			return ne;
		}

	}

	void NetExecutorLookup::releaseNetExecutor(Net* net)
	{
		RTT::os::MutexLock mutex(listmutex);

		auto found_it = find_if(table.begin(), table.end(), containsNet(net));
		if (found_it != table.end())
		{
			// Not the last net of the executor
			if (found_it->nets.size() > 1)
			{
				// remove net
				found_it->nets.erase(net);

				// create new intersection
				auto intersect_it = found_it->nets.begin();
				vector<string> intersect = sortResources((*intersect_it)->getResourceNames());
				intersect_it++;

				while (intersect_it != found_it->nets.end())
				{
					vector<string> new_intersect(intersect.size());
					vector<string> next_intersect = sortResources((*intersect_it)->getResourceNames());
					auto res_it = set_intersection(intersect.begin(), intersect.end(), next_intersect.begin(),
							next_intersect.end(), new_intersect.begin());
					new_intersect.resize(res_it - new_intersect.begin());
					intersect = new_intersect;
					intersect_it++;
				}

				found_it->resources = intersect;

			}
			// Last net of executor
			else
			{
//				found_it->executor->stop();
//				found_it->executor->cleanup();
//
//				RTT::log() << "Deleting Executor " << found_it->executor << " " << found_it->executor->engine() << RTT::endlog();
//
//				delete found_it->executor;

				found_it->executor->finish();

				// Remove executor entry and break for loop
				// one net may only be contained in single
				table.erase(found_it);

			}
		}

	}

	NetExecutorLookup::containsNet::containsNet(Net* net)
	{
		compare_net = net;
	}

	bool NetExecutorLookup::containsNet::operator ()(const NetExecutorTable& te)
	{
		return te.nets.find(compare_net) != te.nets.end();
	}

} /* namespace RPI */
