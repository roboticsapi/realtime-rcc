/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SRC_RCC_NETEXECUTORLOOKUP_HPP_
#define SRC_RCC_NETEXECUTORLOOKUP_HPP_

#include "NetExecutorLookupFwd.hpp"
#include "NetExecutorFwd.hpp"
#include "NetFwd.hpp"

#include <memory>
#include <list>
#include <set>
#include <vector>
#include <string>
#include <algorithm>
#include <rtt/os/Mutex.hpp>

namespace RPI
{
	class NetExecutorLookup
	{
	private:
		// internal structures
		struct NetExecutorTable
		{
			NetExecutor* executor;
			// sorted set of resources
			std::vector<std::string> resources;
			std::set<Net*> nets;
		};

		class containsNet
		{
		public:
			containsNet(Net*);
			bool operator()(const NetExecutorTable&);
		private:
			Net* compare_net;
		};
	public:
		NetExecutorLookup();
		virtual ~NetExecutorLookup();

		NetExecutor* getNetExecutor(Net* net);
		void releaseNetExecutor(Net*);

	private:
		std::list<NetExecutorTable> table;
		template<class T> std::vector<std::string> sortResources(const T& res_set) const
		{
			std::vector<std::string> result(res_set.begin(), res_set.end());
			std::sort(result.begin(), result.end());
			return result;

		}
		RTT::os::Mutex listmutex;
	};

} /* namespace RPI */

#endif /* SRC_RCC_NETEXECUTORLOOKUP_HPP_ */
