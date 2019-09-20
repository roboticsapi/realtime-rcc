/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CRASHDUMP_HPP_
#define CRASHDUMP_HPP_

#include "CrashDumpFwd.hpp"

#include "DeviceFwd.hpp"
#include "NetModulesFwd.hpp"
#include "CrashDumperFwd.hpp"
#include <rtt/os/TimeService.hpp>

namespace RPI
{

	class CrashDump
	{
	private:
		CrashDump();
		virtual ~CrashDump();

	public:
		static void writeCrashDump(std::string name, std::list<RPI::CrashDumper*>);
		static void writeCrashDump(std::string name, long netCycle, RTT::os::TimeService::nsecs endTime, RTT::os::TimeService::nsecs cylceTime, std::vector<RPI::DebugModule*>);
	};

} /* namespace RPI */

#endif /* CRASHDUMP_HPP_ */
