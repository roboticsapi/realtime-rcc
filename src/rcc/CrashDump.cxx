/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "CrashDump.hpp"
#include <time.h>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "Module.hpp"
#include "NetModules.hpp"
#include "CrashDumper.hpp"

namespace RPI
{

	CrashDump::CrashDump()
	{
	}

	CrashDump::~CrashDump()
	{
	}

	void CrashDump::writeCrashDump(std::string name, long netCycle, RTT::os::TimeService::nsecs endTime, RTT::os::TimeService::nsecs cycleTime, std::vector<RPI::DebugModule*> modules)
	{
		if(modules.size() == 0)
			return;

		time_t timer;
		struct tm* timeinfo;
		time(&timer);
		timeinfo = localtime(&timer);
		std::stringstream fn;
		fn << "crash-" <<
			std::setfill('0')<<std::setw(4) << (timeinfo->tm_year + 1900) << "-" <<
			std::setfill('0')<<std::setw(2) << (timeinfo->tm_mon + 1) << "-" <<
			std::setfill('0')<<std::setw(2) << timeinfo->tm_mday << "-" <<
			std::setfill('0')<<std::setw(2) << timeinfo->tm_hour << "-" <<
			std::setfill('0')<<std::setw(2) << timeinfo->tm_min << "-" <<
			std::setfill('0')<<std::setw(2) << timeinfo->tm_sec << "-" <<
			name << ".log";

		std::ofstream log;
		log.open("web/crash/"+fn.str());

		long end = netCycle;

		long start = end;
		for(const auto& module: modules) {
			log << module->getName() << "\n";
			int start = end - module->getCount();
			if(start < 0) start = 0;
			for(int i=start; i<end; i++) {
				std::stringstream value;
				module->output(value, i);
				log << (endTime - cycleTime * (end - i)) << "\t" << value.str() << "\n";
			}
			log << "\n";
		}
		log.close();

	}

	void CrashDump::writeCrashDump(std::string name, std::list<RPI::CrashDumper*> dumpers)
	{
		if(dumpers.size() == 0)
			return;
		time_t timer;
		struct tm* timeinfo;
		time(&timer);
		timeinfo = localtime(&timer);
		std::stringstream fn;
		fn << "crash-" <<
			std::setfill('0')<<std::setw(4) << (timeinfo->tm_year + 1900) << "-" <<
			std::setfill('0')<<std::setw(2) << (timeinfo->tm_mon + 1) << "-" <<
			std::setfill('0')<<std::setw(2) << timeinfo->tm_mday << "-" <<
			std::setfill('0')<<std::setw(2) << timeinfo->tm_hour << "-" <<
			std::setfill('0')<<std::setw(2) << timeinfo->tm_min << "-" <<
			std::setfill('0')<<std::setw(2) << timeinfo->tm_sec << "-" <<
			name << ".log";
		std::ofstream log;
		log.open("web/crash/"+fn.str());
		for(const auto& dumper: dumpers)
		{
			dumper->dump(log);
		}
		log.close();
	}

} /* namespace RPI */
