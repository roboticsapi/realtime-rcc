/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ROUNDROBINLOG_HPP_
#define ROUNDROBINLOG_HPP_

#include <istream>
#include <string>
#include <rtt/os/TimeService.hpp>
#include "CrashDumper.hpp"

namespace RPI
{

	template<class T>
	class RoundRobinLog: public CrashDumper
	{
	public:

		RoundRobinLog(): len(DEFAULT_LEN), pos(0), active(true) {
			values = new T[len];
			times = new RTT::os::TimeService::nsecs[len];
			for(int i=0; i<len; i++) {
				values[i] = T(); times[i] = 0;
			}
		}

		virtual ~RoundRobinLog() {
			delete[] values;
			values = 0;
			delete[] times;
			times = 0;
		}

		void put(T value, RTT::os::TimeService::nsecs time) {
			if(!active) return;
			pos++;
			values[pos % len] = value;
			times[pos % len] = time;
		}

		void put(T value) {
			put(value, RTT::os::TimeService::Instance()->getNSecs());
		}

		void setName(std::string name) {
			this->name = name;
		}

		void dump(std::ostream& out) {
			out << name << "\n";
			int start = pos - len + 1;
			if(start < 0) start = 0;
			for(int i=start; i<pos; i++) {
				out << times[i%len] << "\t" << values[i%len]<< "\n";
			}
			out << "\n";
			pos = 0;
			active = true;
		}

		void freeze() {
			active = false;
		}
	private:
		T* values;
		RTT::os::TimeService::nsecs* times;
		std::string name;
		int pos, len;
		bool active;
		static const int DEFAULT_LEN = 1000;
	};

} /* namespace kuka_youbot */

#endif /* ROUNDROBINLOG_HPP_ */
