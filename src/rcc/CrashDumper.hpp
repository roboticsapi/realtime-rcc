/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CRASHDUMPER_HPP_
#define CRASHDUMPER_HPP_

#include "CrashDumperFwd.hpp"
#include <istream>

namespace RPI {

	class CrashDumper
	{
	protected:
		CrashDumper() {}
		virtual ~CrashDumper() {}

	public:
		virtual void freeze() = 0;
		virtual void dump(std::ostream& out) = 0;
	};

}



#endif /* CRASHDUMPER_HPP_ */
