/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef NETCONDITIONFWD_HPP_
#define NETCONDITIONFWD_HPP_

#include <string>

namespace RPI
{
	enum class SynchronizationRuleState
	{
		Active,
		Fired,
		FireFailed,
		Inactive,
		InvalidRule
	};

	static const std::string SynchronizationRuleStateString[] =
	{ "ACTIVE", "FIRED", "FIREFAILED", "INACTIVE", "INVALIDRULE" };

	typedef unsigned long Sync_ID_t;

	class SynchronizationRule;
}

#endif /* NETCONDITIONFWD_HPP_ */
