/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef COEETHERCATSLAVE_HPP_
#define COEETHERCATSLAVE_HPP_

#include "EthercatSlave.hpp"

namespace ethercat
{

	class CoEEthercatSlave: public EthercatSlave
	{
	public:
		CoEEthercatSlave();
		virtual ~CoEEthercatSlave();

		virtual void writeMailboxMessage();
		virtual void readMailboxMessage();
	};

} /* namespace ethercat */
#endif /* COEETHERCATSLAVE_HPP_ */
