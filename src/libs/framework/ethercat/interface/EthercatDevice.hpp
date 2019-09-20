/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ETHERCATDEVICE_HPP_
#define ETHERCATDEVICE_HPP_

namespace ethercat
{

	class EthercatDevice
	{
	public:
		virtual ~EthercatDevice() {};

		virtual void startupDevice() = 0;
		virtual void updateDevice() = 0;
		virtual void shutdownDevice() = 0;
	};

} /* namespace RPI */

#endif /* ETHERCATDEVICE_HPP_ */
