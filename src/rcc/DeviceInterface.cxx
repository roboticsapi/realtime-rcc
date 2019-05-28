/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "DeviceInterface.hpp"

namespace RPI
{

	DeviceInterface::DeviceInterface(const std::string& name, const HTTPHandlerList& handlers)
	{
		this->name = name;
		this->httpHandlers = handlers;

	}

	DeviceInterface::~DeviceInterface()
	{
		// TODO Auto-generated destructor stub
	}

	HTTPHandlerList DeviceInterface::getHTTPHandlers()	const
	{
		return httpHandlers;
	}

} /* namespace RPI */
