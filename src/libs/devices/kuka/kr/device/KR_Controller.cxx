/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "KR_Controller.hpp"

namespace kuka_kr
{
	/**
	 * Constructor and destructor should never be really called, as KR_Controller is inherited
	 * virtually in all subclasses.
	 */


	KR_Controller::KR_Controller() :
			RPI::Device("KR_Controller_dummy", RPI::parameter_t())
	{
		// TODO Auto-generated constructor stub

	}

	KR_Controller::~KR_Controller()
	{
		// TODO Auto-generated destructor stub
	}

} /* namespace kuka_kr */
