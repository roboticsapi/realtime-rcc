/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef GRASPINGSTATE_HPP_
#define GRASPINGSTATE_HPP_

// Windows #defines ERROR to 0, that causes confusion here
// Maybe enum should be adjusted?
#undef ERROR

namespace schunkwsg {

	enum GraspingState
	{
		IDLE = 0,
		GRASPING,
		NO_PART_FOUND,
		PART_LOST,
		HOLDING,
		RELEASING,
		POSITIONING,
		ERROR
	};

	static const char* GraspingStateNames[ERROR + 1] = {"IDLE",
			"GRASPING",
			"NO_PART_FOUND",
			"PART_LOST",
			"HOLDING",
			"RELEASING",
			"POSITIONING",
			"ERROR"};

}  /* namespace schunkwsg */
#endif /* GRASPINGSTATE_HPP_ */
