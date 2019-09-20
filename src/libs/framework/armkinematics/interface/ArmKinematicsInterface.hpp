/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ARMKINEMATICSINTERFACE_HPP_
#define ARMKINEMATICSINTERFACE_HPP_

#include <templates/TArray.hpp>
#include <kdl/frames.hpp>

namespace armkinematics
{

	class RTT_EXPORT ArmKinematicsInterface
	{
	public:
		ArmKinematicsInterface() { }
		virtual ~ArmKinematicsInterface() { }

		virtual KDL::Frame Kin(const RPI::Array<double>& joints) = 0;
		virtual void InvKin(const RPI::Array<double>& hintjoints, const KDL::Frame& position, RPI::Array<double>& resultJoints) = 0;
		virtual int getJointCount() const = 0;
	};

} /* namespace armkinematics */
#endif /* ARMKINEMATICSINTERFACE_HPP_ */
