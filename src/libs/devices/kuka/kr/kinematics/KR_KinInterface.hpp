/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef SRC_LIBS_KUKA_KR_KINEMATICS_KR_KININTERFACE_HPP_
#define SRC_LIBS_KUKA_KR_KINEMATICS_KR_KININTERFACE_HPP_

#include <kdl/frames.hpp>
#include <templates/TArray.hpp>

namespace kuka_kr
{
	class RTT_EXPORT KR_Kin_Interface
	{
	public:
		virtual ~KR_Kin_Interface()
		{
		}

		virtual KDL::Frame Kin(const RPI::Array<double>& joints) = 0;
		virtual void InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position,
				RPI::Array<double>& values) = 0;
	};
}

#endif /* SRC_LIBS_KUKA_KR_KINEMATICS_KR_KININTERFACE_HPP_ */
