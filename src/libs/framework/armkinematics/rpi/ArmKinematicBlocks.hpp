/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ARMKINEMATICBLOCKS_HPP_
#define ARMKINEMATICBLOCKS_HPP_

#include "../interface/ArmKinematicsInterface.hpp"

#include <rcc/DeviceFactory.hpp>
#include <rcc/Module.hpp>
#include <templates/TArray.hpp>
#include <kdl/frames.hpp>
#include <rcc/DeviceInstanceT.hpp>

namespace armkinematics
{

	class InvKin: public RPI::ActiveModule
	{
	public:
		InvKin(std::string name, RPI::Net* net);
		virtual ~InvKin();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

	protected:
		RPI::InPort<KDL::Frame> inFrame;
		RPI::InPort<RPI::Array<double> > inHintJoints;
		RPI::OutPort<RPI::Array<double> > outJoints;
		RPI::Property<std::string> propRobot;

	private:
		RPI::DeviceInstanceT<ArmKinematicsInterface> devins;

		RPI::Array<double> joints;
	};

	class Kin: public RPI::ActiveModule
	{
	public:
		Kin(std::string name, RPI::Net* net);

		virtual ~Kin();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

	protected:
		RPI::InPort<RPI::Array<double> > inJoints;
		RPI::OutPort<KDL::Frame> outFrame;
		RPI::Property<std::string> propRobot;

	private:
		RPI::DeviceInstanceT<ArmKinematicsInterface> devins;

	};

} /* namespace armkinematics */
#endif /* ARMKINEMATICBLOCKS_HPP_ */
