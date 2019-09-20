/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SCHUNKWSGBLOCKS_HPP_
#define SCHUNKWSGBLOCKS_HPP_

#include <rcc/Module.hpp>
#include <rcc/DeviceFactory.hpp>
#include <rcc/DeviceInstanceT.hpp>

#include "../interface/SchunkWsgDevice.hpp"

namespace schunkwsg
{


	class Homing: public RPI::StatefulModule
	{
	public:
		Homing(std::string name, RPI::Net* net);
		virtual ~Homing();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	private:
		bool init;
		RPI::OutPort<bool> outFinished;
		RPI::OutPort<int> outStatusCode;
		RPI::Property<bool> propDirection;
		RPI::Property<std::string> propDevice;
		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
	};

	class Prepositioning: public RPI::StatefulModule
	{
	public:
		Prepositioning(std::string name, RPI::Net* net);
		virtual ~Prepositioning();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	private:
		bool init;
		RPI::OutPort<bool> outFinished;
		RPI::OutPort<int> outStatusCode;
		RPI::Property<double> propWidth;
		RPI::Property<double> propSpeed;
		RPI::Property<bool> propRelativeMotion;
		RPI::Property<bool> propStopOnBlock;
		RPI::Property<std::string> propDevice;
		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
	};

	class Grasping: public RPI::StatefulModule
	{
	public:
		Grasping(std::string name, RPI::Net* net);
		virtual ~Grasping();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	private:
		bool init;
		RPI::OutPort<bool> outFinished;
		RPI::OutPort<int> outStatusCode;
		RPI::Property<double> propWidth;
		RPI::Property<double> propSpeed;
		RPI::Property<std::string> propDevice;
		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
	};

	class Releasing: public RPI::StatefulModule
	{
	public:
		Releasing(std::string name, RPI::Net* net);
		virtual ~Releasing();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	private:
		bool init;
		RPI::OutPort<bool> outFinished;
		RPI::OutPort<int> outStatusCode;
		RPI::Property<double> propOpenWidth;
		RPI::Property<double> propSpeed;
		RPI::Property<std::string> propDevice;
		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
	};

	class SetAcceleration: public RPI::StatefulModule
	{
	public:
		SetAcceleration(std::string name, RPI::Net* net);
		virtual ~SetAcceleration();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	private:
		bool init;
		RPI::OutPort<bool> outFinished;
		RPI::OutPort<int> outStatusCode;
		RPI::Property<double> propAcceleration;
		RPI::Property<std::string> propDevice;
		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
	};

	class SetForceLimit: public RPI::StatefulModule
	{
	public:
		SetForceLimit(std::string name, RPI::Net* net);
		virtual ~SetForceLimit();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	private:
		bool init;
		RPI::OutPort<bool> outFinished;
		RPI::OutPort<int> outStatusCode;
		RPI::Property<double> propForce;
		RPI::Property<std::string> propDevice;
		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
	};

	class SetSoftLimits: public RPI::StatefulModule
	{
	public:
		SetSoftLimits(std::string name, RPI::Net* net);
		virtual ~SetSoftLimits();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	private:
		bool init;
		RPI::OutPort<bool> outFinished;
		RPI::OutPort<int> outStatusCode;
		RPI::Property<double> propInner;
		RPI::Property<double> propOuter;
		RPI::Property<std::string> propDevice;
		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
	};

	class ClearSoftLimits: public RPI::StatefulModule
	{
	public:
		ClearSoftLimits(std::string name, RPI::Net* net);
		virtual ~ClearSoftLimits();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	private:
		bool init;
		RPI::OutPort<bool> outFinished;
		RPI::OutPort<int> outStatusCode;
		RPI::Property<std::string> propDevice;
		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
	};

//	class AcknowledgeFaststopOrFault: public RPI::StatefulModule
//	{
//	public:
//		AcknowledgeFaststopOrFault(std::string name, RPI::Net* net);
//		virtual ~AcknowledgeFaststopOrFault();
//		bool configureHook();
//		bool startHook();
//		void updateHook();
//		void stopHook();
//		void cleanupHook();
//	private:
//		bool init;
//		RPI::OutPort<bool> outFinished;
//		RPI::OutPort<int> outStatusCode;
//		RPI::Property<std::string> propDevice;
//		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
//	};

	class StopDevice: public RPI::StatefulModule
	{
	public:
		StopDevice(std::string name, RPI::Net* net);
		virtual ~StopDevice();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	private:
		bool init;
		RPI::OutPort<bool> outFinished;
		RPI::Property<std::string> propDevice;
		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
	};

//	class FastStop: public RPI::StatefulModule
//	{
//	public:
//	FastStop(std::string name, RPI::Net* net);
//		virtual ~FastStop();
//		bool configureHook();
//		bool startHook();
//		void updateHook();
//		void stopHook();
//		void cleanupHook();
//	private:
//		bool init;
//		RPI::OutPort<bool> outFinished;
//		RPI::Property<std::string> propDevice;
//		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
//	};

	class Monitor: public RPI::Module
	{
	public:
		Monitor(std::string name, RPI::Net* net);
		virtual ~Monitor();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	private:
		RPI::OutPort<double> outOpeningWidth;
		RPI::OutPort<double> outSpeed;
		RPI::OutPort<double> outForce;
		RPI::OutPort<double> outGraspingState;
		RPI::Property<std::string> propDevice;
		RPI::DeviceInstanceT<schunkwsg::SchunkWsgDevice> devins;
	};

} /* namespace schunkwsg */
#endif /* SCHUNKWSGBLOCKS_HPP_ */
