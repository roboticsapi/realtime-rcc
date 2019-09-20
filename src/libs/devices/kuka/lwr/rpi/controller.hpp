/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef CONTROLER_HPP_
#define CONTROLER_HPP_

#include "../device/IiwaDevice.hpp"

#include <rcc/Extension.hpp>
#include <rcc/DeviceFactory.hpp>
#include <rcc/DeviceInstanceT.hpp>

namespace kuka_iiwa
{

	/**
	 * This module switches the current control strategy of the LWR
	 * This requires a special KRL program running on the LWR controller
	 */
	class ControlStrategy: public RPI::ActiveModule
	{

	public:
		ControlStrategy(std::string name, RPI::Net* net);

		~ControlStrategy();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::set<std::string> getResourceNames() const;
		std::string getDeviceName();
		bool isActuator();
		void updateActuator();

	protected:
		RPI::InPort<int> inStrategy;
		RPI::OutPort<bool> outCompleted;
		RPI::OutPort<int> outError;

		RPI::Property<std::string> propRobot;

	private:
		RPI::DeviceInstanceT<IiwaDevice> devins;

	};

	class ToolParameters: public RPI::ActiveModule
		{
		public:
			ToolParameters(std::string name, RPI::Net* net);
			~ToolParameters();
			bool configureHook();
			bool startHook();
			void updateHook();
			void stopHook();
			void cleanupHook();
			std::set<std::string> getResourceNames() const;
			std::string getDeviceName();
			bool isActuator();
			void updateActuator();
		protected:
			RPI::InPort<KDL::Frame> inTCP; ///< coordinates of tool center point in flange coordinate system

			RPI::InPort<double> inMass; ///< mass of load (in kg)
			RPI::InPort<KDL::Frame> inCOM; ///< coordinates of center of mass in flange coordinate system
			RPI::InPort<KDL::Vector> inMOI; ///< moment of intertia in x, y and z direction

			RPI::OutPort<bool> outCompleted;
			RPI::OutPort<int> outError;

			RPI::Property<std::string> propRobot;

		private:
			RPI::DeviceInstanceT<LwrDevice> devins;
		};


/**
	 * This class implements a module to update cartesian amplitude and frequency
	 */
	class CartSinImpParameters: public RPI::ActiveModule
	{

		public:
			CartSinImpParameters(std::string name, RPI::Net* net);
			~CartSinImpParameters();
			bool configureHook();
			bool startHook();
			void updateHook();
			void stopHook();
			void cleanupHook();
			std::set<std::string> getResourceNames() const;
			std::string getDeviceName();
			bool isActuator();
			void updateActuator();
		protected:
			RPI::InPort<KDL::Wrench> inAmplitude;
			RPI::InPort<KDL::Wrench> inFrequency;
			RPI::Property<std::string> propRobot;
		private:
			RPI::DeviceInstanceT<IiwaDevice> devins;
			bool hasValues;
	};
}

#endif /* CONTROLER_HPP_ */
