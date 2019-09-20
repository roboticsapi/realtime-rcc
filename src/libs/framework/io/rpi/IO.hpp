/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef IO_HPP_
#define IO_HPP_

#include "../interface/IOInterface.hpp"

#include <rcc/Module.hpp>
#include <rcc/DeviceFactory.hpp>
#include <rcc/Extension.hpp>
#include <rcc/DeviceInstanceT.hpp>

#include <string>
#include <rtt/Logger.hpp>

namespace IO
{
	/**
	 * Digital output port
	 */
	class IOOutBool: public RPI::ActiveModule
	{

	public:
		IOOutBool(std::string name, RPI::Net* net);

		virtual ~IOOutBool();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isActuator();
		void updateActuator();

	protected:
		RPI::InPort<bool> inIO;
		RPI::Property<bool> propIO;
		RPI::Property<std::string> propDeviceID;
		RPI::Property<int> propPort;
	private:
		RPI::DeviceInstanceT<IOInterface> devins;
		bool hasValues;
	};

	/**
	 * Digital output sensor
	 */
	class IOOutBoolSensor: public RPI::Module
	{

	public:
		IOOutBoolSensor(std::string name, RPI::Net* net);
		virtual ~IOOutBoolSensor();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isSensor();
		void updateSensor();

	protected:
		RPI::OutPort<bool> outIO;
		RPI::Property<std::string> propDeviceID;
		RPI::Property<int> propPort;
	private:
		RPI::DeviceInstanceT<IOInterface> devins;
	};

	/**
	 * Digital input
	 */
	class IOInBool: public RPI::Module
	{
	public:
		IOInBool(std::string name, RPI::Net* net);
		virtual ~IOInBool();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isSensor();
		void updateSensor();

	protected:
		RPI::OutPort<bool> outIO;
		RPI::Property<std::string> propDeviceID;
		RPI::Property<int> propPort;
	private:
		RPI::DeviceInstanceT<IOInterface> devins;
	};

	/**
	 * Analog output
	 */
	class IOOutReal: public RPI::ActiveModule
	{

	public:
		IOOutReal(std::string name, RPI::Net* net);

		virtual ~IOOutReal();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isActuator();
		void updateActuator();

	protected:
		RPI::InPort<double> inIO;
		RPI::Property<double> propIO;
		RPI::Property<std::string> propDeviceID;
		RPI::Property<int> propPort;
	private:
		RPI::DeviceInstanceT<IOInterface> devins;
		bool hasValues;
	};

	/**
	 * Analog output sensor
	 */
	class IOOutRealSensor: public RPI::Module
	{

	public:
		IOOutRealSensor(std::string name, RPI::Net* net);
		virtual ~IOOutRealSensor();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isSensor();
		void updateSensor();

	protected:
		RPI::OutPort<double> outIO;
		RPI::Property<std::string> propDeviceID;
		RPI::Property<int> propPort;
	private:
		RPI::DeviceInstanceT<IOInterface> devins;
	};

	/**
	 * Analog input
	 */
	class IOInReal: public RPI::Module
	{

	public:
		IOInReal(std::string name, RPI::Net* net);
		virtual ~IOInReal();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isSensor();
		void updateSensor();

	protected:
		RPI::OutPort<double> outIO;
		RPI::Property<std::string> propDeviceID;
		RPI::Property<int> propPort;
	private:
		RPI::DeviceInstanceT<IOInterface> devins;
	};

}

#endif /* IO_HPP_ */
