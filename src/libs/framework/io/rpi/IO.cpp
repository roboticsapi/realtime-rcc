/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "IO.hpp"

namespace IO
{
	using namespace RPI;
	using namespace std;

	IOOutBool::IOOutBool(string name, RPI::Net* net) :
			ActiveModule(name, net), inIO("inIO", this), propIO("IO",
					"Value to set to output when inIO is not connected", false), propDeviceID("DeviceID",
					"Device name IO is connected to", ""), propPort("Port", "Port to be used", 0), devins(), hasValues(
					false)
	{
		setDescription("Digital output");
		this->ports()->addPort(&inIO, "Value to set on output");
		this->properties()->addProperty(&propDeviceID);
		this->properties()->addProperty(&propPort);
		this->properties()->addProperty(&propIO);
	}

	IOOutBool::~IOOutBool()
	{
	}

	bool IOOutBool::configureHook()
	{
		if (!devins.fetchInstance(propDeviceID.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propDeviceID.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool IOOutBool::startHook()
	{
		return true;
	}

	void IOOutBool::updateHook()
	{
		if (active())
		{
			hasValues = true;
		}
	}

	void IOOutBool::stopHook()
	{
	}

	void IOOutBool::cleanupHook()
	{
	}

	string IOOutBool::getDeviceName()
	{
		return propDeviceID.get();
	}

	bool IOOutBool::isActuator()
	{
		return true;
	}

	void IOOutBool::updateActuator()
	{
		if (hasValues)
		{
			bool value = inIO.Get(propIO);
			devins.getDevice()->setDigitalOut(propPort.get(), value);
			hasValues = false;
		}
	}

	IOOutBoolSensor::IOOutBoolSensor(string name, RPI::Net* net) :
			Module(name, net), outIO("outIO", this), propDeviceID("DeviceID", "Device name IO is connected to", ""), propPort(
					"Port", "Port to be used", 0), devins()
	{
		setDescription("Digital output sensor");
		this->ports()->addPort(&outIO, "value currently set on output");
		this->properties()->addProperty(&propDeviceID);
		this->properties()->addProperty(&propPort);
	}

	IOOutBoolSensor::~IOOutBoolSensor()
	{
	}

	bool IOOutBoolSensor::configureHook()
	{
		if (!devins.fetchInstance(propDeviceID.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propDeviceID.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool IOOutBoolSensor::startHook()
	{
		return true;
	}

	void IOOutBoolSensor::updateHook()
	{

	}

	void IOOutBoolSensor::stopHook()
	{
	}

	void IOOutBoolSensor::cleanupHook()
	{
	}

	string IOOutBoolSensor::getDeviceName()
	{
		return propDeviceID.get();
	}

	bool IOOutBoolSensor::isSensor()
	{
		return true;
	}

	void IOOutBoolSensor::updateSensor()
	{
		outIO.Set(devins.getDevice()->getDigitalOut(propPort.get()));
	}

	IOInBool::IOInBool(string name, RPI::Net* net) :
			Module(name, net), outIO("outIO", this), propDeviceID("DeviceID", "Device name IO is connected to", ""), propPort(
					"Port", "Port to be used", 0), devins()
	{
		setDescription("Digital input");
		this->ports()->addPort(&outIO, "value read from input port");
		this->properties()->addProperty(&propDeviceID);
		this->properties()->addProperty(&propPort);
	}

	IOInBool::~IOInBool()
	{
	}

	bool IOInBool::configureHook()
	{
		if (!devins.fetchInstance(propDeviceID.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propDeviceID.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool IOInBool::startHook()
	{
		return true;
	}

	void IOInBool::updateHook()
	{

	}

	void IOInBool::stopHook()
	{
	}

	void IOInBool::cleanupHook()
	{
	}

	string IOInBool::getDeviceName()
	{
		return propDeviceID.get();
	}

	bool IOInBool::isSensor()
	{
		return true;
	}

	void IOInBool::updateSensor()
	{
		outIO.Set(devins.getDevice()->getDigitalIn(propPort.get()));
	}

	/**
	 * Analog output
	 */

	IOOutReal::IOOutReal(string name, RPI::Net* net) :
			ActiveModule(name, net), inIO("inIO", this), propIO("IO",
					"Value to set to output when inIO is not connected", 0), propDeviceID("DeviceID",
					"Device name IO is connected to", ""), propPort("Port", "Port to be used", 0), devins(), hasValues(
					false)
	{
		setDescription("Analog output");
		this->ports()->addPort(&inIO, "Value to set on output");
		this->properties()->addProperty(&propDeviceID);
		this->properties()->addProperty(&propPort);
		this->properties()->addProperty(&propIO);
	}

	IOOutReal::~IOOutReal()
	{
	}

	bool IOOutReal::configureHook()
	{
		if (!devins.fetchInstance(propDeviceID.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propDeviceID.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool IOOutReal::startHook()
	{
		return true;
	}

	void IOOutReal::updateHook()
	{
		if (active())
		{
			hasValues = true;
		}
	}

	void IOOutReal::stopHook()
	{
	}

	void IOOutReal::cleanupHook()
	{
	}

	string IOOutReal::getDeviceName()
	{
		return propDeviceID.get();
	}

	bool IOOutReal::isActuator()
	{
		return true;
	}

	void IOOutReal::updateActuator()
	{
		if (hasValues)
		{
			double value = inIO.Get(propIO);
			devins.getDevice()->setAnalogOut(propPort.get(), value);
			hasValues = false;
		}
	}

	IOOutRealSensor::IOOutRealSensor(string name, RPI::Net* net) :
			Module(name, net), outIO("outIO", this), propDeviceID("DeviceID", "Device name IO is connected to", ""), propPort(
					"Port", "Port to be used", 0), devins()
	{
		setDescription("Analog output sensor");
		this->ports()->addPort(&outIO, "value currently set on output");
		this->properties()->addProperty(&propDeviceID);
		this->properties()->addProperty(&propPort);
	}

	IOOutRealSensor::~IOOutRealSensor()
	{
	}

	bool IOOutRealSensor::configureHook()
	{
		if (!devins.fetchInstance(propDeviceID.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propDeviceID.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool IOOutRealSensor::startHook()
	{
		return true;
	}

	void IOOutRealSensor::updateHook()
	{
	}

	void IOOutRealSensor::stopHook()
	{
	}

	void IOOutRealSensor::cleanupHook()
	{
	}

	string IOOutRealSensor::getDeviceName()
	{
		return propDeviceID.get();
	}

	bool IOOutRealSensor::isSensor()
	{
		return true;
	}

	void IOOutRealSensor::updateSensor()
	{
		outIO.Set(devins.getDevice()->getAnalogOut(propPort.get()));
	}

	IOInReal::IOInReal(string name, RPI::Net* net) :
			Module(name, net), outIO("outIO", this), propDeviceID("DeviceID", "Device name IO is connected to", ""), propPort(
					"Port", "Port to be used", 0), devins()
	{
		setDescription("Analog input");
		this->ports()->addPort(&outIO, "value read from input port");
		this->properties()->addProperty(&propDeviceID);
		this->properties()->addProperty(&propPort);
	}

	IOInReal::~IOInReal()
	{
	}

	bool IOInReal::configureHook()
	{
		if (!devins.fetchInstance(propDeviceID.get()))
		{
			RTT::log(RTT::Error) << "LBR " << propDeviceID.get() << " not available." << RTT::endlog();
			return false;
		}
		return true;
	}

	bool IOInReal::startHook()
	{
		return true;
	}

	void IOInReal::updateHook()
	{

	}

	void IOInReal::stopHook()
	{
	}

	void IOInReal::cleanupHook()
	{
	}

	string IOInReal::getDeviceName()
	{
		return propDeviceID.get();
	}

	bool IOInReal::isSensor()
	{
		return true;
	}

	void IOInReal::updateSensor()
	{
		outIO.Set(devins.getDevice()->getAnalogIn(propPort.get()));
	}

}
