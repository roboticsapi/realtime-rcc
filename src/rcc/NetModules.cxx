/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "NetModules.hpp"

#include "Net.hpp"
#include "Registry.hpp"
#include "VirtualRCCDevice.hpp"

using namespace std;

namespace RPI
{
	Cancel::Cancel(string name,Net* net) :
		Module(name, net), outCancel("outCancel", this), cancel(false)
	{
		this->ports()->addPort(&outCancel, "true, if user requested cancel of net execution");
		this->setDescription("Block informs net of user-requested cancel; may only be used once per net");
		outCancel.Set(false);
	}

	void Cancel::setCancel(bool cancel)
	{
		this->cancel = cancel;
	}

	bool Cancel::getCancel() const
	{
		return this->cancel;
	}

	bool Cancel::configureHook()
	{
		if (inNet)
			inNet->addCancelModule(this);
		return true;
	}

	bool Cancel::startHook()
	{
		return true;
	}

	void Cancel::updateHook()
	{
		outCancel.Set(cancel);
	}

	void Cancel::stopHook()
	{
	}

	void Cancel::cleanupHook()
	{
	}

	EStop::EStop(string name, Net* net) :
			Module(name, net), inEStop("inEStop", this), isEStop(false)
	{
		this->ports()->addPort(&inEStop, "this port signals an emergency stop for all running devices");
	}

	EStop::~EStop()
	{

	}

	bool EStop::configureHook()
	{
		return true;
	}

	bool EStop::startHook()
	{
		return true;
	}

	void EStop::updateHook()
	{
		Registry::getRegistry()->getVirtualRCC()->setEStop(inEStop.GetNoNull(false));
	}

	void EStop::cleanupHook()
	{

	}

	void EStop::stopHook()
	{

	}

	// Netcomm
	Netcomm::Netcomm(std::string name, Net* net) :
			Module(name, net), propKey("Key", "Key (unique name) of the property", "")
	{
		this->properties()->addProperty(&propKey);
	}

	bool Netcomm::startHook()
	{
		return true;
	}

	void Netcomm::stopHook()
	{
	}

	Netcomm_In::Netcomm_In(std::string name, Net* net) :
			Netcomm(name, net), outLastUpdated("outLastUpdated", this)
	{
		this->ports()->addPort(&outLastUpdated);
	}

	bool Netcomm_In::configureHook()
	{
		key = "in" + this->propKey.get();

		if (inNet)
		{
			inNet->addCommunicationInProperty(key, this);
		}
		return true;
	}

	void Netcomm_In::cleanupHook()
	{
		if (inNet)
		{
			inNet->removeCommunicationInProperty(key);
		}
	}

	Netcomm_Out::Netcomm_Out(std::string name, Net* net) :
			Netcomm(name, net), propReport("Report", "Report value to application (set to false for pure InterNetcomm)", true)
	{
		this->properties()->addProperty(&propReport);
		initialized = false;
	}

	bool Netcomm_Out::configureHook()
	{
		key = "out" + this->propKey.get();

		if (inNet)
		{
			inNet->addCommunicationOutProperty(key, this, propReport.get());
		}
		return true;
	}

	void Netcomm_Out::cleanupHook()
	{
		if (inNet)
		{
			inNet->removeCommunicationOutProperty(key);
		}
	}

	bool Netcomm_Out::isInitialized()
	{
		return initialized;
	}

	// InterNetcomm
	InterNetcomm::InterNetcomm(string name, Net* net) :
			Module(name, net), propRemoteRCC("RemoteRCC", "Name of the remote RCC the communicate with"), propRemoteNet(
					"RemoteNet", "Name of the remote net"), propRemoteKey("RemoteKey",
					"Key (unique name) of the remote property", "")
	{
		this->properties()->addProperty(&propRemoteRCC);
		this->properties()->addProperty(&propRemoteNet);
		this->properties()->addProperty(&propRemoteKey);

	}

	bool InterNetcomm::startHook()
	{
		return true;
	}

	void InterNetcomm::stopHook()
	{
	}

	void InterNetcomm::cleanupHook()
	{
	}

	InterNetcomm_In::InterNetcomm_In(std::string name, Net* net) :
			InterNetcomm(name, net), outLastUpdated("outLastUpdated", this)
	{
		this->ports()->addPort(&outLastUpdated);
	}

	bool InterNetcomm_In::configureHook()
	{
		std::string remotekey = "out" + propRemoteKey.get();
		std::string remotenet = propRemoteNet.get();

		auto net = Registry::getRegistry()->getNet(remotenet);
		if (!net)
			return false;

		dataptr = net->getCommunicationData(remotekey);

		// If no pointer is returned no Netcomm primitive has been found
		if (!dataptr)
			return false;

		return true;
	}

	InterNetcomm_Out::InterNetcomm_Out(std::string name, Net* net) :
			InterNetcomm(name, net)
	{

	}

	bool InterNetcomm_Out::configureHook()
	{
		std::string remotekey = "in" + propRemoteKey.get();
		std::string remotenet = propRemoteNet.get();

		auto net = Registry::getRegistry()->getNet(remotenet);
		if (!net)
			return false;

		dataptr = net->getCommunicationData(remotekey);

		// If no pointer is returned no Netcomm primitive has been found
		if (!dataptr)
			return false;

		return true;
	}

}
