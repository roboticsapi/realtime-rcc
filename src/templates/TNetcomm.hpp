/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef TNETCOMM_HPP_
#define TNETCOMM_HPP_

#include "../rcc/Net.hpp"

#include "../rcc/NetModules.hpp"
#include "TArray.hpp"
#include <string.h>
#include <rtt/os/MutexLock.hpp>

namespace RPI
{

	template<class T> class TNetcommData: public NetcommData
	{
	public:
		TNetcommData(std::shared_ptr<RTT::os::Mutex> mutex, T defaultValue = T())
		{
			netCommMutex = mutex;
			lastUpdate = 0;
			data = defaultValue;
		}

		std::pair<T, RTT::os::TimeService::nsecs> getData() const
		{
			RTT::os::MutexLock(*this->netCommMutex);
			return std::make_pair(data, lastUpdate);
		}

		void setData(const T& data)
		{
			RTT::os::MutexLock(*this->netCommMutex);
			this->data = data;
			this->lastUpdate = RTT::os::TimeService::Instance()->getNSecs();
		}

		void updateDataTime()
		{
			this->lastUpdate = RTT::os::TimeService::Instance()->getNSecs();
		}
	private:
		T data;
		RTT::os::TimeService::nsecs lastUpdate;
		std::shared_ptr<RTT::os::Mutex> netCommMutex;
	};

	template<class T> class TNetcomm_In: public RPI::Netcomm_In
	{
	public:
		TNetcomm_In(std::string name, RPI::Net* net) :
				Netcomm_In(name, net), propValue("Value", "Initial value to write to net"), outValue("outValue", this)
		{
			this->ports()->addPort(&outValue);
			this->properties()->addProperty(&propValue);
			this->setDescription("Communication Module to propagate value into net");

			// If this primitive is not assigned to a primitive net, create own mutex
			std::shared_ptr<RTT::os::Mutex> lockmutex;
			if (net)
			{
				lockmutex = net->netCommMutex;
			} else
			{
				lockmutex = std::make_shared<RTT::os::Mutex>();
			}

			this->dataptr = std::make_shared<TNetcommData<T> >(lockmutex);
			this->typekit = TypeKits::getInstance()->getTypeKit(typeid(T));
		}

		virtual ~TNetcomm_In()
		{
		}

		bool configureHook()
		{
			if (inNet)
			{
				auto dp = inNet->getCommunicationData(key);
				// Check whether another Netcomm_In already exists and share NetcommData
				if (dp)
				{
					auto tdp = std::dynamic_pointer_cast<TNetcommData<T>, NetcommData>(dp);
					if (tdp)
					{
						dataptr = tdp;
					} else
					{
						// Conversion to target type failed
						return false;
					}
				}
			}

			dataptr->setData(propValue.get());
			dataptr->setNull(false);

			return Netcomm_In::configureHook();
		}

		std::string getValue()
		{
			const auto& tmpdata = dataptr->getData();
			return typekit->toString(&tmpdata.first);
		}

		void setValue(const std::string& value)
		{
			T tmpdata;
			typekit->fromString(&tmpdata, value);
			dataptr->setData(tmpdata);
			dataptr->setNull(false);
		}

		void updateHook()
		{
			const auto& tmp = dataptr->getData();
			outValue.Set(tmp.first);
			outLastUpdated.Set(tmp.second);
			if (dataptr->isNull())
				outValue.SetNull();
		}

		virtual std::shared_ptr<NetcommData> getNetcommData() const
		{
			return dataptr;
		}

	protected:
		std::shared_ptr<TNetcommData<T> > dataptr;
		RPI::Property<T> propValue;
		RPI::OutPort<T> outValue;
	private:
		TypeKit* typekit;
	};

	template<class T> class TNetcomm_Out: public RPI::Netcomm_Out
	{
	public:
		TNetcomm_Out(std::string name, RPI::Net* net) :
				Netcomm_Out(name, net), propValue("Value", "Initial value to read from net"), inValue("inValue", this)
		{
			this->ports()->addPort(&inValue);
			this->properties()->addProperty(&propValue);
			this->setDescription("Communication module to read value from net");

			// If this primitive is not assigned to a primitive net, create own mutex
			std::shared_ptr<RTT::os::Mutex> lockmutex;
			if (net)
			{
				lockmutex = net->netCommMutex;
			} else
			{
				lockmutex = std::make_shared<RTT::os::Mutex>();
			}

			this->dataptr = std::make_shared<TNetcommData<T> >(lockmutex);
			this->typekit = TypeKits::getInstance()->getTypeKit(typeid(T));
		}

		virtual ~TNetcomm_Out()
		{
		}

		virtual bool configureHook()
		{
			dataptr->setData(propValue.get());

			return Netcomm_Out::configureHook();
		}

		std::string getValue()
		{
			const auto& tmpdata = dataptr->getData();
			return typekit->toString(&tmpdata.first);
		}

		void updateHook()
		{
			if (inValue.connected() && !inValue.isNull())
			{
				T newValue = inValue.Get();
				if (!(newValue == dataptr->getData().first))
				{
					dataptr->setData(inValue.Get());
				}
				initialized = true;
				dataptr->setNull(false);

			} else
			{
				dataptr->setNull(true);
			}
		}

		RTT::os::TimeService::nsecs getLastValueChange()
		{
			return dataptr->getData().second;
		}

		virtual std::shared_ptr<NetcommData> getNetcommData() const
		{
			return dataptr;
		}

		std::shared_ptr<TNetcommData<T> > getTNetcommData() const
		{
			return dataptr;
		}

	protected:
		std::shared_ptr<TNetcommData<T> > dataptr;
		RPI::Property<T> propValue;
		RPI::InPort<T> inValue;
	private:
		TypeKit* typekit;

	};

	template<class T> class TArrayNetcomm_In: public RPI::TNetcomm_In<RPI::Array<T> >
	{
	public:
		TArrayNetcomm_In(std::string name, RPI::Net* net) :
				TNetcomm_In<RPI::Array<T> >(name, net)
		{
		}

		virtual ~TArrayNetcomm_In()
		{
		}
	};

	template<class T> class TArrayNetcomm_Out: public RPI::Netcomm_Out
	{
	public:
		TArrayNetcomm_Out(std::string name, RPI::Net* net) :
				Netcomm_Out(name, net), propValue("Value", "Initial value to read from net"), inValue("inValue", this)
		{
			this->ports()->addPort(&inValue);
			this->properties()->addProperty(&propValue);
			this->setDescription("Communication module to read value from net");

			// If this primitive is not assigned to a primitive net, create own mutex
			std::shared_ptr<RTT::os::Mutex> lockmutex;
			if (net)
			{
				lockmutex = net->netCommMutex;
			} else
			{
				lockmutex = std::make_shared<RTT::os::Mutex>();
			}

			this->dataptr = std::make_shared<TNetcommData<RPI::Array<T>> >(lockmutex);
			this->typekit = TypeKits::getInstance()->getTypeKit(typeid(RPI::Array<T>));
		}

		virtual ~TArrayNetcomm_Out()
		{
		}

		virtual bool configureHook()
		{
			dataptr->setData(propValue.get());

			return Netcomm_Out::configureHook();
		}

		std::string getValue()
		{
			const auto& tmpdata = dataptr->getData();
			return typekit->toString(&tmpdata.first);
		}

		void updateHook()
		{
			if (inValue.connected() && !inValue.isNull())
			{
				RPI::Array<T> newValue = inValue.Get();
				RPI::Array<T> prop = dataptr->getData().first;
				initialized = true;
				for (int i = 0; i < prop.getSize(); i++)
				{
					if (newValue[i] != prop[i])
					{
						newValue.copyTo(prop);
						dataptr->updateDataTime();
						return;
					}
				}
				dataptr->setNull(false);

			} else
			{
				dataptr->setNull(true);
			}
		}

		RTT::os::TimeService::nsecs getLastValueChange()
		{
			return dataptr->getData().second;
		}

		virtual std::shared_ptr<NetcommData> getNetcommData() const
		{
			return dataptr;
		}

	protected:
		std::shared_ptr<TNetcommData<RPI::Array<T> > > dataptr;
		RPI::Property<RPI::Array<T> > propValue;
		RPI::InPort<RPI::Array<T> > inValue;
	private:
		TypeKit* typekit;

	};

}
#endif /* TNETCOMM_HPP_ */
