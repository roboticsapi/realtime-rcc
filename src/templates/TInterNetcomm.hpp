/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef TINTERNETCOMM_HPP_
#define TINTERNETCOMM_HPP_

#include "../rcc/Net.hpp"

#include "../rcc/NetModules.hpp"
#include "TArray.hpp"
#include <string.h>
#include <rtt/os/MutexLock.hpp>

namespace RPI
{

	template<class T> class TInterNetcomm_In: public RPI::InterNetcomm_In
	{
	public:
		TInterNetcomm_In(std::string name, RPI::Net* net) :
				InterNetcomm_In(name, net), outValue("outValue", this)
		{
			this->ports()->addPort(&outValue);
			this->setDescription("Communication Module read value from remote net into local net");
		}

		virtual ~TInterNetcomm_In()
		{
		}

		bool configureHook()
		{
			// first call configure Hook recursively to create dataptr
			if (!InterNetcomm_In::configureHook())
				return false;

			// Connection to remote net failed?
			if (!dataptr)
				return false;

			Tdataptr = std::dynamic_pointer_cast<TNetcommData<T>, NetcommData>(dataptr);

			// converting pointer failed, i.e. types do not match
			if (!Tdataptr)
				return false;

			return true;
		}

		void updateHook()
		{
			const auto& tmp = Tdataptr->getData();
			outValue.Set(tmp.first);
			if (Tdataptr->isNull())
				outValue.SetNull();
			outLastUpdated.Set(tmp.second);
		}

	protected:
		std::shared_ptr<TNetcommData<T> > Tdataptr;
		RPI::OutPort<T> outValue;
	};

	template<class T> class TInterNetcomm_Out: public RPI::InterNetcomm_Out
	{
	public:
		TInterNetcomm_Out(std::string name, RPI::Net* net) :
				InterNetcomm_Out(name, net), inValue("inValue", this)
		{
			this->ports()->addPort(&inValue);
			this->setDescription("Communication module to write value from net to remote net");
		}

		virtual ~TInterNetcomm_Out()
		{
		}

		virtual bool configureHook()
		{
			// first call configure Hook recursively to create dataptr
			if (!InterNetcomm_Out::configureHook())
				return false;

			// Connection to remote net failed?
			if (!dataptr)
				return false;

			Tdataptr = std::dynamic_pointer_cast<TNetcommData<T>, NetcommData>(dataptr);

			// converting pointer failed, i.e. types do not match
			if (!Tdataptr)
				return false;

			return true;
		}

		void updateHook()
		{
			if (inValue.connected() && !inValue.isNull())
			{
				T newValue = inValue.Get();
				if (!(newValue == Tdataptr->getData().first))
				{
					Tdataptr->setData(inValue.Get());
				}
				Tdataptr->setNull(false);
			} else
			{
				Tdataptr->setNull(true);
			}
		}

	protected:
		std::shared_ptr<TNetcommData<T> > Tdataptr;
		RPI::InPort<T> inValue;
	};

	template<class T> class TInterArrayNetcomm_In: public RPI::TInterNetcomm_In<RPI::Array<T> >
	{
	public:
		TInterArrayNetcomm_In(std::string name, RPI::Net* net) :
				TInterNetcomm_In<RPI::Array<T> >(name, net)
		{
		}

		virtual ~TInterArrayNetcomm_In()
		{
		}
	};

	template<class T> class TInterArrayNetcomm_Out: public RPI::InterNetcomm_Out
	{
	public:
		TInterArrayNetcomm_Out(std::string name, RPI::Net* net) :
				InterNetcomm_Out(name, net), inValue("inValue", this)
		{
			this->ports()->addPort(&inValue);
			this->setDescription("Communication module to write value from net to remote net");
		}

		virtual ~TInterArrayNetcomm_Out()
		{
		}

		virtual bool configureHook()
		{
			// first call configure Hook recursively to create dataptr
			if (!InterNetcomm_Out::configureHook())
				return false;

			// Connection to remote net failed?
			if (!dataptr)
				return false;

			Tdataptr = std::dynamic_pointer_cast<TNetcommData<RPI::Array<T>>, NetcommData>(dataptr);

			// converting pointer failed, i.e. types do not match
			if (!Tdataptr)
				return false;

			return true;
		}

		void updateHook()
		{
			if (inValue.connected() && !inValue.isNull())
			{
				RPI::Array<T> newValue = inValue.Get();
				RPI::Array<T> prop = Tdataptr->getData().first;

				Tdataptr->setNull(false);

				for (int i = 0; i < prop.getSize(); i++)
				{
					if (newValue[i] != prop[i])
					{
						newValue.copyTo(prop);
						Tdataptr->updateDataTime();
						return;
					}
				}
			} else
			{
				Tdataptr->setNull(true);
			}
		}

	protected:
		std::shared_ptr<TNetcommData<RPI::Array<T> > > Tdataptr;
		RPI::InPort<RPI::Array<T> > inValue;
	};

}
#endif /* TINTERNETCOMM_HPP_ */
