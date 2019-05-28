/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef NETMODULES_HPP_
#define NETMODULES_HPP_

#include "NetModulesFwd.hpp"

#include <string>
#include <limits>

#include "Net.hpp"
#include "Module.hpp"

namespace RPI
{
	// forward declarations

	class Termination: public Module
	{
	public:
		Termination(std::string name, Net* net);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

		bool getTerminationState() const;
		int getFailState() const;
	protected:
		InPort<bool> inTerminate;
		InPort<int> inFail;
	private:
		bool terminate;
		int fail;
	};

	class Cancel: public Module
	{
	public:
		Cancel(std::string name, Net* net);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

		void setCancel(bool);
		bool getCancel() const;
	protected:
		OutPort<bool> outCancel;
	private:
		bool cancel;
	};

	class Takeover: public Module
	{
	public:
		Takeover(std::string name, Net* net);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();

		void setTakeover(bool);

	protected:
		OutPort<bool> outTakeover;
	private:
		bool takeover;
	};

	class EStop: public Module
	{
	public:
		EStop(std::string name, Net* net);
		virtual ~EStop();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	protected:
		InPort<bool> inEStop;
	private:
		bool isEStop;
	};

	class DebugModule: public Module {
	public:
		DebugModule(std::string name, Net* net) :
			Module(name, net) {
		}

		virtual bool configureHook() {
			if(inNet)
				inNet->addDebugModule(this);
			return true;
		}

		virtual void output(std::stringstream& to, long tick) = 0;
		//virtual void *output(void *to, long tick) = 0;
		virtual int getSize() = 0;
		virtual long getCount() = 0;
	};

	template<class T> class Debug: public DebugModule
	{
	public:
		Debug(std::string name, Net* net, int size) :
			DebugModule(name, net), inValue("inValue", this)
		{
			this->ports()->addPort(&inValue);
			this->size = size;
			history = new T[size];
			historytime = new long[size];
			historynull = new bool[size];

			// initialize arrays to avoid bad things if uninitialized value accidentally matches cyclecouunt
			for(int i = 0; i < size; ++i)
			{
				historytime[i] = -1;
				history[i] = T();
				historynull[i] = false;
			}
		}

		virtual ~Debug() {
			delete[] history;
			delete[] historytime;
			delete[] historynull;
			history = NULL;
			historytime = NULL;
			historynull = NULL;
		}

		void output(std::stringstream& to, long tick)
		{
			if (historytime[tick % size] != tick)
			{
				to << "n/a";
			} else if (historynull[tick % size])
			{
				to << "null";
			} else
			{
				TypeKits::getInstance()->getTypeKit<T>()
						->toString(&history[tick % size], to);
			}

		}

		/*void *output(void *to, long tick) {
			if(tick > current || tick + size < current)
				return 0;
			*((T*)to) = history[tick%size];
			return (void*)(((T*)to) + 1);
		}*/
		int getSize() {
			return sizeof(T);
		}

		long getCount() {
			return size;
		}

		void updateHook()
		{
			long current = getNetCurrentCycle();
			history[current % size] = inValue.Get();
			historynull[current % size] = inValue.isNull();
			historytime[current % size] = current;
		}

		bool configureHook()
		{
			DebugModule::configureHook();
			return true;
		}

		void cleanupHook()
		{
		}

	protected:
		InPort<T> inValue;

	private:
		T* history;
		long* historytime;
		bool* historynull;
		long size;
	};

	/**
	 * Generic abstract Pre module
	 */
	class RTT_EXPORT  Pre: public StatefulModule
	{
	protected:
		Pre(std::string name, Net* net) :
			StatefulModule(name, net)
		{
		}
	public:
		bool isPre()
		{
			return true;
		}

	};

	class RTT_EXPORT NetcommData
	{
	private:
		bool _isNull;
	public:
		NetcommData()
		{
			_isNull = true;
		}
		virtual ~NetcommData()
		{
		}

		virtual void setNull(bool isNull)
		{
			this->_isNull = isNull;
		}

		virtual bool isNull()
		{
			return this->_isNull;
		}
	};

	class RTT_EXPORT Netcomm: public Module
	{
	public:
		Netcomm(std::string name, Net* net);

		virtual bool startHook();
		virtual void stopHook();

		virtual std::string getValue() = 0;
		virtual std::shared_ptr<NetcommData> getNetcommData() const = 0;
	protected:
		Property<std::string> propKey;
		std::string key;
	};

	class RTT_EXPORT Netcomm_In: public Netcomm
	{
	public:
		Netcomm_In(std::string name, Net* net);
		virtual void setValue(const std::string& value) = 0;
		virtual void cleanupHook();
		virtual bool configureHook();
	protected:
		RPI::OutPort<RTT::os::TimeService::nsecs> outLastUpdated;
	};

	class RTT_EXPORT Netcomm_Out: public Netcomm
	{
	public:
		Netcomm_Out(std::string name, Net* net);
		virtual void cleanupHook();
		virtual bool configureHook();

		bool isInitialized();

		virtual RTT::os::TimeService::nsecs getLastValueChange() = 0;
	protected:
		bool initialized;
	};

	class RTT_EXPORT InterNetcomm: public Module
	{
	public:
		InterNetcomm(std::string name, Net* net);
		bool startHook();
		void stopHook();
		void cleanupHook();

	protected:
		Property<std::string> propRemoteRCC;
		Property<std::string> propRemoteNet;
		Property<std::string> propRemoteKey;

		std::shared_ptr<NetcommData> dataptr;
	};

	class RTT_EXPORT InterNetcomm_In: public InterNetcomm
	{
	public:
		InterNetcomm_In(std::string name, Net* net);

		virtual bool configureHook();
	protected:
		RPI::OutPort<RTT::os::TimeService::nsecs> outLastUpdated;
	};

	class RTT_EXPORT InterNetcomm_Out: public InterNetcomm
	{
	public:
		InterNetcomm_Out(std::string name, Net* net);

		virtual bool configureHook();
	};


}

#endif /* NETMODULES_HPP_ */
