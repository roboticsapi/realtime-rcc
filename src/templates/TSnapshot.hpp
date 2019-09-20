/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef TSNAPSHOT_HPP_
#define TSNAPSHOT_HPP_

namespace RPI {

	/**
	 * \brief Generic RPI module to preserve data flow value from certain time
	 */
	template<class T> class TSnapshot: public RPI::StatefulModule
	{

	public:
		TSnapshot(std::string name, RPI::Net* net) :
			StatefulModule(name, net), inSnapshot("inSnapshot", this), inValue("inValue", this), outValue("outValue", this),
			propValue("Value","Initial value", T()), value()
		{
			setDescription("A snapshot module (remembers the last value of inValue when inSnapshot was true)");

			this->ports()->addPort(&inSnapshot, "Snapshot activator");
			this->ports()->addPort(&inValue, "Value to watch");
			this->ports()->addPort(&outValue, "Value of inValue when inSnapshot was true");

			this->properties()->addProperty(&propValue);
		}

		bool configureHook()
		{
			if(!inSnapshot.connected()) return false;
			if(!inValue.connected()) return false;
			return true;
		}
		bool startHook()
		{
			value = propValue.get();
			return true;
		}
		void updateHook()
		{
			if (active())
			{
				if(reset()) {
					value = propValue.get();
				}

				if (inSnapshot.Get())
				{
					value = inValue.Get();
				}
				outValue.Set(value);
			}
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<bool> inSnapshot;
		RPI::InPort<T> inValue;
		RPI::OutPort<T> outValue;

		RPI::Property<T> propValue;
	private:
		T value;
	};

}

#endif /* TSNAPSHOT_HPP_ */
