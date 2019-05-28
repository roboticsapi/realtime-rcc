/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef TPRE_HPP_
#define TPRE_HPP_

#include "../rcc/NetModules.hpp"

namespace RPI
{

	/**
	 * First module used for modelling Pre, not displayed to the user
	 */
	template<class T> class TPre: public Pre
	{
	public:
		TPre(std::string name, RPI::Net* net) :
				Pre(name, net), inValue("inValue", this), outValue("outValue", this), bufferedValue(), hasValue(
						false)
		{
			this->ports()->addPort(&inValue, "Input value");
			this->ports()->addPort(&outValue, "Output value");

		}

	protected:
		void updateHook()
		{
			if (this->active())
			{
				if (!this->reset() && !this->inValue.isNull())
				{
					bufferedValue = this->inValue.Get();
					hasValue = true;
				}
			}
		}
		void updatePre()
		{
			if (hasValue)
			{
				this->outValue.Set(bufferedValue);
				hasValue = false;
			}
		}
		// These names are also hard-coded in Net.cxx!
		RPI::InPort<T> inValue;
		RPI::OutPort<T> outValue;

	private:
		T bufferedValue;
		bool hasValue;
	};

}

#endif /* TPRE_HPP_ */
