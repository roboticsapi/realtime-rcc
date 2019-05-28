/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <rcc/Net.hpp>
#include <stdlib.h>
#include <math.h>

#include "ValueCoding.hpp"

namespace Core
{
	bool decToBinCalc(int decValue, RPI::Array<bool>& binArray, unsigned int amountbits, int min, int max)
	{
		if (decValue < min || decValue > max)
			return true;
		else
		{
			for (int i = 1, j = 0; j < amountbits; i <<= 1, ++j)
			{
				binArray[j] = (i & decValue);
			}
		}
		return false;
	}

	int binToDecCalc(RPI::Array<bool>& binArray, unsigned int amountBits, bool signBit)
	{
		int decValue = 0;

		for (int i = 1, j = 0; j < amountBits; i <<= 1, ++j)
		{
			if (binArray[j])
				decValue = decValue | i;
		}

		if (signBit && (decValue > (pow(2, amountBits - 1) - 1)))
			decValue = decValue - pow(2, amountBits);

		return decValue;
	}

	DecToBin::DecToBin(std::string name, RPI::Net* net) :
			ActiveModule(name, net), inValue("inValue", this), outArray("outArray", this), overflowBit("overflowBit",
					this), amountBits("amountBits", "Size of the Array", 8), signBit("signBit",
					"Bit responsible for sign", false), binArray(), max(), min()
	{
		setDescription("A decimal to binary value-coding module");
		this->ports()->addPort(&inValue, "Incoming Value");
		this->ports()->addPort(&outArray, "Array");
		this->ports()->addPort(&overflowBit, "StackOverflow Bit");

		this->properties()->addProperty(&amountBits);
		this->properties()->addProperty(&signBit);
	}

	bool DecToBin::configureHook()
	{
		binArray.resize(amountBits.get());
		overflowBit.Set(false);

		if (signBit.get())
		{
			max = pow(2, amountBits.get() - 1) - 1;
			min = -pow(2, amountBits.get() - 1);
		} else
		{
			max = pow(2, amountBits.get()) - 1;
			min = 0;
		}
		return true;
	}
	bool DecToBin::startHook()
	{
		return true;
	}
	void DecToBin::updateHook()
	{
		int decValue = inValue.Get();

		overflowBit.Set(decToBinCalc(decValue, binArray, amountBits.get(), min, max));

		outArray.Set(binArray);
	}
	void DecToBin::stopHook()
	{
	}
	void DecToBin::cleanupHook()
	{
	}

	// -------------------------------------------- //
	BinToDec::BinToDec(std::string name, RPI::Net* net) :
			ActiveModule(name, net), inArray("inArray", this), outValue("outValue", this), decValue(), amountBits(
					"amountBits", "Size of the Array", 8), signBit("signBit", "Bit responsible for sign", false)
	{
		setDescription("A decimal to binary value-coding module");
		this->ports()->addPort(&inArray, "Incoming Array");
		this->ports()->addPort(&outValue, "Outgoing Value");

		this->properties()->addProperty(&amountBits);
		this->properties()->addProperty(&signBit);
	}

	bool BinToDec::configureHook()
	{
		return true;
	}
	bool BinToDec::startHook()
	{
		return true;
	}
	void BinToDec::updateHook()
	{
		RPI::Array<bool> binArray = inArray.Get();

		decValue = binToDecCalc(binArray, amountBits.get(), signBit.get());

		outValue.Set(decValue);
	}
	void BinToDec::stopHook()
	{
	}
	void BinToDec::cleanupHook()
	{
	}

// -------------------------------------------- //
	DecToBCD::DecToBCD(std::string name, RPI::Net* net) :
			ActiveModule(name, net), inValue("inValue", this), outArray("outArray", this), overflowBit("overflowBit",
					this), amountBits("amountBits", "Bits of the Array", 16), signBit("signBit",
					"Bit responsible for possible sign", true), BCDArray(), value(), state(), length(), min(), max(), neg(),numberBin()
	{
		setDescription("A decimal to binary value-coding module");
		this->ports()->addPort(&inValue, "Incoming Value");
		this->ports()->addPort(&outArray, "Array");
		this->ports()->addPort(&overflowBit, "StackOverflow Bit");

		this->properties()->addProperty(&amountBits);
		this->properties()->addProperty(&signBit);
	}

	bool DecToBCD::configureHook()
	{
		BCDArray.resize(amountBits.get());
		overflowBit.Set(false);

		if (signBit.get())
		{
			min = -pow(10, ((amountBits.get() / 4) - 1)) + 1;
			max = pow(10, ((amountBits.get() / 4) - 1)) - 1;
		} else
		{
			min = 0;
			max = pow(10, (amountBits.get() / 4)) - 1;
		}
		return true;
	}
	bool DecToBCD::startHook()
	{
		return true;
	}
	void DecToBCD::updateHook()
	{
		int decValue = inValue.Get();

		decToBCDCalc(decValue, amountBits.get(), signBit.get());

		outArray.Set(BCDArray);
	}
	void DecToBCD::stopHook()
	{
	}
	void DecToBCD::cleanupHook()
	{
	}

	void DecToBCD::decToBCDCalc(int decValue, int amountBits, bool signBit)
	{
		if (decValue < min || decValue > max)
		{
			overflowBit.Set(true);
		} else
		{
			value = 0;
			state = 0;
			neg = false;
			numberBin.resize(4);

			if (decValue < 0)
			{
				neg = true;
				decValue = -decValue;
			}
			for (int i = 10; state < amountBits / 4; i *= 10, ++state)
			{
				value = (decValue % i) / (i / 10);
				decToBinCalc(value, numberBin, 4, 0, 9);

				for (int j = 0; j < 4; ++j)
				{
					BCDArray[state * 4 + j] = numberBin[j];
				}
			}

			if (signBit && neg)
				BCDArray[amountBits - 4] = true;

			overflowBit.Set(false);
		}
	}

// -------------------------------------------- //

	BCDToDec::BCDToDec(std::string name, RPI::Net* net) :
			ActiveModule(name, net), inArray("inArray", this), outValue("outValue", this), decValue(), exp(), stages(), value(), amountBits(
			"amountBits", "Bits of the Array", 16), signBit("signBit", "Bit responsible for possible sign",true), BCDArray()
	{
		setDescription("A decimal to binary value-coding module");
		this->ports()->addPort(&inArray, "Incoming Values");
		this->ports()->addPort(&outValue, "Outgoing Value");

		this->properties()->addProperty(&amountBits);
		this->properties()->addProperty(&signBit);
	}

	bool BCDToDec::configureHook()
	{
		return true;
	}
	bool BCDToDec::startHook()
	{
		return true;
	}
	void BCDToDec::updateHook()
	{
		RPI::Array<bool> BCDArray = inArray.Get();

		BCDToDecCalc(BCDArray, amountBits.get(), signBit.get());

		outValue.Set(decValue);
	}
	void BCDToDec::stopHook()
	{
	}
	void BCDToDec::cleanupHook()
	{
	}

	void BCDToDec::BCDToDecCalc(RPI::Array<bool> binArray, int amountBits, bool signBit)
	{
		decValue = 0;
		stages = amountBits / 4;
		value = 0;
		exp = 1;
		BCDArray.resize(4);

		if (signBit)
			stages--;

		for (int i = 0; i < stages; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				BCDArray[j] = binArray[j + (i * 4)];

			}
			value = binToDecCalc(BCDArray, 4, false);
			exp = pow(10, i);
			decValue += value * exp;
		}
		if (signBit && binArray[amountBits - 4] == 1)
			decValue = -decValue;
	}
}
