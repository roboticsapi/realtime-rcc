/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef MODULE_HPP_
#define MODULE_HPP_

#include <rcc/Module.hpp>
#include <rcc/ModuleBase.hpp>
#include <templates/TArray.hpp>

namespace Core
{
	bool decToBinCalc(int decValue, RPI::Array<bool>& binArray, unsigned int amountbits, int min, int max);
	int binToDecCalc(RPI::Array<bool>& binArray, unsigned int amountBits, bool signBit);

	class DecToBin: public RPI::ActiveModule
	{
	public:
		DecToBin(std::string name, RPI::Net* net);
		bool configureHook();
		bool startHook();
		void stopHook();
		void updateHook();
		void cleanupHook();

	protected:
		RPI::InPort<int> inValue;
		RPI::OutPort<RPI::Array<bool> > outArray;
		RPI::OutPort<bool> overflowBit;

		RPI::Property<unsigned int> amountBits;
		RPI::Property<bool> signBit;

	private:
		RPI::Array<bool> binArray;
		int max, min;
	};

	class BinToDec: public RPI::ActiveModule
	{
	public:
		BinToDec(std::string name, RPI::Net* net);
		bool configureHook();
		bool startHook();
		void stopHook();
		void updateHook();
		void cleanupHook();

	protected:
		RPI::InPort<RPI::Array<bool> > inArray;
		RPI::OutPort<int> outValue;

		RPI::Property<unsigned int> amountBits;
		RPI::Property<bool> signBit;

	private:
		unsigned int decValue;
	};

	class DecToBCD: public RPI::ActiveModule
	{
	public:
		DecToBCD(std::string name, RPI::Net* net);
		bool configureHook();
		bool startHook();
		void stopHook();
		void updateHook();
		void cleanupHook();

	protected:
		RPI::InPort<int> inValue;
		RPI::OutPort<RPI::Array<bool> > outArray;
		RPI::OutPort<bool> overflowBit;

		RPI::Property<unsigned int> amountBits;
		RPI::Property<bool> signBit;

	private:
		void decToBCDCalc(int decValue, int amountBits, bool signBit);
		RPI::Array<bool> BCDArray;
		RPI::Array<bool> numberBin;
		unsigned int value, state, length;
		int min, max;
		bool neg;
	};

	class BCDToDec: public RPI::ActiveModule
	{
	public:
		BCDToDec(std::string name, RPI::Net* net);
		bool configureHook();
		bool startHook();
		void stopHook();
		void updateHook();
		void cleanupHook();

	protected:
		RPI::InPort<RPI::Array<bool> > inArray;
		RPI::OutPort<int> outValue;

		RPI::Property<unsigned int> amountBits;
		RPI::Property<bool> signBit;

	private:
		void BCDToDecCalc(RPI::Array<bool> binArray, int amountBits, bool signBit);
		unsigned int decValue, exp, stages, value;
		RPI::Array<bool> BCDArray;
	};
}

#endif /* MODULE_HPP_ */
