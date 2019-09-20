/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef TCOMPARISON_HPP_
#define TCOMPARISON_HPP_

namespace RPI {

/**
 * \brief Generic RPI module to compare two values for equality (w.r.t. to epsilon)
 * \param AbsTool Helper class providing static method T abs(T x) which calculates the absolute
 * 	value of x
 */
template<class T, class AbsTool> class TEquals: public RPI::Module
{

public:
	TEquals(std::string name, RPI::Net* net) :
		Module(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue("outValue", this),
				propFirst("First", "First value", T()), propSecond("Second", "Second value", T()),
				propEpsilon("Epsilon", "Allowed difference", T())
	{
		setDescription("A comparison module");
		this->ports()->addPort(&inFirst, "First value");
		this->ports()->addPort(&inSecond, "Second value");
		this->ports()->addPort(&outValue, "Result (true if First = Second)");

		this->properties()->addProperty(&propFirst);
		this->properties()->addProperty(&propSecond);
		this->properties()->addProperty(&propEpsilon);
	}

	bool configureHook()
	{
		return true;
	}
	bool startHook()
	{
		return true;
	}
	void updateHook()
	{
		T first = inFirst.Get(propFirst);
		T second = inSecond.Get(propSecond);
		outValue.Set(AbsTool::abs(first - second) <= propEpsilon.get());
	}
	void stopHook()
	{
	}
	void cleanupHook()
	{
	}

protected:
	RPI::InPort<T> inFirst;
	RPI::InPort<T> inSecond;
	RPI::OutPort<bool> outValue;

	RPI::Property<T> propFirst;
	RPI::Property<T> propSecond;
	RPI::Property<T> propEpsilon;
};

/**
 * \brief Generic RPI module to compare two values
 * \param T Type of dataflow, must support operator>
 */
template<class T> class TGreater: public RPI::Module
{

public:
	TGreater(std::string name, RPI::Net* net) :
		Module(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue("outValue", this),
				propFirst("First", "First value", T()), propSecond("Second", "Second value", T())
	{
		setDescription("A comparison module");
		this->ports()->addPort(&inFirst, "First value");
		this->ports()->addPort(&inSecond, "Second value");
		this->ports()->addPort(&outValue, "Result (true if First > Second)");

		this->properties()->addProperty(&propFirst);
		this->properties()->addProperty(&propSecond);
	}

	bool configureHook()
	{
		return true;
	}
	bool startHook()
	{
		return true;
	}
	void updateHook()
	{
		T first = inFirst.Get(propFirst);
		T second = inSecond.Get(propSecond);
		outValue.Set(first > second);
	}
	void stopHook()
	{
	}
	void cleanupHook()
	{
	}

protected:
	RPI::InPort<T> inFirst;
	RPI::InPort<T> inSecond;
	RPI::OutPort<bool> outValue;

	RPI::Property<T> propFirst;
	RPI::Property<T> propSecond;
};

/**
 * \brief Generic RPI module which outputs data from one or another input depending on boolean condition
 */
template<class T> class TConditional: public RPI::Module
{

public:
	TConditional(std::string name, RPI::Net* net) :
		Module(name, net), inCondition("inCondition", this), inTrue("inTrue", this), inFalse("inFalse", this),
				outValue("outValue", this), propTrue("True", "Value if true", T()),
				propFalse("False", "Value if false", T())
	{
		setDescription("Conditional");
		this->ports()->addPort(&inCondition, "Condition");
		this->ports()->addPort(&inTrue, "Frame if condition is true");
		this->ports()->addPort(&inFalse, "Frame if condition is false");
		this->ports()->addPort(&outValue, "Result");

		this->properties()->addProperty(&propTrue);
		this->properties()->addProperty(&propFalse);
	}

	bool configureHook()
	{
		if (!inCondition.connected())
			return false;
		return true;
	}

	void updateHook()
	{
		bool cond = inCondition.Get();
		T t = inTrue.Get(propTrue);
		T f = inFalse.Get(propFalse);
		outValue.Set(cond ? t : f);
	}

	bool startHook()
	{
		return true;
	}
	void stopHook()
	{
	}
	void cleanupHook()
	{
	}

protected:
	RPI::InPort<bool> inCondition;
	RPI::InPort<T> inTrue;
	RPI::InPort<T> inFalse;
	RPI::OutPort<T> outValue;

	RPI::Property<T> propTrue;
	RPI::Property<T> propFalse;

};

}

#endif /* TCOMPARISON_HPP_ */
