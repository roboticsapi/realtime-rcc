/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef TARITHMETIC_HPP_
#define TARITHMETIC_HPP_

namespace RPI {

/**
 * \brief Generic RPI module for addition
 */
template<class T> class TAdd: public RPI::Module
{

public:
	TAdd(std::string name, RPI::Net* net) :
		Module(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue("outValue", this),
				propFirst("First", "First value", T()), propSecond("Second", "Second value", T())
	{
		setDescription("An addition module");
		this->ports()->addPort(&inFirst, "First value");
		this->ports()->addPort(&inSecond, "Second value");
		this->ports()->addPort(&outValue, "Result (First + Second)");

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
		outValue.Set(first + second);
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
	RPI::OutPort<T> outValue;

	RPI::Property<T> propFirst;
	RPI::Property<T> propSecond;
};

/**
 * \brief Generic RPI module for multiplication
 */
template<class T> class TMultiply: public RPI::Module
{

public:
	TMultiply(std::string name, RPI::Net* net) :
		Module(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue("outValue", this),
				propFirst("First", "First value", T()), propSecond("Second", "Second value", T())
	{
		setDescription("A multiplication module");
		this->ports()->addPort(&inFirst, "First value");
		this->ports()->addPort(&inSecond, "Second value");
		this->ports()->addPort(&outValue, "Result (First * Second)");

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
		outValue.Set(first * second);
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
	RPI::OutPort<T> outValue;

	RPI::Property<T> propFirst;
	RPI::Property<T> propSecond;
};


/**
 * \brief Generic RPI module for division
 */
template<class T> class TDivide: public RPI::Module
{

public:
	TDivide(std::string name, RPI::Net* net) :
		Module(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue("outValue", this),
				propFirst("First", "First value", T()), propSecond("Second", "Second value", T())
	{
		setDescription("A division module");
		this->ports()->addPort(&inFirst, "First value");
		this->ports()->addPort(&inSecond, "Second value");
		this->ports()->addPort(&outValue, "Result (First / Second)");

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
		if(second != 0 || std::numeric_limits<T>::has_infinity) {
			outValue.Set(first / second);
		}
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
	RPI::OutPort<T> outValue;

	RPI::Property<T> propFirst;
	RPI::Property<T> propSecond;
};

}

#endif /* TARITHMETIC_HPP_ */
