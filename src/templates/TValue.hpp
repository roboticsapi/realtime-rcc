/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef TVALUE_HPP_
#define TVALUE_HPP_


namespace RPI {

/**
 * \brief Generic RPI module to inject a constant value
 */
template<class T> class TValue: public RPI::Module
{

public:
	TValue(std::string name, RPI::Net* net) :
		RPI::Module(name, net), outValue("outValue", this), propValue("Value", "Value to output", T())
	{
		setDescription("value injection module");
		this->ports()->addPort(&outValue, "Value");

		this->properties()->addProperty(&propValue);
	}

	bool configureHook()
	{
		return true;
	}

	void updateHook()
	{
		outValue.Set(propValue.get());
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
	RPI::OutPort<T> outValue;

	RPI::Property<T> propValue;
};

/**
 * \brief Generic RPI module to check if a value is null
 */
template<class T> class TIsNull: public RPI::Module
{

public:
	TIsNull(std::string name, RPI::Net* net) :
		RPI::Module(name, net), inValue("inValue", this), outValue("outValue", this)
	{
		setDescription("Checks if a value is null");
		this->ports()->addPort(&inValue, "value");
		this->ports()->addPort(&outValue, "true if value is null");
	}

	bool configureHook()
	{
		if(!inValue.connected())
			return false;
		return true;
	}

	void updateHook()
	{
		outValue.Set(inValue.isNull());
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
	RPI::InPort<T> inValue;
	RPI::OutPort<bool> outValue;

};


/**
 * \brief Generic RPI module to return a value or null if requested
 */
template<class T> class TSetNull: public RPI::Module
{

public:
	TSetNull(std::string name, RPI::Net* net) :
		RPI::Module(name, net), inValue("inValue", this), inNull("inNull", this), outValue("outValue", this)
	{
		setDescription("Returns a value or null if requested");
		this->ports()->addPort(&inValue, "value to return");
		this->ports()->addPort(&inNull, "return null");
		this->ports()->addPort(&outValue, "null if inNull is true, otherwise value");
	}

	bool configureHook()
	{
		if(!inNull.connected())
			return false;
		if(!inValue.connected())
			return false;
		return true;
	}

	void updateHook()
	{
		if(!inNull.Get() && !inValue.isNull())
		{
			outValue.Set(inValue.Get());
		}
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
	RPI::InPort<T> inValue;
	RPI::InPort<bool> inNull;
	RPI::OutPort<T> outValue;

};


/**
 * \brief Generic RPI module to check if a value is null
 */
template<class T> class TAtTime: public RPI::Module
{

public:
	TAtTime(std::string name, RPI::Net* net) :
		RPI::Module(name, net),
		inValue("inValue", this), inAge("inAge", this),
		outValue("outValue", this),
		propMaxAge("MaxAge", "Maximum age supported (s)", 1),
		propAge("Age", "Age to return the value for (s)", 0),
		history(0), historyIndex(0), historySize(0), maxAge(0)
	{
		setDescription("Retrieves the value at a given time (age)");
		this->ports()->addPort(&inValue, "Current value");
		this->ports()->addPort(&inAge, "Age to return the value for (s)");
		this->ports()->addPort(&outValue, "Value at the given time");
		this->properties()->addProperty(&propAge);
		this->properties()->addProperty(&propMaxAge);

	}

	bool configureHook()
	{
		if(!inValue.connected())
			return false;
		maxAge = propMaxAge.get();
		historySize = ceil(maxAge / getInNet()->getNetFrequency()) + 1;
		historyIndex = 0;
		history = new T[historySize];
		for(int i=0; i<historySize; i++) history[i] = T();
		return true;
	}

	void updateHook()
	{
		history[historyIndex % historySize] = inValue.Get();
		double age = inAge.Get(propAge);
		if(age < 0) age = 0; // don't allow asking about the future
		int pos = floor((age / maxAge) * (historySize-1)) + 1;
		if(pos >= historySize) pos = historySize - 1;
		pos = historyIndex - pos;
		if(pos < 0) pos = 0;
		outValue.Set(history[pos%historySize]);
		historyIndex++;
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
		delete[] history;
		history = 0;
	}

protected:
	T* history;
	int historyIndex;
	int historySize;
	double maxAge;
	RPI::InPort<T> inValue;
	RPI::InPort<double > inAge;
	RPI::OutPort<T> outValue;
	RPI::Property<double> propMaxAge;
	RPI::Property<double> propAge;

};


}

#endif /* TVALUE_HPP_ */
