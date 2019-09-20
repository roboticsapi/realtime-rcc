/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef TARRAY_HPP_
#define TARRAY_HPP_

#include "../rcc/Module.hpp"

namespace RPI
{


	/**
	 * \brief Module to create an empty array dataflow
	 */
	template<class T> class TArray: public RPI::Module
	{

	public:
		TArray(std::string name, RPI::Net* net) :
				RPI::Module(name, net), outArray("outArray", this), propSize("Size", "Size of the array", 1), value()
		{
			setDescription("Creates an array.");
			this->ports()->addPort(&outArray, "Value");

			this->properties()->addProperty(&propSize);
		}

		bool configureHook()
		{
			value.resize(propSize.get());
			return true;
		}

		void updateHook()
		{
			outArray.Set(value);
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
		RPI::OutPort< Array<T> > outArray;

		RPI::Property<int> propSize;
	private:
		Array<T> value;
	};

	/**
	 * \brief Module to extract a single item of an array
	 */
	template<class T> class TArrayGet: public RPI::Module
	{

	public:
		TArrayGet(std::string name, RPI::Net* net) :
				RPI::Module(name, net), inArray("inArray", this), outValue("outValue", this), propSize("Size",
						"Size of the array", 1), propIndex("Index", "Item index", 0)
		{
			setDescription("Reads an array value.");
			this->ports()->addPort(&inArray, "Input array");
			this->ports()->addPort(&outValue, "Extracted value");

			this->properties()->addProperty(&propSize);
			this->properties()->addProperty(&propIndex);
		}

		bool configureHook()
		{
			if (!inArray.connected())
				return false;

			if (propIndex.get() >= propSize.get())
				return false;
			return true;
		}

		void updateHook()
		{
			Array<T> value = inArray.Get();
			outValue.Set(value[propIndex.get()]);
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
		RPI::InPort< Array<T> > inArray;
		RPI::OutPort<T> outValue;

		RPI::Property<int> propSize;
		RPI::Property<int> propIndex;
	};

	/**
	 * \brief Module to slice parts from an array dataflow
	 */
	template<class T> class TArraySlice: public RPI::Module
	{

	public:
		TArraySlice(std::string name, RPI::Net* net) :
				RPI::Module(name, net), inArray("inArray", this), outArray("outArray", this), propSize("Size",
						"Size of the array", 1), propFrom("Start", "Start index", 0), array()
		{
			setDescription("Extracts a part of an array.");
			this->ports()->addPort(&inArray, "Input array");
			this->ports()->addPort(&outArray, "Output array");

			this->properties()->addProperty(&propSize);
			this->properties()->addProperty(&propFrom);
		}

		bool configureHook()
		{
			if (!inArray.connected())
				return false;

			array.resize(propSize.get());

			if (propFrom.get() < 0)
				return false;
			return true;
		}

		void updateHook()
		{
			Array<T> value = inArray.Get();
			value.copyOfRange(array, propFrom.get());
			outArray.Set(array);
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
		RPI::InPort< Array<T> > inArray;
		RPI::OutPort< Array<T> > outArray;

		RPI::Property<int> propSize;
		RPI::Property<int> propFrom;
		Array<T> array;
	};

	/**
	 * \brief Module to write a value into specified position in an array dataflow
	 */
	template<class T> class TArraySet: public RPI::Module
	{

	public:
		TArraySet(std::string name, RPI::Net* net) :
				RPI::Module(name, net), inArray("inArray", this), inValue("inValue", this), outArray("outArray",
						this), propSize("Size", "Size of the array", 1), propIndex("Index", "Item index", 0), value()
		{
			setDescription("Sets an array value.");
			this->ports()->addPort(&inArray, "Input array");
			this->ports()->addPort(&inValue, "Value");
			this->ports()->addPort(&outArray, "Output array");

			this->properties()->addProperty(&propSize);
			this->properties()->addProperty(&propIndex);
		}

		bool configureHook()
		{
			if (!inArray.connected())
				return false;
			if (!inValue.connected())
				return false;

			if (propIndex.get() >= propSize.get())
				return false;
			value.resize(propSize.get());
			return true;
		}

		void updateHook()
		{
			inArray.Get().copyTo(value);
			value[propIndex.get()] = inValue.Get();
			outArray.Set(value);
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
		RPI::InPort< Array<T> > inArray;
		RPI::InPort< T > inValue;
		RPI::OutPort< Array<T> > outArray;

		RPI::Property<int> propSize;
		RPI::Property<int> propIndex;

	private:
		Array<T> value;

	};

	/**
	 * \brief Module to set an array data-flow to null
	 */
	template<class T> class TArraySetNull: public RPI::Module
	{

	public:
		TArraySetNull(std::string name, RPI::Net* net) :
				RPI::Module(name, net), inValue("inValue", this), outValue("outValue", this), propSize("Size",
						"Size of the array", 1), inNull("inNull", this), value()
		{
			setDescription("Sets an array value to null.");
			this->ports()->addPort(&inValue, "Input array");
			this->ports()->addPort(&outValue, "Output array");
			this->ports()->addPort(&inNull, "whether to set array to null");

			this->properties()->addProperty(&propSize);
		}

		bool configureHook()
		{
			if (!inValue.connected())
				return false;

			value.resize(propSize.get());
			return true;
		}

		void updateHook()
		{
			if (!inNull.Get() && !inValue.isNull())
			{
				inValue.Get().copyTo(value);
				outValue.Set(value);
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
		RPI::InPort<Array<T> > inValue;
		RPI::OutPort<Array<T> > outValue;
		RPI::InPort<bool> inNull;

		RPI::Property<int> propSize;

	private:
		Array<T> value;

	};

	/**
	 * \brief Generic RPI module to check if an array is null
	 */
	template<class T> class TArrayIsNull: public RPI::Module
	{

	public:
		TArrayIsNull(std::string name, RPI::Net* net) :
				RPI::Module(name, net), inValue("inValue", this), outValue("outValue", this)
		{
			setDescription("Checks if an array is null");
			this->ports()->addPort(&inValue, "input array");
			this->ports()->addPort(&outValue, "true if array is null");
		}

		bool configureHook()
		{
			if (!inValue.connected())
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
		RPI::InPort<Array<T> > inValue;
		RPI::OutPort<bool> outValue;

	};

}

#endif /* TARRAY_HPP_ */

