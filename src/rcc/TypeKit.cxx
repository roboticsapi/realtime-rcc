/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "TypeKit.hpp"
#include "TypeKitT.hpp"
#include <cmath>
#include <iostream>

using namespace std;

namespace RPI
{

	TypeKit::TypeKit(const std::string& humanname)
	{
		this->humanname = humanname;
	}

	TypeKit::~TypeKit()
	{

	}

	std::string TypeKit::getHumanName() const
	{
		return humanname;
	}


	class BoolTypeKit: public TypeKitT<bool>
	{
	public:
		BoolTypeKit() :
				TypeKitT<bool>("Core::bool")
		{
		}

		virtual ~BoolTypeKit()
		{

		}

		virtual void toString(const void* data, std::stringstream& value) const
		{
			value << (*((bool*) data) ? "true" : "false");
		}
		virtual void fromString(void* data, std::stringstream& value) const
		{
			int chr = value.get();
			std::stringstream bval;
			while(chr >= 'a' && chr <= 'z') {
				bval << (char) chr;
				chr = value.get();
			}

			value.putback(chr);

			*((bool*) data) = (bval.str() == "true");
		}
	};

	class DoubleTypeKit: public TypeKitT<double>
	{
	public:
		DoubleTypeKit() :
				TypeKitT<double>("Core::double")
		{
		}

		virtual ~DoubleTypeKit()
		{

		}

		virtual void toString(const void* data, std::stringstream& value) const
		{
			double d = *((double*) data);
			if (d != d)
				value << "nan";
			else if (d == -std::numeric_limits<double>::infinity())
				value << "-inf";
			else if (d == std::numeric_limits<double>::infinity())
				value << "+inf";
			else
				value << std::setprecision(15) << d;
		}

		virtual void fromString(void* data, std::stringstream& value) const
		{
			int chr = value.get();
			std::stringstream bval;
			while (chr != -1 && strchr("+-naif0123456789.eE", chr) != 0)
			{
				bval << (char) chr;
				chr = value.get();
			}
			std::string s = bval.str();
			value.putback(chr);
			if (s == "nan")
				*((double*) data) = std::numeric_limits<double>::quiet_NaN();
			else if (s == "+inf" || s == "inf")
				*((double*) data) = std::numeric_limits<double>::infinity();
			else if (s == "-inf")
				*((double*) data) = -std::numeric_limits<double>::infinity();
			else
				*((double*) data) = strtod(s.c_str(), 0);
		}
	};

	class StringTypeKit: public TypeKitT<std::string>
	{
	public:
		StringTypeKit() :
				TypeKitT<std::string>("Core::string")
		{

		}

		virtual ~StringTypeKit()
		{

		}

		virtual void toString(const void* data, std::stringstream& value) const
		{
			value << *((std::string*) data);
		}
		virtual void fromString(void* data, std::stringstream& value) const
		{
			*((std::string*) data) = value.str();
		}
	};

	class VoidTypeKit: public TypeKit
	{
	public:
		VoidTypeKit() :
				TypeKit("unknown")
		{

		}

		virtual ~VoidTypeKit()
		{

		}

		virtual DebugModule* createDebug(const std::string& name, Net* net,
				int size) const
		{
			return 0;
		}
		virtual void toString(const void* data, std::stringstream& value) const
		{
			value << "";
		}
		virtual void fromString(void* data, std::stringstream& value) const
		{

		}
	};

	TypeKits* TypeKits::theInstance = 0;

	TypeKits* TypeKits::getInstance()
	{
		if (!theInstance)
			theInstance = new TypeKits();
		return theInstance;
	}

	TypeKits::TypeKits()
	{
		registerTypeKit(typeid(bool), new BoolTypeKit());
		registerTypeKit(typeid(int), new TypeKitTStream<int>("Core::int"));
		registerTypeKit(typeid(double), new DoubleTypeKit());
		registerTypeKit(typeid(string), new StringTypeKit());

//		registerTypeKit(typeid(KDL::Vector), new VectorTypeKit());
//		registerTypeKit(typeid(KDL::Rotation), new RotationTypeKit());
//		registerTypeKit(typeid(KDL::Frame), new FrameTypeKit());
//		registerTypeKit(typeid(KDL::Twist), new TwistTypeKit());
//		registerTypeKit(typeid(KDL::Wrench), new WrenchTypeKit());

		registerTypeKit(typeid(Array<bool>), new ArrayTypeKit<bool>("Core::bool[]"));
		registerTypeKit(typeid(Array<int>), new ArrayTypeKit<int>("Core::int[]"));
		registerTypeKit(typeid(Array<double>), new ArrayTypeKit<double>("Core::double[]"));
	}

	void TypeKits::registerTypeKit(const type_info& type, TypeKit* typekit)
	{
		std::string strtypename = string(type.name());

		// don't overwrite existing typekits [why not?]
		//	if(found != typeKits.end()) {
		//		return;
		//	}
		typeKits[strtypename] = typekit;
	}

	void TypeKits::unregisterTypeKit(const type_info& type)
	{
		std::string strtypename = string(type.name());

		typekits_t::const_iterator it = typeKits.find(strtypename);
		if (it != typeKits.end())
		{
			delete typeKits[strtypename];
			typeKits.erase(strtypename);
		}
	}

	std::list<TypeKit*> TypeKits::getTypeKits() {
		std::list<TypeKit*> ret;
		for(typekits_t::iterator it = typeKits.begin(); it != typeKits.end(); ++it) {
			ret.push_back(it->second);
		}
		return ret;
	}

	TypeKit* TypeKits::getTypeKit(const std::type_info& type)
	{
		std::string strtypename = string(type.name());
		typekits_t::const_iterator it = typeKits.find(strtypename);

		// unknown type, create a new void typekit for this instance
		if (it == typeKits.end())
		{
			typeKits[strtypename] = new VoidTypeKit();
			return typeKits[strtypename];
		}
		return it->second;
	}

	std::string TypeKit::toString(const void* data) const
	{
		stringstream tmp;
		this->toString(data, tmp);
		return tmp.str();
	}

	void TypeKit::fromString(void* data, const string& value) const
	{
		stringstream tmp(value);
		this->fromString(data, tmp);
	}


}
