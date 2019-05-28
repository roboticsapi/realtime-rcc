/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "TypeKit.hpp"
#include "TypeKitT.hpp"
#include "kdl/frames.hpp"
#include <cmath>

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

//	class VectorTypeKit: public JSONTypeKit<KDL::Vector> {
//	public:
//		VectorTypeKit() : JSONTypeKit<KDL::Vector>("World::Vector")
//		{
//		}
//
//		virtual ~VectorTypeKit()
//		{
//
//		}
//
//		virtual void toString(const KDL::Vector* data, std::stringstream& value) const {
//			printMember<double>(data->x(), std::string("x"), value, true);
//			printMember<double>(data->y(), std::string("y"), value, false);
//			printMember<double>(data->z(), std::string("z"), value, false);
//		}
//		virtual void handleMember(KDL::Vector* data, std::string key, std::stringstream& value) const {
//			if(key=="x") data->x(fromString<double>(value));
//			if(key=="y") data->y(fromString<double>(value));
//			if(key=="z") data->z(fromString<double>(value));
//		}
//		virtual void getMemberInfo(members_t& members) const {
//			addMember<double>(members, "x", "X position [m]");
//			addMember<double>(members, "y", "Y position [m]");
//			addMember<double>(members, "z", "Z position [m]");
//		}
//	};
//
//	class RotationTypeKit: public BasicJSONTypeKit<KDL::Rotation,double> {
//	public:
//		RotationTypeKit() : BasicJSONTypeKit<KDL::Rotation,double>("World::Rotation")
//		{
//		}
//
//		virtual ~RotationTypeKit()
//		{
//
//		}
//
//		virtual void toString(const KDL::Rotation* data, std::stringstream& value) const {
//			double a, b, c; data->GetRPY(c, b, a);
//			printMember<double>(a, "a", value, true);
//			printMember<double>(b, "b", value, false);
//			printMember<double>(c, "c", value, false);
//		}
//		virtual double* beginMembers(KDL::Rotation* data) const {
//			double* ret = new double[3];
//			ret[0] = 0; ret[1] = 0; ret[2] = 0;
//			return ret;
//		}
//		virtual void handleMember(double* data, std::string key, std::stringstream& value) const {
//			if(key=="a") data[0] = fromString<double>(value);
//			if(key=="b") data[1] = fromString<double>(value);
//			if(key=="c") data[2] = fromString<double>(value);
//		}
//		virtual void endMembers(KDL::Rotation* data, double* json) const {
//			*data = KDL::Rotation::RPY(json[2], json[1], json[0]);
//			delete[] json;
//		}
//		virtual void getMemberInfo(members_t& members) const {
//			addMember<double>(members, "a", "A rotation (rad, around Z)");
//			addMember<double>(members, "b", "B rotation (rad, around Y)");
//			addMember<double>(members, "c", "C rotation (rad, around X)");
//		}
//	};
//
//	class FrameTypeKit: public JSONTypeKit<KDL::Frame> {
//	public:
//		FrameTypeKit() : JSONTypeKit<KDL::Frame>("World::Frame")
//		{
//		}
//
//		virtual ~FrameTypeKit()
//		{
//
//		}
//
//		virtual void toString(const KDL::Frame* data, std::stringstream& value) const {
//			printMember<KDL::Vector>(data->p, std::string("pos"), value, true);
//			printMember<KDL::Rotation>(data->M, std::string("rot"), value, false);
//		}
//		virtual void handleMember(KDL::Frame* data, std::string key, std::stringstream& value) const {
//			if(key=="pos") data->p = fromString<KDL::Vector>(value);
//			if(key=="rot") data->M = fromString<KDL::Rotation>(value);
//		}
//		virtual void getMemberInfo(members_t& members) const {
//			addMember<KDL::Vector>(members, "pos", "Position [m]");
//			addMember<KDL::Rotation>(members, "rot", "Rotation");
//		}
//	};
//
//	class TwistTypeKit: public JSONTypeKit<KDL::Twist> {
//	public:
//		TwistTypeKit() : JSONTypeKit<KDL::Twist>("World::Twist")
//		{
//		}
//
//		virtual ~TwistTypeKit()
//		{
//
//		}
//
//		virtual void toString(const KDL::Twist* data, std::stringstream& value) const {
//			printMember<KDL::Vector>(data->vel, std::string("vel"), value, true);
//			printMember<KDL::Vector>(data->rot, std::string("rot"), value, false);
//		}
//		virtual void handleMember(KDL::Twist* data, std::string key, std::stringstream& value) const {
//			if(key=="vel") data->vel = fromString<KDL::Vector>(value);
//			if(key=="rot") data->rot = fromString<KDL::Vector>(value);
//		}
//		virtual void getMemberInfo(members_t& members) const {
//			addMember<KDL::Vector>(members, "vel", "Translational velocity [m/s]");
//			addMember<KDL::Vector>(members, "rot", "Rotational velocity (direction = rotation axis, length = velocity) [rad/s]");
//		}
//	};
//
//
//	class WrenchTypeKit: public JSONTypeKit<KDL::Wrench> {
//	public:
//		WrenchTypeKit() : JSONTypeKit<KDL::Wrench>("World::Wrench")
//		{
//		}
//
//		virtual ~WrenchTypeKit()
//		{
//
//		}
//
//		virtual void toString(const KDL::Wrench* data, std::stringstream& value) const {
//			printMember<KDL::Vector>(data->force, std::string("force"), value, true);
//			printMember<KDL::Vector>(data->torque, std::string("torque"), value, false);
//		}
//		virtual void handleMember(KDL::Wrench* data, std::string key, std::stringstream& value) const {
//			if(key=="force") data->force = fromString<KDL::Vector>(value);
//			if(key=="torque") data->torque = fromString<KDL::Vector>(value);
//		}
//		virtual void getMemberInfo(members_t& members) const {
//			addMember<KDL::Vector>(members, "force", "Force [N]");
//			addMember<KDL::Vector>(members, "torque", "Torque (direction = rotation axis, length = amount) [Nm]");
//		}
//	};

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
