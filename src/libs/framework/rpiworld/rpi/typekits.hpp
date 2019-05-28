/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "kdl/frames.hpp"
#include <rcc/TypeKitT.hpp>

using namespace std;

namespace World
{
	using namespace RPI;

	class VectorTypeKit: public JSONTypeKit<KDL::Vector> {
	public:
		VectorTypeKit() : JSONTypeKit<KDL::Vector>("World::Vector")
		{
		}

		virtual ~VectorTypeKit()
		{

		}

		virtual void toString(const KDL::Vector* data, std::stringstream& value) const {
			printMember<double>(data->x(), std::string("x"), value, true);
			printMember<double>(data->y(), std::string("y"), value, false);
			printMember<double>(data->z(), std::string("z"), value, false);
		}
		virtual void handleMember(KDL::Vector* data, std::string key, std::stringstream& value) const {
			if(key=="x") data->x(fromString<double>(value));
			if(key=="y") data->y(fromString<double>(value));
			if(key=="z") data->z(fromString<double>(value));
		}
		virtual void getMemberInfo(members_t& members) const {
			addMember<double>(members, "x", "X position [m]");
			addMember<double>(members, "y", "Y position [m]");
			addMember<double>(members, "z", "Z position [m]");
		}
	};

	class RotationTypeKit: public BasicJSONTypeKit<KDL::Rotation,double> {
	public:
		RotationTypeKit() : BasicJSONTypeKit<KDL::Rotation,double>("World::Rotation")
		{
		}

		virtual ~RotationTypeKit()
		{

		}

		virtual void toString(const KDL::Rotation* data, std::stringstream& value) const {
			double a, b, c; data->GetRPY(c, b, a);
			printMember<double>(a, "a", value, true);
			printMember<double>(b, "b", value, false);
			printMember<double>(c, "c", value, false);
		}
		virtual double* beginMembers(KDL::Rotation* data) const {
			double* ret = new double[3];
			ret[0] = 0; ret[1] = 0; ret[2] = 0;
			return ret;
		}
		virtual void handleMember(double* data, std::string key, std::stringstream& value) const {
			if(key=="a") data[0] = fromString<double>(value);
			if(key=="b") data[1] = fromString<double>(value);
			if(key=="c") data[2] = fromString<double>(value);
		}
		virtual void endMembers(KDL::Rotation* data, double* json) const {
			*data = KDL::Rotation::RPY(json[2], json[1], json[0]);
			delete[] json;
		}
		virtual void getMemberInfo(members_t& members) const {
			addMember<double>(members, "a", "A rotation (rad, around Z)");
			addMember<double>(members, "b", "B rotation (rad, around Y)");
			addMember<double>(members, "c", "C rotation (rad, around X)");
		}
	};

	class FrameTypeKit: public JSONTypeKit<KDL::Frame> {
	public:
		FrameTypeKit() : JSONTypeKit<KDL::Frame>("World::Frame")
		{
		}

		virtual ~FrameTypeKit()
		{

		}

		virtual void toString(const KDL::Frame* data, std::stringstream& value) const {
			printMember<KDL::Vector>(data->p, std::string("pos"), value, true);
			printMember<KDL::Rotation>(data->M, std::string("rot"), value, false);
		}
		virtual void handleMember(KDL::Frame* data, std::string key, std::stringstream& value) const {
			if(key=="pos") data->p = fromString<KDL::Vector>(value);
			if(key=="rot") data->M = fromString<KDL::Rotation>(value);
		}
		virtual void getMemberInfo(members_t& members) const {
			addMember<KDL::Vector>(members, "pos", "Position [m]");
			addMember<KDL::Rotation>(members, "rot", "Rotation");
		}
	};

	class TwistTypeKit: public JSONTypeKit<KDL::Twist> {
	public:
		TwistTypeKit() : JSONTypeKit<KDL::Twist>("World::Twist")
		{
		}

		virtual ~TwistTypeKit()
		{

		}

		virtual void toString(const KDL::Twist* data, std::stringstream& value) const {
			printMember<KDL::Vector>(data->vel, std::string("vel"), value, true);
			printMember<KDL::Vector>(data->rot, std::string("rot"), value, false);
		}
		virtual void handleMember(KDL::Twist* data, std::string key, std::stringstream& value) const {
			if(key=="vel") data->vel = fromString<KDL::Vector>(value);
			if(key=="rot") data->rot = fromString<KDL::Vector>(value);
		}
		virtual void getMemberInfo(members_t& members) const {
			addMember<KDL::Vector>(members, "vel", "Translational velocity [m/s]");
			addMember<KDL::Vector>(members, "rot", "Rotational velocity (direction = rotation axis, length = velocity) [rad/s]");
		}
	};


	class WrenchTypeKit: public JSONTypeKit<KDL::Wrench> {
	public:
		WrenchTypeKit() : JSONTypeKit<KDL::Wrench>("World::Wrench")
		{
		}

		virtual ~WrenchTypeKit()
		{

		}

		virtual void toString(const KDL::Wrench* data, std::stringstream& value) const {
			printMember<KDL::Vector>(data->force, std::string("force"), value, true);
			printMember<KDL::Vector>(data->torque, std::string("torque"), value, false);
		}
		virtual void handleMember(KDL::Wrench* data, std::string key, std::stringstream& value) const {
			if(key=="force") data->force = fromString<KDL::Vector>(value);
			if(key=="torque") data->torque = fromString<KDL::Vector>(value);
		}
		virtual void getMemberInfo(members_t& members) const {
			addMember<KDL::Vector>(members, "force", "Force [N]");
			addMember<KDL::Vector>(members, "torque", "Torque (direction = rotation axis, length = amount) [Nm]");
		}
	};

}
