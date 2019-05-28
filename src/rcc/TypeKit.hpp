/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef TYPEKIT_HPP_
#define TYPEKIT_HPP_

#include <string>
#include <map>
#include <list>
#include <typeinfo>
#include <rtt/rtt-config.h>
#include <typeinfo>

#include "NetModulesFwd.hpp"
#include "NetFwd.hpp"

namespace RPI
{
	/**
	 * \brief Type information class
	 */
	class RTT_EXPORT TypeKit
	{
	private:
		std::string humanname;
	public:
		/**
		 * \brief creates new TypeKit
		 * \poaram humanname Human readable name of type
		 */
		TypeKit(const std::string& humanname);
		virtual ~TypeKit();

		virtual DebugModule* createDebug(const std::string& name, Net* net,
				int size) const = 0;
		virtual void toString(const void* data, std::stringstream& value) const = 0;
		virtual void fromString(void* data, std::stringstream& value) const = 0;

		std::string toString(const void* data) const;
		void fromString(void* data, const std::string& value) const;

		std::string getHumanName() const;
	};

	typedef std::map<std::string, TypeKit*> typekits_t;


	/**
	 * \brief Collection of TypeKit instances
	 */
	class RTT_EXPORT TypeKits
	{
	private:
		typekits_t typeKits;
		static TypeKits* theInstance;
		TypeKits();
	public:
		static TypeKits* getInstance();
		/**
		 * Returns typekit for specified type. It is guaranteed that a typekit is returned.
		 * If no typekit for given type exists, a generic void typekit (without any conversion
		 * capabilities) will be returned
		 */
		TypeKit* getTypeKit(const std::type_info& type);

		template<class T>
		TypeKit* getTypeKit()
		{
			return getTypeKit(typeid(T));
		}

		void registerTypeKit(const std::type_info& type, TypeKit* typekit);
		void unregisterTypeKit(const std::type_info& type);

		std::list<TypeKit*> getTypeKits();

	};


	class RTT_EXPORT ArrayType
	{
	public:
		virtual ~ArrayType() { };
		virtual TypeKit* getBasicTypeKit() const = 0;
	};

	class RTT_EXPORT ComplexType
	{
	public:
		virtual ~ComplexType() { };

		struct MemberInfo {
			std::string name;
			TypeKit* type;
			std::string description;
			MemberInfo(std::string name, TypeKit* type, std::string description): name(name), type(type), description(description) {
			}
		};

		typedef std::list<MemberInfo> members_t;


	protected:
		template <class U> void addMember(members_t& members, std::string name, std::string description) const {
			members.push_back(MemberInfo(name, TypeKits::getInstance()->getTypeKit<U>(), description));
		}

		virtual void getMemberInfo(members_t& ret) const = 0;

	public:
		virtual members_t getMemberTypeKits() {
			members_t ret;
			getMemberInfo(ret);
			return ret;
		}

	};

}
#endif /* TYPEKIT_HPP_ */

