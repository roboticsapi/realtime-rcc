/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef TYPEKITT_HPP_
#define TYPEKITT_HPP_

#include "TypeKit.hpp"
#include "NetModules.hpp"
#include "../templates/TArray.hpp"
#include <iomanip>

namespace RPI
{
	template<class T> class TypeKitT: public TypeKit
	{
	public:
		TypeKitT(const std::string& name) :
			TypeKit(name)
		{
		}

		virtual ~TypeKitT()
		{

		}

		virtual DebugModule* createDebug(const std::string& name, Net* net, int size) const
		{
			return new Debug<T> (name, net, size);
		}
	};

	template<class T> class TypeKitTStream: public TypeKitT<T>
	{
	public:
		TypeKitTStream(const std::string& name) :
			TypeKitT<T>(name)
		{
		}

		virtual ~TypeKitTStream()
		{

		}

		virtual void toString(const void* data, std::stringstream& value) const
		{
			value << std::setprecision(15) << *((T*) data);
		}
		virtual void fromString(void* data, std::stringstream& value) const
		{
			value >> *((T*) data);
		}
	};

	template<class T> class ArrayTypeKit: public TypeKitT<T>, public ArrayType
		{
		public:
			ArrayTypeKit(const std::string& name) :
				TypeKitT<T>(name)
			{
			}

			virtual ~ArrayTypeKit()
			{

			}

			virtual TypeKit* getBasicTypeKit() const {
				return TypeKits::getInstance()->getTypeKit<T>();
			}

			virtual DebugModule* createDebug(const std::string& name, Net* net, int size) const
			{
				return new Debug<Array<T> > (name, net, size);
			}


			virtual void toString(const void* data, std::stringstream& value) const
			{
				Array<T> ar = *(Array<T>*) data;

				value << "[";
				TypeKit* tk = TypeKits::getInstance()->getTypeKit<T>();

				for(int i = 0; i < ar.getSize(); ++i)
				{
					tk->toString(&ar[i], value);
					if(i < ar.getSize() - 1)
						value << ",";
				}

				value << "]";
			}
			virtual void fromString(void* data, std::stringstream& value) const
			{
				if(value.get() != '[') return;

				TypeKit* tk = TypeKits::getInstance()->getTypeKit<T>();
				std::vector<T> objects;

				bool sepfound = true;

				while(!value.eof())
				{
					// throw away whitespace or line breaks
					while (value.peek() == ' ' || value.peek() == '\r' || value.peek() == '\n' || value.peek() == ',')
					{
						sepfound = true;
						value.get();
					}

					if(value.peek() == ']') break;

					if(!sepfound)
						return;

					sepfound = false;

					T x;

					tk->fromString(&x, value);

					objects.push_back(x);
				}

				value.get();

				Array<T>& ar = *(Array<T>*) data;

				ar.resize(objects.size());

				for(int i = 0; i < ar.getSize(); ++i)
				{
					ar[i] = objects.at(i);
				}
			}

		};

	template<class T, class V> class BasicJSONTypeKit: public TypeKitT<T>, public ComplexType {
	protected:
		template <class U> void printMember(const U& data, std::string key, std::stringstream& value, bool first) const {
			if(!first) value << ",";
			value << key << ":";
			TypeKits::getInstance()->getTypeKit<U>()->toString(&data, value);
		}
		template <class U> void fromString(U& data, std::stringstream& value) const {
			TypeKits::getInstance()->getTypeKit<U>()->fromString(&data, value);
		}
		template <class U> U fromString(std::stringstream& value) const {
			U ret;
			TypeKits::getInstance()->getTypeKit<U>()->fromString(&ret, value);
			return ret;
		}

	public:
		BasicJSONTypeKit(std::string name) :
				TypeKitT<T>(name)
		{
		}

		virtual ~BasicJSONTypeKit()
		{

		}

		virtual void toString(const void* data, std::stringstream& value) const
		{
			value << "{";
			toString((T*)data, value);
			value << "}";
		}

		virtual void fromString(void* data, std::stringstream& value) const
		{
			if(value.get() != '{') return;
			V* json = beginMembers((T*)data);
			std::stringstream key;
			while(!value.eof()) {
				int ch = value.get();
				if(ch == ':') {
					while(value.peek()==' ' ||
							value.peek()=='\r' ||
							value.peek()=='\n')
						value.get();
					handleMember(json, key.str(), value);
					while(value.peek()==' ' ||
							value.peek()=='\r' ||
							value.peek()=='\n')
						value.get();
					int ch = value.get();
					if(ch == '}') break;
					if(ch == ',')
						key.str("");
					else
						return;
				} else if(ch==' ' || ch=='\n' || ch=='\r') {
				} else {
					key << (char)ch;
				}
			}
			endMembers((T*)data, json);
		}

		virtual void toString(const T* data, std::stringstream& value) const = 0;
		virtual V* beginMembers(T* data) const = 0;
		virtual void handleMember(V* data, std::string key, std::stringstream& value) const = 0;
		virtual void endMembers(T* data, V* json) const = 0;

	};

	template<class T> class JSONTypeKit: public BasicJSONTypeKit<T,T> {

	public:
		JSONTypeKit(std::string name) :
				BasicJSONTypeKit<T,T>(name)
		{
		}

		virtual ~JSONTypeKit()
		{

		}

		virtual T* beginMembers(T* data) const {
			return data;
		}
		virtual void endMembers(T* data, T* json) const {
		}
	};


}

#endif /* TYPEKITT_HPP_ */
