/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DIONET_HPP_
#define DIONET_HPP_

#include <cstdlib>
#include <string>
#include <map>
#include <sstream>
#include <limits.h>
#include <memory>
#include <boost/algorithm/string.hpp>

namespace DirectIONet
{
	class AstPortRef;
	class AstPrimitive;

	typedef std::shared_ptr<AstPrimitive> AstPrimitivePtr;
	typedef std::shared_ptr<AstPortRef> AstPortRefPtr;

	 class AstFragment {
	 public:
		std::map<std::string, AstPrimitivePtr> primitives;
		std::map<std::string, AstPortRefPtr> outPorts;

		static AstFragment newFragment() { AstFragment ret; return ret; }

		void putPrimitive(std::string name, AstPrimitivePtr value) { primitives[name] = value; }
		void putOutPort(std::string name, AstPortRefPtr value) { outPorts[name] = value; }
	};

	class AstPrimitive {
	public:
		std::map<std::string, std::string> parameters;
		std::map<std::string, AstPortRefPtr> inPorts;

		enum AstPrimitiveType { PrimReference, PrimDefinition, PrimFragment };

		std::string name;
		std::string type;
		AstFragment fragment;

		AstPrimitiveType primtype;

		static AstPrimitive newReference(std::string name) { AstPrimitive ret; ret.name = name; ret.primtype = PrimReference; return ret; }
		static AstPrimitive newDefinition(std::string type) { AstPrimitive ret; ret.type = type; ret.primtype = PrimDefinition; return ret; }
		static AstPrimitive newFragment(AstFragment fragment) { AstPrimitive ret; ret.fragment = fragment; ret.primtype = PrimFragment; return ret; }

		void putParameter(std::string name, std::string value) { parameters[name] = value; }
		void putInPort(std::string name, AstPortRefPtr port) { inPorts[name] = port; }
	};


	class AstPortRef {
	public:
		AstPrimitive prim;
		std::string name;
		double debug;
		static AstPortRef newPortRef(AstPrimitive prim, std::string name, double debug) { AstPortRef ret; ret.prim = prim; ret.name = name; ret.debug = debug; return ret; }
	};

	class AstPValue {
	public:
		std::string str;
		AstPortRef portRef;
		bool _isString;

		static AstPValue newString(std::string value) { AstPValue ret; ret.str = value; ret._isString = true; return ret; }
		static AstPValue newPortRef(AstPortRef portRef) { AstPValue ret; ret.portRef = portRef; ret._isString = false; return ret; }

		bool isString()
		{
			return _isString;
		}
	};

	class AstFValue {
	public:
		AstPrimitive prim ;
		AstPortRef portRef;
		bool _isPrim;

		static AstFValue newPrimitive(AstPrimitive prim) { AstFValue ret; ret.prim = prim; ret._isPrim = true; return ret; }
		static AstFValue newPortRef(AstPortRef portRef) { AstFValue ret; ret.portRef = portRef; ret._isPrim = false; return ret; }

		bool isPrim() {
			return _isPrim;
		}
	};

	inline std::string tostr(wchar_t* wc)
	{
		size_t newlen = wcslen(wc) * 2 + 1;

		char* mbs = new char[newlen];

		wcstombs(mbs, wc, newlen);
		std::string res(mbs);

		delete[] mbs;

		return res;
	}

	/**
	 * Remove enclosing ' characters and undo escaping
	 */
	inline std::string unescape_dionet(std::string input)
	{
		// input too short - should not happen

		if(input.length() < 2) return input;
		auto shortstr = input.substr(1, input.length() - 2);

		boost::algorithm::replace_all(shortstr, "\\'", "'");
		boost::algorithm::replace_all(shortstr, "\\\\", "\\");

		return shortstr;
	}

}


#endif /* DIONET_HPP_ */
