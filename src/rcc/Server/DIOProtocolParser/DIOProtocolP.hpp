/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DIOPROTOCOLP_HPP_
#define DIOPROTOCOLP_HPP_

#include <cstdlib>
#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <limits.h>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>

namespace DIOProtocolP
{
	class DIOString;
	class DIOInteger;
	class DIOFloat;
	class DIOParameterList;
	class DIOParameterMap;

	class DIOPException: public std::exception
	{
	private:
		std::string cause;

	public:
		DIOPException(const std::string& cause);
		virtual ~DIOPException() throw () { }

		std::string getCause() const;

	};

	class DIOParameter
	{
	public:
		virtual ~DIOParameter()
		{
		}
		virtual std::string toString() const = 0;

		template<class T> const T* getParameterT() const
		{
			const T* res = dynamic_cast<const T*>(this);
			if (res)
				return res;

			throw DIOPException("Wrong parameter type");
		}
	};

	class DIOParameterList: public DIOParameter
	{
	public:
		typedef std::vector<std::unique_ptr<DIOParameter> > dioparameterlist_t;
		virtual ~DIOParameterList()
		{
		}

		virtual std::string toString() const;
		virtual std::string toStringNoParen() const;

		void addParameter(std::unique_ptr<DIOParameter> parameter);

		size_t getSize() const;

		dioparameterlist_t::const_iterator begin() const;
		dioparameterlist_t::const_iterator end() const;

		template<class T> T* getParameter(int index) const
		{
			if (index < 0 || index >= parameters.size())
				throw DIOPException("Parameter index out of bounds");

			T* res = dynamic_cast<T*>(parameters.at(index).get());

			if (!res)
				throw DIOPException("Parameter type does not match");

			return res;
		}
	private:
		dioparameterlist_t parameters;

	};

	class DIOParameterMap: public DIOParameter
	{
	public:
		typedef std::map<std::string, std::unique_ptr<DIOParameter> > dioparametermap_t;
		virtual ~DIOParameterMap()
		{
		}

		virtual std::string toString() const;

		void addParameter(const std::string& key, std::unique_ptr<DIOParameter> parameter);

		dioparametermap_t::const_iterator begin() const;
		dioparametermap_t::const_iterator end() const;

		template<class T> T* getParameter(const std::string& key) const
		{
			dioparametermap_t::const_iterator it = maps.find(key);
			if (it == maps.end())
				throw DIOPException("Key not found in map");

			T* res = dynamic_cast<T*>(*it);

			if (!res)
				throw DIOPException("Parameter type does not match");

			return res;
		}
	private:
		dioparametermap_t maps;

	};

	class DIOLiteral: public DIOParameter
	{
	public:
		virtual ~DIOLiteral()
		{
		}
		virtual std::string toString() const = 0;
	};

	class DIOString: public DIOLiteral
	{
	public:
		virtual ~DIOString()
		{
		}

		DIOString(wchar_t* content);
		DIOString(const std::string& content, bool unescape = false);

		virtual std::string toString() const;

		virtual std::string getString() const;

		static std::string unescape(const std::string&);
		static std::string escape(std::string);
		static std::string tostr(wchar_t*);
	private:
		std::string content;
	};

	class DIOInteger: public DIOLiteral
	{
	public:
		virtual ~DIOInteger()
		{
		}

		DIOInteger(wchar_t* content);
		DIOInteger(int content);

		virtual std::string toString() const;

		virtual int getInt(int def = 0) const;
	private:
		int content;
	};

	class DIOFloat: public DIOLiteral
	{
	public:
		virtual ~DIOFloat()
		{
		}

		DIOFloat(wchar_t* content);
		DIOFloat(double content);

		virtual std::string toString() const;

		virtual double getDouble(double def = 0.0) const;
	private:
		double content;
	};

	class DIOCommand
	{
	public:
		DIOCommand();
		DIOCommand(const std::string& tag, const std::string& command);

		std::string tag;
		std::string command;

		std::string toString() const;

		template<class T> T* getParameter(int index) const
		{
			return parameters.getParameter<T>(index);
		}

		DIOParameterList parameters;
	};
}

#endif /* DIOPROTOCOLP_HPP_ */
