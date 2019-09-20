/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "DIOProtocolP.hpp"

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

namespace DIOProtocolP
{
	DIOPException::DIOPException(const std::string& cause)
	{
		this->cause = cause;
	}

	std::string DIOPException::getCause() const
	{
		return cause;
	}

	std::string DIOParameterList::toStringNoParen() const
	{
		std::string res;

		bool first = true;
		for (const auto& param : parameters)
		{
			if (first)
				first = false;
			else
				res += ",";

			res += param->toString();
		}

		return res;
	}

	std::string DIOParameterList::toString() const
	{
		return "[" + toStringNoParen() + "]";
	}

	void DIOParameterList::addParameter(std::unique_ptr<DIOParameter> parameter)
	{
		parameters.push_back(std::move(parameter));
	}

	size_t DIOParameterList::getSize() const
	{
		return parameters.size();
	}

	DIOParameterList::dioparameterlist_t::const_iterator DIOParameterList::begin() const
	{
		return parameters.begin();
	}

	DIOParameterList::dioparameterlist_t::const_iterator DIOParameterList::end() const
	{
		return parameters.end();
	}

	std::string DIOParameterMap::toString() const
	{
		std::string res;

		res = "{";

		bool first = true;
		for (const auto& mentry : maps)
		{
			if (first)
				first = false;
			else
				res += ",";

			// check whether key is an identifier, otherwise escape as string
			boost::regex expr("[a-zA-Z_][a-zA-Z_0-9]*");
			if (boost::regex_match(mentry.first, expr))
				res += mentry.first;
			else
				res += DIOString(mentry.first).toString();

			res += ":" + mentry.second->toString();
		}

		res += "}";

		return res;
	}

	void DIOParameterMap::addParameter(const std::string& key, std::unique_ptr<DIOParameter> parameter)
	{
		maps[key] = std::move(parameter);
	}

	DIOParameterMap::dioparametermap_t::const_iterator DIOParameterMap::begin() const
	{
		return maps.begin();
	}

	DIOParameterMap::dioparametermap_t::const_iterator DIOParameterMap::end() const
	{
		return maps.end();
	}

	DIOString::DIOString(const std::string& content, bool unescapestr)
	{
		if (unescapestr)
			this->content = unescape(content);
		else
			this->content = content;
	}

	DIOString::DIOString(wchar_t* content)
	{
		this->content = unescape(tostr(content));

	}

	std::string DIOString::toString() const
	{
		return escape(content);
	}

	std::string DIOString::unescape(const std::string& input)
	{
		// input too short - should not happen

		if (input.length() < 2)
			return input;
		auto shortstr = input.substr(1, input.length() - 2);

		boost::algorithm::replace_all(shortstr, "\\\"", "\"");
		boost::algorithm::replace_all(shortstr, "\\\\", "\\");

		return shortstr;
	}

	std::string DIOString::escape(std::string input)
	{
		boost::algorithm::replace_all(input, "\\", "\\\\");
		boost::algorithm::replace_all(input, "\"", "\\\"");

		return "\"" + input + "\"";
	}

	std::string DIOString::tostr(wchar_t* wc)
	{
		size_t newlen = wcslen(wc) * 2 + 1;

		char* mbs = new char[newlen];

		wcstombs(mbs, wc, newlen);
		std::string res(mbs);

		delete[] mbs;

		return res;
	}

	std::string DIOString::getString() const
	{
		return content;
	}

	DIOInteger::DIOInteger(wchar_t* content)
	{
		try
		{
			this->content = boost::lexical_cast<int>(DIOString::tostr(content));
		} catch (boost::bad_lexical_cast&)
		{
			this->content = 0;
		}
	}

	std::string DIOInteger::toString() const
	{
		return std::to_string(content);
	}

	int DIOInteger::getInt(int def) const
	{
		return content;
	}

	DIOFloat::DIOFloat(wchar_t* content)
	{
		try
		{
			this->content = boost::lexical_cast<double>(DIOString::tostr(content));
		} catch (boost::bad_lexical_cast&)
		{
			this->content = 0;
		}
	}

	std::string DIOFloat::toString() const
	{
		return std::to_string(content);
	}

	double DIOFloat::getDouble(double def) const
	{
		return content;
	}

	DIOCommand::DIOCommand() :
			tag(""), command("")
	{

	}

	DIOCommand::DIOCommand(const std::string& tag, const std::string& command) :
			tag(tag), command(command)
	{

	}

	std::string DIOCommand::toString() const
	{
		std::string res = tag + "=" + command + "(";

		res += parameters.toStringNoParen();

		res += ")";
		return res;
	}
}
