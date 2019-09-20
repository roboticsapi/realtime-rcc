/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SCHEDULINGCONDITIONP_HPP_
#define SCHEDULINGCONDITIONP_HPP_

#include <string>
#include <sstream>
#include <memory>
#include <list>

namespace SchedCond
{
	class CDisjunction;
	class CConjunction;
	class CNegation;
	class CBoolExpr;

	class CDisjunction
	{
		friend class SchedulingConditionParser;
	public:
		CDisjunction();
		virtual ~CDisjunction();

		void add(std::unique_ptr<CConjunction>);

		std::string toString(int depth) const;
	private:
		std::list<std::unique_ptr<CConjunction> > conjunctions;
	};

	class CConjunction
	{
		friend class SchedulingConditionParser;
	public:
		CConjunction();
		virtual ~CConjunction();

		void add(std::unique_ptr<CNegation>);

		std::string toString(int depth) const;
	private:
		std::list<std::unique_ptr<CNegation> > negations;
	};

	class CNegation
	{
		friend class SchedulingConditionParser;
	public:
		CNegation();
		virtual ~CNegation();

		void add(std::unique_ptr<CBoolExpr>);
		void negate();

		std::string toString(int depth) const;
	private:
		std::unique_ptr<CBoolExpr> boolex;
		bool negated;
	};

	class CBoolExpr
	{
		friend class SchedulingConditionParser;
	public:
		CBoolExpr();
		virtual ~CBoolExpr();

		void add(std::unique_ptr<CDisjunction>);
		void setNet(const std::string&);
		void setPort(const std::string&);
		void setTrue();

		std::string toString(int depth) const;
	private:
		std::unique_ptr<CDisjunction> subexpr;
		std::string net;
		std::string port;
		bool isTrue;
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


} /* namespace SchedCond */

#endif /* SCHEDULINGCONDITIONP_HPP_ */
