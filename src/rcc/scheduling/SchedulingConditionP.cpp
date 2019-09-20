/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "SchedulingConditionP.hpp"

namespace SchedCond
{
	std::string genSpace(int depth)
	{
		std::string ret;

		for (int i = 0; i < depth; ++i)
			ret += " ";

		return ret;
	}

	CDisjunction::CDisjunction()
	{
	}

	CDisjunction::~CDisjunction()
	{

	}

	void CDisjunction::add(std::unique_ptr<CConjunction> conjunction)
	{
		conjunctions.push_back(std::move(conjunction));
	}

	std::string CDisjunction::toString(int depth) const
	{
		std::string ret;

		ret = genSpace(depth);
		ret += "Disjunction:\r\n";

		for (const auto& conjunction : conjunctions)
		{
			ret += conjunction->toString(depth + 1);
		}

		return ret;
	}

	CConjunction::CConjunction()
	{
	}

	CConjunction::~CConjunction()
	{

	}

	void CConjunction::add(std::unique_ptr<CNegation> negation)
	{
		negations.push_back(std::move(negation));
	}

	std::string CConjunction::toString(int depth) const
	{
		std::string ret;

		ret = genSpace(depth);
		ret += "Conjunction:\r\n";

		for (const auto& negation : negations)
		{
			ret += negation->toString(depth + 1);
		}

		return ret;
	}

	CNegation::CNegation()
	{
		negated = false;
		boolex = 0;
	}

	CNegation::~CNegation()
	{

	}

	void CNegation::add(std::unique_ptr<CBoolExpr> boolex)
	{
		this->boolex = std::move(boolex);
	}

	void CNegation::negate()
	{
		negated = true;
	}

	std::string CNegation::toString(int depth) const
	{
		std::string ret;

		ret = genSpace(depth);
		if (negated)
			ret += "Negation: true\r\n";
		else
			ret += "Negation: false\r\n";
		//ret += negated ? "t " : "f ";
		if (boolex)
			ret += boolex->toString(depth + 1);
		else
			ret += genSpace(depth + 1) + "0\r\n";

		return ret;
	}

	CBoolExpr::CBoolExpr()
	{
		subexpr = 0;
		isTrue = false;
	}

	CBoolExpr::~CBoolExpr()
	{

	}

	void CBoolExpr::add(std::unique_ptr<CDisjunction> subexpr)
	{
		this->subexpr = std::move(subexpr);
	}

	void CBoolExpr::setNet(const std::string& net)
	{
		this->net = net;
	}

	void CBoolExpr::setPort(const std::string& port)
	{
		this->port = port;
	}

	std::string CBoolExpr::toString(int depth) const
	{
		std::string ret;

		ret = genSpace(depth);
		ret += "BoolExpr\r\n";

		if (subexpr)
		{
			ret += subexpr->toString(depth + 1);
		} else
		{
			ret += genSpace(depth + 1) + net + "." + port + "\r\n";
		}

		return ret;
	}

	void CBoolExpr::setTrue()
	{
		isTrue = true;
	}

} /* namespace SchedCond */
