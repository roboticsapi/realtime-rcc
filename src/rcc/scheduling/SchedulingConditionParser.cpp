/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "SchedulingConditionParser.hpp"

#include "SchedulingConditionP_inc.h"
#include <iostream>

namespace SchedCond
{

	SchedulingConditionParser::SchedulingConditionParser()
	{
		// TODO Auto-generated constructor stub

	}

	SchedulingConditionParser::~SchedulingConditionParser()
	{
		// TODO Auto-generated destructor stub
	}

	std::unique_ptr<SchedCondition> SchedulingConditionParser::parse(const std::string& input)
	{
		Scanner scanner((unsigned char*) input.c_str(), input.length());
		Parser parser(&scanner);
		parser.Parse();
		if (parser.errors->count > 0)
		{
			// errors = parser.errors->getMessage();
			return 0;
		}

		//std::cout << parser.cond->toString(0) << std::endl;

		auto res = parseDisjunction(*parser.cond);

		//std::cout << res->toString() << std::endl;

		return res;
	}

	std::unique_ptr<SchedCondition> SchedulingConditionParser::parseDisjunction(const CDisjunction& dis)
	{
		if (dis.conjunctions.size() > 1)
		{
			std::unique_ptr<SchedDisjunction> ndis(new SchedDisjunction());

			for (const auto& con : dis.conjunctions)
			{
				ndis->add(parseConjunction(*con));
			}

			return std::move(ndis);
		} else if (dis.conjunctions.size() == 1)
		{
			return parseConjunction(*dis.conjunctions.front());
		}
		return 0;
	}

	std::unique_ptr<SchedCondition> SchedulingConditionParser::parseConjunction(const CConjunction& con)
	{
		if (con.negations.size() > 1)
		{
			std::unique_ptr<SchedConjunction> ncon(new SchedConjunction());

			for (const auto& neg : con.negations)
			{
				ncon->add(parseNegation(*neg));
			}

			return std::move(ncon);
		} else if (con.negations.size() == 1)
		{
			return parseNegation(*con.negations.front());
		}
		return 0;

	}

	std::unique_ptr<SchedCondition> SchedulingConditionParser::parseNegation(const CNegation& neg)
	{
		if (neg.negated)
		{
			return std::unique_ptr<SchedNegation>(new SchedNegation(parseBoolExpr(*neg.boolex)));
		} else
		{
			return parseBoolExpr(*neg.boolex);
		}
	}

	std::unique_ptr<SchedCondition> SchedulingConditionParser::parseBoolExpr(const CBoolExpr& boolexpr)
	{
		if (boolexpr.subexpr)
		{
			return parseDisjunction(*boolexpr.subexpr);
		} else if(boolexpr.isTrue)
		{
			return std::unique_ptr<SchedTrue>(new SchedTrue());
		} else
		{
			return std::unique_ptr<SchedLiteral>(new SchedLiteral(boolexpr.net, boolexpr.port));
		}

	}

} /* namespace SchedCond */
