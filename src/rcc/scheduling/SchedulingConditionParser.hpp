/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SCHEDULINGCONDITIONPARSER_HPP_
#define SCHEDULINGCONDITIONPARSER_HPP_

#include <string>
#include <memory>
#include "SchedulingCondition.hpp"
#include "SchedulingConditionP.hpp"

namespace SchedCond
{

	class SchedulingConditionParser
	{
	public:
		SchedulingConditionParser();
		virtual ~SchedulingConditionParser();

		static std::unique_ptr<SchedCondition> parse(const std::string& input);
	private:
		static std::unique_ptr<SchedCondition> parseDisjunction(const CDisjunction& dis);
		static std::unique_ptr<SchedCondition> parseConjunction(const CConjunction& con);
		static std::unique_ptr<SchedCondition> parseNegation(const CNegation& neg);
		static std::unique_ptr<SchedCondition> parseBoolExpr(const CBoolExpr& boolexpr);
	};

} /* namespace SchedCond */

#endif /* SCHEDULINGCONDITIONPARSER_HPP_ */
