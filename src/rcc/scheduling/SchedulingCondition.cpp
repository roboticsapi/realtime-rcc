/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <rcc/scheduling/SchedulingCondition.hpp>
#include <rcc/Registry.hpp>
#include <rcc/Net.hpp>

namespace SchedCond
{
	SchedValue::SchedValue() :
			value(boost::indeterminate), active(false)
	{
	}

	bool SchedValue::setValue(boost::tribool newvalue)
	{
		if (newvalue == value)
			return false;

		value = newvalue;
		return true;
	}

	boost::tribool SchedValue::getValue() const
	{
		return value;
	}

	void SchedValue::setActive()
	{
		active = true;
	}

	bool SchedValue::isActive() const
	{
		return active;
	}

	SchedCondition::SchedCondition()
	{
	}

	std::string SchedTrue::toString() const
	{
		return "true";
	}

	bool SchedTrue::initialize()
	{
		return true;
	}

	boost::tribool SchedTrue::getValue() const
	{
		return true;
	}

	std::set<std::string> SchedTrue::getCondNets() const
	{
		return std::set<std::string>();
	}

	void SchedDisjunction::add(std::unique_ptr<SchedCondition> condition)
	{
		conditions.push_back(std::move(condition));
	}

	std::string SchedDisjunction::toString() const
	{
		std::string res = "";

		for (const auto& condition : conditions)
			res += " | (" + condition->toString() + ")";

		return res.substr(3);
	}

	bool SchedDisjunction::initialize()
	{
		for (const auto& condition : conditions)
			if (!condition->initialize())
				return false;

		return true;
	}

	boost::tribool SchedDisjunction::getValue() const
	{
		boost::tribool val(false);

		for (const auto& condition : conditions)
		{
			val = val || condition->getValue();

			// we can stop evaluation iff we have the value true already
			if (val)
				return true;
		}

		return val;
	}

	std::set<std::string> SchedDisjunction::getCondNets() const
	{
		std::set<std::string> result;

		for (const auto& cond : conditions)
		{
			const auto& conres = cond->getCondNets();
			result.insert(conres.begin(), conres.end());
		}

		return result;
	}

	void SchedConjunction::add(std::unique_ptr<SchedCondition> condition)
	{
		conditions.push_back(std::move(condition));
	}

	std::string SchedConjunction::toString() const
	{
		std::string res;

		for (const auto& condition : conditions)
			res += " & (" + condition->toString() + ")";

		return res.substr(3);
	}

	bool SchedConjunction::initialize()
	{
		for (const auto& condition : conditions)
			if (!condition->initialize())
				return false;

		return true;
	}

	boost::tribool SchedConjunction::getValue() const
	{
		boost::tribool val(true);

		for (const auto& condition : conditions)
		{
			val = val && condition->getValue();

			// we can stop evaluation iff we have the value false already
			if (!val)
				return false;
		}

		return val;
	}

	std::set<std::string> SchedConjunction::getCondNets() const
	{
		std::set<std::string> result;

		for (const auto& cond : conditions)
		{
			const auto& conres = cond->getCondNets();
			result.insert(conres.begin(), conres.end());
		}

		return result;
	}

	SchedNegation::SchedNegation(std::unique_ptr<SchedCondition> condition)
	{
		this->condition = std::move(condition);
	}

	std::string SchedNegation::toString() const
	{
		return "!" + condition->toString();
	}

	bool SchedNegation::initialize()
	{
		return condition->initialize();
	}

	boost::tribool SchedNegation::getValue() const
	{
		return !condition->getValue();
	}

	std::set<std::string> SchedNegation::getCondNets() const
	{
		return condition->getCondNets();
	}

	SchedLiteral::SchedLiteral(const std::string& net, const std::string& port)
	{
		this->net = net;
		this->port = port;
		this->value = 0;
	}

	std::string SchedLiteral::toString() const
	{
		return net + "." + port;
	}

	// check whether net and port are valid
	bool SchedLiteral::initialize()
	{
		auto netptr = RPI::Registry::getRegistry()->getNet(net);

		if (!netptr)
			return false;

		value = netptr->getSchedValue(port);

		if (!value)
			return false;

		value->setActive();
		return true;
	}

	boost::tribool SchedLiteral::getValue() const
	{
		if (value)
		{
			return value->getValue();
		}

		return boost::indeterminate;
	}

	std::set<std::string> SchedLiteral::getCondNets() const
	{
		std::set<std::string> res;
		res.insert(net);

		return res;
	}

} /* namespace SchedCond */
