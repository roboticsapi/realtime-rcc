/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SCHEDULINGCONDITION_HPP_
#define SCHEDULINGCONDITION_HPP_

#include "SchedulingConditionFwd.hpp"

#include <list>
#include <string>
#include <memory>
#include <set>
#include <boost/logic/tribool.hpp>

namespace SchedCond
{
	class SchedValue
	{
	public:
		SchedValue();
		virtual ~SchedValue()
		{
		}

		/**
		 * set tri-boolean value
		 *
		 * @return true, if value has changed, false otherwise
		 */
		bool setValue(boost::tribool value);

		/**
		 * read current value
		 */
		boost::tribool getValue() const;

		/**
		 * Activate this SchedValue
		 *
		 * A sched value is active, if at least one SchedCondition is connected
		 */
		void setActive();

		/**
		 * Query active state of SchedValue
		 */
		bool isActive() const;
	private:
		boost::tribool value;
		bool active;
	};

	class SchedCondition
	{
	public:
		SchedCondition();
		virtual ~SchedCondition()
		{
		}

		/**
		 * Initialize this SchedCondition
		 *
		 * During initialization, all connections to nets are created
		 */
		virtual bool initialize() = 0;

		virtual boost::tribool getValue() const = 0;

		virtual std::set<std::string> getCondNets() const = 0;

		virtual std::string toString() const = 0;
	};

	class SchedTrue: public SchedCondition
	{
	public:
		virtual ~SchedTrue()
		{
		}

		virtual boost::tribool getValue() const;
		virtual bool initialize();
		virtual std::set<std::string> getCondNets() const;

		virtual std::string toString() const;
	};

	class SchedDisjunction: public SchedCondition
	{
	public:
		~SchedDisjunction()
		{
		}

		void add(std::unique_ptr<SchedCondition> condition);

		virtual boost::tribool getValue() const;
		virtual bool initialize();
		virtual std::set<std::string> getCondNets() const;

		virtual std::string toString() const;
	private:
		std::list<std::unique_ptr<SchedCondition> > conditions;
	};

	class SchedConjunction: public SchedCondition
	{
	public:
		~SchedConjunction()
		{
		}

		void add(std::unique_ptr<SchedCondition> condition);

		virtual boost::tribool getValue() const;
		virtual bool initialize();
		virtual std::set<std::string> getCondNets() const;

		virtual std::string toString() const;
	private:
		std::list<std::unique_ptr<SchedCondition> > conditions;
	};

	class SchedNegation: public SchedCondition
	{
	public:
		~SchedNegation()
		{
		}

		SchedNegation(std::unique_ptr<SchedCondition>);

		virtual boost::tribool getValue() const;
		virtual bool initialize();
		virtual std::set<std::string> getCondNets() const;

		virtual std::string toString() const;

	private:
		std::unique_ptr<SchedCondition> condition;
	};

	class SchedLiteral: public SchedCondition
	{
	public:
		~SchedLiteral()
		{
		}

		SchedLiteral(const std::string& net, const std::string& port);

		virtual bool initialize();
		virtual boost::tribool getValue() const;
		virtual std::set<std::string> getCondNets() const;

		virtual std::string toString() const;
	private:
		std::string net, port;
		std::shared_ptr<SchedValue> value;
	};

} /* namespace SchedCond */

#endif /* SCHEDULINGCONDITION_HPP_ */
