/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef NETCONDITION_HPP_
#define NETCONDITION_HPP_

#include "SynchronizationRuleFwd.hpp"

#include <memory>
#include <set>
#include <list>
#include <map>
#include <vector>
#include "scheduling/SchedulingConditionFwd.hpp"
#include "NetExecutorFwd.hpp"
#include "NetFwd.hpp"

namespace RPI
{
	class SynchronizationRuleException: public std::exception
	{

	};

	/**
	 * Rule for synchronization of multiple primitive nets
	 *
	 * The implementation of this class should be real-time capable,
	 * because the condition is evaluated and used in RT context
	 */
	class SynchronizationRule
	{
	public:
		SynchronizationRule(Sync_ID_t id, std::unique_ptr<SchedCond::SchedCondition>,
				std::set<std::shared_ptr<Net>> stopNets, std::set<std::shared_ptr<Net>> cancelNets,
				std::set<std::shared_ptr<Net>> startNets, const std::string& description);
		virtual ~SynchronizationRule();

		/**
		 * Check whether this condition is currently true
		 */
		bool CheckCondition() const;

		/**
		 * Checks whether this condition is still alive, i.e. not all nets in the
		 * condition are terminated already
		 */
		bool CheckConditionAlive() const;

		/**
		 * Get all nets which should be stopped when this condition is
		 * triggered.
		 */
		const std::set<std::shared_ptr<Net>>& getStopNets() const;

		/**
		 * Get all nets which should be cancelled when this condition is
		 * triggered.
		 */
		const std::set<std::shared_ptr<Net>>& getCancelNets() const;

		/**
		 * Get all nets which are started by this condition
		 */
		const std::set<std::shared_ptr<Net>>& getStartNets() const;

		/**
		 * Gets all nets contained in the condition
		 */
		const std::set<std::shared_ptr<Net>>& getConditionNets() const;

		const std::string& getConditionName() const;
		const std::set<std::string>& getStartNames() const;
		const std::set<std::string>& getCancelNames() const;
		const std::set<std::string>& getStopNames() const;

		bool hasStopNet(const std::string& net) const;

		/**
		 * Set condition as fired
		 */
		void fired();

		bool isFired() const;

		void invalidate();

		const std::set<std::string>& getStartResources() const;
		const std::set<std::string>& getStopResources() const;

		const Sync_ID_t getSyncID() const;
	private:
		bool conditionFired;

		std::unique_ptr<SchedCond::SchedCondition> condition;
		std::set<std::shared_ptr<Net>> stopNets;
		std::set<std::shared_ptr<Net>> cancelNets;
		std::set<std::shared_ptr<Net>> startNets;
		std::set<std::shared_ptr<Net>> conditionNets;
		// resources are required by start nets, and resources owned by stop nets
		std::set<std::string> startResources, stopResources;

		std::set<std::string> startNames, stopNames, cancelNames;
		std::string conditionName;

		/**
		 * Number of synchronization rule
		 */
		Sync_ID_t id;

		/**
		 * Description of synchronization rule
		 */
		std::string description;
	};

} /* namespace RPI */
#endif /* NETCONDITION_HPP_ */
