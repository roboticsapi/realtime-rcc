/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "SynchronizationRule.hpp"

#include "Net.hpp"
#include "NetExecutor.hpp"
#include "Registry.hpp"

#include "scheduling/SchedulingCondition.hpp"

namespace RPI
{
	using namespace std;

	SynchronizationRule::SynchronizationRule(Sync_ID_t id, std::unique_ptr<SchedCond::SchedCondition> _cond,
			std::set<std::shared_ptr<Net>> _stopNets, std::set<std::shared_ptr<Net>> _cancelNets,
			std::set<std::shared_ptr<Net>> _startNets, const std::string& description) :
			conditionFired(false)
	{
		this->description = description;
		this->id = id;

		this->condition = std::move(_cond);
		this->stopNets = std::move(_stopNets);
		this->cancelNets = std::move(_cancelNets);
		this->startNets = std::move(_startNets);

		// Check condition
		if (!condition->initialize())
			throw SynchronizationRuleException();

		for (const auto& condnetid : condition->getCondNets())
		{
			auto net = Registry::getRegistry()->getNet(condnetid);

			conditionNets.insert(net);
		}

		// find resources
		for (const auto& snet : startNets)
		{
			auto netresources = snet->getResourceNames();
			for (const auto& res : netresources)
			{
				startResources.insert(res);
			}
			startNames.insert(snet->getName());
		}

		for (const auto& snet : stopNets)
		{
			auto netresources = snet->getResourceNames();
			for (const auto& res : netresources)
			{
				stopResources.insert(res);
			}
			stopNames.insert(snet->getName());
		}

		for (const auto& snet : cancelNets)
		{
			cancelNames.insert(snet->getName());
		}

		conditionName = condition->toString();

		Registry::getRegistry()->setSyncState(id, SynchronizationRuleState::Active);
	}

	SynchronizationRule::~SynchronizationRule()
	{

	}

	bool SynchronizationRule::CheckCondition() const
	{
		// If condition has already been triggered or is invalid, it cannot be
		// triggered again
		if (conditionFired)
			return false;

		// should not happen
		if (!condition)
			return false;

		// Returns true iff condition is true, false otherwise (false or indeterminate)
		return condition->getValue();
	}

	bool SynchronizationRule::CheckConditionAlive() const
	{
		for (const auto& condnet : conditionNets)
		{
			if (condnet->isRequiredState(3, RPI::Ready, RPI::Scheduled, RPI::Running))
			{
				return true;
			}
		}
		return false;
	}

	const string& SynchronizationRule::getConditionName() const
	{
		return conditionName;
	}

	const set<string>& SynchronizationRule::getStartNames() const
	{
		return startNames;
	}

	const set<string>& SynchronizationRule::getStopNames() const
	{
		return stopNames;
	}

	const set<string>& SynchronizationRule::getCancelNames() const
	{
		return cancelNames;
	}

	const set<std::shared_ptr<Net>>& SynchronizationRule::getStopNets() const
	{
		return stopNets;
	}


	const set<shared_ptr<Net>>& SynchronizationRule::getConditionNets() const
	{
		return conditionNets;
	}

	bool SynchronizationRule::hasStopNet(const std::string& net) const
	{
		for (const auto& snet : stopNets)
			if (snet->getName() == net)
				return true;

		return false;
	}

	const set<std::shared_ptr<Net>>& SynchronizationRule::getCancelNets() const
	{
		return cancelNets;
	}

	const set<std::shared_ptr<Net>>& SynchronizationRule::getStartNets() const
	{
		return startNets;
	}

	void SynchronizationRule::fired()
	{
		conditionFired = true;
	}

	bool SynchronizationRule::isFired() const
	{
		return conditionFired;
	}

	void SynchronizationRule::invalidate()
	{
		// remove any references to nets to allow release of shared_ptr
		stopNets.clear();
		startNets.clear();
		cancelNets.clear();

		conditionNets.clear();

	}

	const set<string>& SynchronizationRule::getStartResources() const
	{
		return startResources;
	}

	const set<string>& SynchronizationRule::getStopResources() const
	{
		return stopResources;
	}

	const Sync_ID_t SynchronizationRule::getSyncID() const
	{
		return id;
	}

} /* namespace RPI */
