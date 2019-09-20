/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "ResourceManager.hpp"
#include <set>
#include <rtt/os/MutexLock.hpp>

namespace RPI
{
	using namespace std;


	const string& ResourceManager::getResource(const string& resourceID) const
	{
		RTT::os::MutexLock mapMutex(resourceMapMutex);
		auto it = resourceMap.find(resourceID);
		if (it != resourceMap.end())
			return it->second;
		else
			return empty;
	}

	const string& ResourceManager::getWeakResource(const string& resourceID) const
	{
		RTT::os::MutexLock mapMutex(resourceMapMutex);
		auto it = weakResourceMap.find(resourceID);
		if (it != weakResourceMap.end())
			return it->second;
		else
			return empty;
	}

	void ResourceManager::prepareResource(const string& resourceID)
	{
		RTT::os::MutexLock mapMutex(resourceMapMutex);
		if (resourceMap.find(resourceID) == resourceMap.end())
			resourceMap[resourceID].reserve(15);

		if (weakResourceMap.find(resourceID) == weakResourceMap.end())
			weakResourceMap[resourceID].reserve(15);
	}

	void ResourceManager::addResource(const string& resourceID, const string& netID)
	{
		RTT::os::MutexLock mapMutex(resourceMapMutex);
		if (resourceMap.find(resourceID) != resourceMap.end())
		{
			auto& res = resourceMap[resourceID];
			res.clear();
			res += netID;

			weakResourceMap[resourceID].clear();
		}
	}

	void ResourceManager::removeResource(const string& resourceID, const string& net)
	{
		RTT::os::MutexLock mapMutex(resourceMapMutex);
		if (resourceMap.find(resourceID) != resourceMap.end())
		{
			if (resourceMap[resourceID] == net)
			{
				resourceMap[resourceID].clear();
				auto& wr = weakResourceMap[resourceID];
				wr.clear();
				wr += net;
			}
		}
	}

	void ResourceManager::removeWeakResource(const string& resourceID, const string& net)
	{
		RTT::os::MutexLock mapMutex(resourceMapMutex);
		if (weakResourceMap.find(resourceID) != weakResourceMap.end())
		{
			if (weakResourceMap[resourceID] == net)
			{
				weakResourceMap[resourceID].clear();
			}
		}
	}

	/**
	 * remove all locks for specific net (for use in ~Net)
	 */
	void ResourceManager::removeNetUsage(const string& netID)
	{
		RTT::os::MutexLock mapMutex(resourceMapMutex);
		for (auto it = resourceMap.begin(); it != resourceMap.end(); ++it)
			if (it->second == netID)
				removeResource(it->first, netID);

		// Do not clean up weak resources
	}

	ResourceManager* ResourceManager::theManager = 0;
	const std::string ResourceManager::empty = "";

	ResourceManager::ResourceManager()
	{
	}

	ResourceManager* ResourceManager::getResourceManager()
	{
		if (!theManager)
			theManager = new ResourceManager();
		return theManager;
	}

} /* namespace RPI */
