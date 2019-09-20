/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef RESOURCEMANAGER_HPP_
#define RESOURCEMANAGER_HPP_

#include <string>
#include <map>
#include <unordered_map>
#include <memory>
#include <rtt/os/Mutex.hpp>
#include "NetFwd.hpp"

namespace RPI
{
	class Net;

	/**
	 * \brief Management of resources used in net
	 *
	 * Some resources like robot joints can only be controlled by one net at any time.
	 * All other nets requesting these resources must be rejected, except they are scheduled
	 * directly after another net which has a superset of acquired resources.
	 *
	 * Nets must clean up their resources before their destructor, otherwise we may still
	 * own a shared pointer to the net (thus destructor will not be called)
	 */
	class ResourceManager
	{
	public:
		/**
		 * \brief Singleton access to ResourceManager
		 */
		static ResourceManager* getResourceManager();

		/**
		 * \brief Get users for given resource
		 *
		 * This method is guaranteed to be RT safe
		 *
		 * \return empty string, if resource is currently unused or not prepared
		 * properly
		 */
		const std::string& getResource(const std::string& resourceID) const;

		/**
		 * \brief Get last user for given resource
		 *
		 * This method is guaranteed to be RT safe. The last owner once locked the
		 * resource, but does not currently hold the resource. The resource has since
		 * not been acquired by any other user
		 *
		 * \return empty string, if resource is currently used, or it has not been
		 * prepared properly
		 */
		const std::string& getWeakResource(const std::string& resourceID) const;

		/**
		 * \brief Add usage of resource
		 *
		 * This method only works with resources that have been prepared and is
		 * guaranteed to be RT safe.
		 * The previous owner is cleared
		 *
		 * Although ResourceManager is thread-safe, this method should only be called
		 * within scheduling mutex to prevent others from stealing resources after
		 * freeing in previous nets
		 */
		void addResource(const std::string& resourceID, const std::string& net);

		/**
		 * \brief Prepares the ResourceManager for given resource
		 *
		 * This method is NOT RT safe
		 */
		void prepareResource(const std::string& resourceID);
		/**
		 * \brief Remove usage of resource for given net
		 *
		 * If resource is not held by given net, nothing happens
		 * This method can be called multiple times for the same resource/net
		 * without causing problems.
		 *
		 * This method is RT safe
		 */
		void removeResource(const std::string& resourceID, const std::string& net);

		/**
		 * Removes previous owner - should never be called explicitly
		 */
		void removeWeakResource(const std::string& resourceID, const std::string& net);

		/**
		 * \brief Remove usages on all resources a net currently holds
		 */
		void removeNetUsage(const std::string& netID);

		/**
		 * \brief Global Mutex for writing actuator setpoints
		 */
		RTT::os::Mutex actuatorMutex;
		const static std::string empty;
	private:
		ResourceManager();
		std::unordered_map<std::string, std::string> resourceMap;
		std::unordered_map<std::string, std::string> weakResourceMap;
		mutable RTT::os::MutexRecursive resourceMapMutex;

		static ResourceManager* theManager;
	};

} /* namespace RPI */
#endif /* RESOURCEMANAGER_HPP_ */

