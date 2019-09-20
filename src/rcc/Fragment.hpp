/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef FRAGMENT_H_
#define FRAGMENT_H_

#include "FragmentFwd.hpp"

#include "Module.hpp"
#include "NetAST.hpp"

#include <queue>
#include <string>

namespace RPI
{
	typedef std::map<std::string, Module*> modulemap_t;
	typedef std::list<std::string> topomapitem_t;
	typedef std::map<std::string, topomapitem_t> topomap_t;

	/**
	 * \brief Subpart of RPI net
	 *
	 * A fragment behaves like any other RPI module in an RPI net. Internally, a fragment
	 * is constructed like an RPI, so a hierarchy can be created. If a fragment is not
	 * active, no child module will be executed, so some execution time can be saved by
	 * disabling fragment execution instead of single module execution.
	 */
	class Fragment: public RPI::ActiveModule
	{
	public:
		/**
		 * \param net RPI net this fragment belongs to. Always the top-level Net, not a parent
		 * 	Fragment is required here.
		 * \param factory pointer to module factory map to create new modules
		 * \param fragment description of fragment
		 */
		Fragment(RPI::Net* net, RPI::ModuleFactoryMap* factory, RPI::rpi_fragment fragment);

		virtual ~Fragment();

		/**
		 * \brief Construct fragment, create all sub-elements
		 * \return true, if Fragment was successfully built.
		 *
		 * The Fragment constructor only initializes internal values, but does not create
		 * the necessary modules etc. Calling buildFragment() will do so, potentially recursively
		 * calling buildFragment() of sub-fragments.
		 */
		bool buildFragment();

		/**
		 * \brief List of leaf modules in Fragment
		 *
		 * All leaf modules, i.e. all real RPI modules contained in this Fragment, and possibly
		 * contained in sub-fragments will be returned.
		 */
		std::vector<Module*> getLeafModules();
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		virtual void updatePre();
		virtual void stopHook();
		virtual void cleanupHook();

		virtual void updateSensor();
		virtual void updateActuator();
		virtual bool isSensor();
		virtual bool isActuator();

	protected:
		/**
		 * \brief Ordered list of all modules in the Fragment
		 *
		 * The items in this list are topologically sorted. Execution of fragment must
		 * be performed in order of this list.
		 *
		 * \note Sub-fragments are represented as single module in this list.
		 * \see getLeafModules
		 */
		std::list<RPI::Module*> modules;

		/**
		 * \brief List of modules and corresponding module factories
		 *
		 * In contrast to modules, this list is not ordered, and also contains
		 * references to the module factories used for creation of the module.
		 * List is used to be able to destroy modules after use.
		 * Sub-fragments are not included in list.
		 */
		std::list<std::pair<ModuleFactory*, Module*> > loaded_modules;
		/**
		 * \brief List of sub-fragments contained in fragment
		 *
		 * Used for deletion of sub-fragments when fragment is destroyed
		 */
		std::list<Fragment*> loaded_fragments;

		virtual std::set<std::string> getResourceNames() const;
	private:
		Net* net;
		ModuleFactoryMap* factory;
		rpi_fragment fragment;

		template<class T>
		bool connectPorts(T& it, modulemap_t& moduleMap, topomap_t& leftMap, topomap_t& rightMap,
				std::queue<std::string>& empty);
		bool connectPort(rpi_port* port, Module* destModule, const std::string& modulename, modulemap_t& moduleMap, topomap_t& leftMap, topomap_t& rightMap);

		std::vector<Module*> sensorModules;
		std::vector<Module*> actuatorModules;
	};

}

#endif /* FRAGMENT_H_ */
