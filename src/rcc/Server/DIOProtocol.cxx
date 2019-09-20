/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

/**
 * \class RPI::DIOProtocol
 * \verbatim
 * protocol for DirectIO
 * Port: 8081
 *
 * General syntax:
 * $tag|$command[$argument]*#[\n]
 *
 * $tag is a client specified identifier, that is responded as first token in each answer
 * 		corresponding to $command
 *
 * $command can be
 * 	  ver
 * 	    Arguments: |$version
 * 	    Checks version compatibility of client, must be first command issued
 *    gne
 *    	Arguments: |$netname|$refreshtimeout
 *    	$netname: name of net (rpinetX)
 *    	$refreshtimeout (in s): Value of Netcomm will only be updated if at lease $refreshtimeout seconds
 *    		have passed since last update. Net status will always be updated in next cycle
 *    snc
 *    	Arguments: ({$rpinetname:{$netcomminname:$netcomminvalue,...},...})
 *    nene
 *      Create new net
 *      Arguments: ($rpinet, $session, $desc, $frequency, $realtime)
 *      $rpinet: Net in DIO format
 *      $session: ID of session, 0 for no session (optional)
 *      $desc: Description (optional)
 *      $frequency: Frequency of net in s, or 0 for default (optional)
 *      $isrealtime: 1 if net is realtime critical, 0 otherwise (optional)
 *      Returns: err on error
 *               ok|$netname on success
 *               Net can be rejected even in ok case
 *    nest
 *    	Start net
 *    	Arguments: |$rpinetname
 *    neca
 *    	Cancel net
 *    	Arguments: |$rpinetname
 *    neab
 *    	Abort net
 *    	Argument: |$rpinetname
 *    nesc
 *    	Schedule net
 *    	Argument: $condition[|$stopnetname]+|endnet[|$startnetname]+
 *    neun
 *    	Unload net
 *    	Argument: |$rpinetname
 *    nse
 *    	Start new session
 *    	Argument: |$sessionname
 *    ase
 *    	Abort session, abort all running nets of session
 *    	Argument: |$sessionname
 *    gse
 *    	Arguments: |$sessionname
 *    	Notifies client of any state changes of a net within this session
 *    gde
 *    	Returns a list of all existing devices and activates automatic
 *    	push of new/removed devices
 *    	Arguments: none
 *    	Returns: All currently existing devices will be pushed first, and ok will be
 *    		issued afterwards
 *    quit
 *      Close communication channel (for debugging)
 *
 * Syntax of values returned is identically
 * $command can be
 * 	  err
 * 	  	An error occured, next argumnt will contain text description
 * 	  ok
 * 	  	Command successful
 * 	  nc
 * 	    Update of NetComm values
 * 	    Arguments: [|$netcommoutname|$netcommoutvalue]*
 * 	    Line is only send on update of at least one value
 * 	  ns
 * 	    Update of net status
 * 	  	Arguments: |$status
 * 	  st
 * 	  	Update of net status in session
 * 	  	Arguments: [|$netname|$status]*
 * 	  da
 * 	  	Device added (or anwser to gde)
 * 	  	Arguments: |$device|$type
 * 	  dr
 * 	  	Device removed
 * 	  	Arguments |$device
 * 	  ds
 * 	  	Device state changed
 * 	  	Arguments |$device|$state
 * \endverbatim
 */

#include "DIOProtocol.hpp"
#include <sstream>
#include <rtt/os/MutexLock.hpp>
#include "../scheduling/SchedulingConditionParser.hpp"
#include <rtt/Logger.hpp>
#include "DIOProtocolParser/DIOProtocolParser.hpp"

namespace RPI
{
	using namespace std;
	using namespace DIOProtocolP;

	DIOProtocol::DIOProtocol() :
			dofinish(false), handshake_ok(false), devTag("")
	{

	}

	DIOProtocol::~DIOProtocol()
	{
	}

	std::unique_ptr<DIOProtocolP::DIOCommand> DIOProtocol::handleInput(string input)
	{
		unique_ptr<DIOCommand> command = DIOProtocolParser::parse(input);

		if (!command)
		{
			return errCmd("invalid syntax", "none");
		}

		std::unique_ptr<DIOCommand> ret;

		if (command->command == "quit")
		{
			ret = okCmd("quitting");
			dofinish = true;
		} else if (!this->handshake_ok)
		{
			try
			{
				// version agreement
				if ((command->command == "ver")
						&& (command->getParameter<DIOString>(0)->getString() == DirectIOVersion))
				{
					ret = okCmd("handshake ok");
					this->handshake_ok = true;
				} else
				{
					ret = errCmd("bad handshake");
				}
			} catch (DIOPException& dioe)
			{
				ret = errCmd("bad handshake");
			}
		} else
		{
			ret = evaluateCommand(command);
		}

		if (ret->tag == "")
			ret->tag = command->tag;

		return ret;
	}

	std::unique_ptr<DIOProtocolP::DIOCommand> DIOProtocol::evaluateCommand(const unique_ptr<DIOCommand>& diocommand)
	{
		string command = diocommand->command;

		std::unique_ptr<DIOCommand> ret = errCmd("unknown command or uninitialized result");

		try
		{
			// get net - gne, $tag|gne|$netname|$refreshtime#
			if (command == "gne")
			{
				Net_ID_t net = Registry::getRegistry()->getNetID(diocommand->getParameter<DIOString>(0)->getString());
				if (net == NET_INVALID)
				{
					ret = errCmd("invalid net");
				} else
				{
					subscription_t sub;
					sub.tag = diocommand->tag;

					sub.updateFrequency = diocommand->getParameter<DIOFloat>(1)->getDouble();
					{
						RTT::os::MutexLock lock(dioprotocolmutex);
						subscriptions[net] = sub;
					}

					ret = okCmd();
				}
			} else if (command == "nene")
			{
				size_t numparam = diocommand->parameters.getSize();

				string dio = diocommand->getParameter<DIOString>(0)->getString();

				Session_ID_t sessionid;
				if (numparam >= 2)
					sessionid = Registry::getRegistry()->getSessionForName(
							diocommand->getParameter<DIOString>(1)->getString());
				else
					sessionid = SESSION_INVALID;

				// if no valid session found, use no session
				if (sessionid == SESSION_INVALID)
					sessionid = SESSION_NONE;

				string desc = "";
				if (numparam >= 3)
					desc = diocommand->getParameter<DIOString>(2)->getString();

				double frequency = RPI_DEFAULT_CYCLE_TIME;
				bool isRealtime = true;

				if (numparam >= 4)
				{
					frequency = diocommand->getParameter<DIOFloat>(3)->getDouble();
					if (frequency == 0)
						frequency = RPI_DEFAULT_CYCLE_TIME;
				}

				if (numparam >= 5)
					if (diocommand->getParameter<DIOInteger>(4)->getInt() == 0)
						isRealtime = false;

				Net_ID_t net = Registry::getRegistry()->createNetID();
				string errors;
				bool success = Registry::getRegistry()->loadNetDIO(desc, dio, net, sessionid, errors, frequency,
						isRealtime);

				if (success)
					ret = okCmd(Registry::getRegistry()->getNet(net)->getName());
				else
					ret = errCmd(errors);
			} else if (command == "snc")
			{
				bool ok = true;

				// snc has only one parameter which is a map rpinet -> netcommvalues
				for (const auto& rpinetentry : *diocommand->getParameter<DIOParameterMap>(0))
				{
					auto net = Registry::getRegistry()->getNet(rpinetentry.first);
					if (!net)
					{
						ret = errCmd("invalidnetname");
						ok = false;
						break;
					} else
					{
						map<string, string> ncupdate;
						for (const auto& ncentry : *rpinetentry.second->getParameterT<DIOParameterMap>())
						{
							ncupdate[ncentry.first] = ncentry.second->getParameterT<DIOString>()->getString();
						}
						net->writeCommunicationInProperties(ncupdate);
					}
				}

				if (ok)
				{
					ret = okCmd();
				}
			} else if (command == "nest" || command == "neca" || command == "neab" || command == "neun")
			{
				auto net = Registry::getRegistry()->getNet(diocommand->getParameter<DIOString>(0)->getString());
				auto netid = Registry::getRegistry()->getNetID(diocommand->getParameter<DIOString>(0)->getString());
				if (!net)
				{
					ret = errCmd("invalid net");
				} else
				{
					bool ok = true;
					if (command == "nest")
					{
						std::unique_ptr<SchedCond::SchedCondition> condition(new SchedCond::SchedTrue());
						set<Net_ID_t> empty;
						set<Net_ID_t> start;

						start.insert(netid);

						auto syncID = Registry::getRegistry()->createSyncID();

						ok &= Net::scheduleNets(syncID, std::move(condition), empty, empty, start, "net start");
					} else if (command == "neca")
						net->cancel();
					else if (command == "neab")
						net->abort();
					else if (command == "neun")
						net->unload();

					if (ok)
						ret = okCmd();
					else
						ret = errCmd("net start failed");
				}

			} else if (command == "nesc")
			{
				std::set<Net_ID_t> startnets, stopnets, cancelnets;
				bool ok = true;

				// Parse condition (field 2)
				std::unique_ptr<SchedCond::SchedCondition> condition = SchedCond::SchedulingConditionParser::parse(
						diocommand->getParameter<DIOString>(0)->getString());

				auto dstopnets = diocommand->getParameter<DIOParameterList>(1);
				auto dcancelnets = diocommand->getParameter<DIOParameterList>(2);
				auto dstartnets = diocommand->getParameter<DIOParameterList>(3);

				std::string description;
				try {
					auto diodescription = diocommand->getParameter<DIOString>(4);
					description = diodescription->getString();
				} catch(DIOPException&)
				{
					description = "";
				}

				if (!dstopnets || !dcancelnets || !dstartnets)
				{
					ret = errCmd("invalid arguments");

				} else
				{

					for (const auto& sn : *dstopnets)
					{
						Net_ID_t net = Registry::getRegistry()->getNetID(sn->getParameterT<DIOString>()->getString());

						if (net == NET_INVALID)
						{
							ret = errCmd("invalid netname");
							ok = false;
							break;
						} else
						{
							stopnets.insert(net);
						}

					}

					for (const auto& sn : *dcancelnets)
					{
						Net_ID_t net = Registry::getRegistry()->getNetID(sn->getParameterT<DIOString>()->getString());

						if (net == NET_INVALID)
						{
							ret = errCmd("invalid netname");
							ok = false;
							break;
						} else
						{
							cancelnets.insert(net);
						}

					}

					for (const auto& sn : *dstartnets)
					{
						Net_ID_t net = Registry::getRegistry()->getNetID(sn->getParameterT<DIOString>()->getString());

						if (net == NET_INVALID)
						{
							ret = errCmd("invalid netname");
							ok = false;
							break;
						} else
						{
							startnets.insert(net);
						}

					}
					if (ok)
					{
//						if(startnets.size() > 0 && stopnets.size() > 0)
//							RTT::log(RTT::LoggerLevel::Info) << "Scheduling rpinet" << *startnets.begin() << " after rpinet" << *stopnets.begin() << RTT::endlog();
//						else if(startnets.size() > 0 && stopnets.size() == 0)
//							RTT::log(RTT::LoggerLevel::Info) << "Starting rpinet" << *startnets.begin() << RTT::endlog();
						auto syncID = Registry::getRegistry()->createSyncID();
						if (Net::scheduleNets(syncID, std::move(condition), stopnets, cancelnets, startnets, description))
							ret = okCmd(std::to_string(syncID));
						else
							ret = errCmd("scheduling not possible");
					}
				}
			} else if (command == "nse")
			{
				Session_ID_t sid = Registry::getRegistry()->createSessionID(
						diocommand->getParameter<DIOString>(0)->getString());
				ret = okCmd(Registry::getRegistry()->getSessionName(sid));
			} else if (command == "ase")
			{
				Session_ID_t sid = Registry::getRegistry()->getSessionForName(
						diocommand->getParameter<DIOString>(0)->getString());
				if (sid == SESSION_INVALID || sid == SESSION_NONE)
				{
					ret = errCmd("invalid session");
				} else
				{
					vector<Net_ID_t> nets = Registry::getRegistry()->listSessionNets(sid);

					for (vector<Net_ID_t>::iterator ni = nets.begin(); ni != nets.end(); ++ni)
					{
						auto net = Registry::getRegistry()->getNet(*ni);
						net->abort();
						net->unload();
					}

					Registry::getRegistry()->endSession(sid);

					ret = okCmd();
				}
			} else if (command == "gse")
			{
				Session_ID_t sid = Registry::getRegistry()->getSessionForName(
						diocommand->getParameter<DIOString>(0)->getString());

				if (sid == SESSION_INVALID || sid == SESSION_NONE)
				{
					ret = errCmd("invalid session");
				} else
				{
					subscription_session_t subs;
					subs.tag = diocommand->tag;

					subscriptions_session[sid] = subs;
					ret = okCmd();
				}
			} else if (command == "gde")
			{
				RTT::os::MutexLock ml(dioprotocolmutex);
				devKnown.clear();
				devStates.clear();
				devTag = diocommand->tag;
				ret = okCmd();
			} else if (command == "gsy")
			{
				RTT::os::MutexLock ml(dioprotocolmutex);

				auto syncstr = diocommand->getParameter<DIOString>(0)->getString();
				Sync_ID_t syncID = boost::lexical_cast<unsigned long>(syncstr);
				auto current_state = Registry::getRegistry()->getSyncState(syncID);
				if (current_state == SynchronizationRuleState::InvalidRule)
				{
					ret = errCmd("Invalid synchronization rule");
				} else
				{
					syncsubscription_t sub;
					sub.tag = diocommand->tag;
					sub.lastState = SynchronizationRuleState::InvalidRule;

					syncSubscriptions[syncID] = sub;

					ret = okCmd();
				}
			}
		} catch (DIOPException& dioe)
		{
			ret = errCmd(dioe.getCause());
		} catch (...)
		{
			RTT::log(RTT::Critical) << "Unknown exception caught" << RTT::endlog();
			ret = errCmd("Fatal error in DIOProtocol.cxx");
		}

		return ret;
	}

	vector<unique_ptr<DIOCommand> > DIOProtocol::updateStatus()
	{
		RTT::os::MutexLock lock(dioprotocolmutex);
		vector<unique_ptr<DIOCommand> > result;

		// send netcomm and net status updates
		for (map<Net_ID_t, subscription_t>::iterator it = subscriptions.begin(); it != subscriptions.end(); ++it)
		{
			bool ncdata = false;

			unique_ptr<DIOCommand> netstate, netcomm;

			auto net = Registry::getRegistry()->getNet(it->first);
			NetState state;
			// net disappeared ...
			if (!net || (state = net->getNetState()) == Invalid)
			{
				state = Invalid;
				netstate = unique_ptr<DIOCommand>(new DIOCommand(it->second.tag, "ns"));
				netstate->parameters.addParameter(unique_ptr<DIOParameter>(new DIOString("UNLOADED")));
			} else
			{
				string netname = net->getName();

				// check whether state of net has changed
				if (lastNetState.find(it->first) == lastNetState.end() || lastNetState[it->first] != state)
				{
					lastNetState[it->first] = state;
					netstate = unique_ptr<DIOCommand>(new DIOCommand(it->second.tag, "ns"));
					netstate->parameters.addParameter(
							unique_ptr<DIOParameter>(new DIOString(HTTPServer_state_text(state))));
				}

				if (state != Ready && state != Rejected)
				{

					Netcomm_Map_t netout = net->readCommunicationOutProperties();

					netcomm = unique_ptr<DIOCommand>(new DIOCommand(it->second.tag, "nc"));
					unique_ptr<DIOParameterMap> pmap(new DIOParameterMap());

					for (Netcomm_Map_t::iterator it2 = netout.begin(); it2 != netout.end(); ++it2)
					{
						string ncname = netname + "." + it2->first;
						RTT::os::TimeService::nsecs lread = lastUpdate[ncname];
						if (lread == 0)
							lread = 1;

						// updated since last access (or never accessed before)
						if (netstate || lread == 1 || it2->second.second > lread)
						{
							// passed enough time since last update?
							RTT::os::TimeService::Seconds s = RTT::os::TimeService::Instance()->getNSecs(lread) / 1e9;
							if (netstate || s > it->second.updateFrequency)
							{
								pmap->addParameter(it2->first,
										unique_ptr<DIOParameter>(new DIOString(it2->second.first)));
								//n1 << "|" << it2->first << "|" << it2->second.first;
								lastUpdate[ncname] = RTT::os::TimeService::Instance()->getNSecs();
								ncdata = true;
							}
						}
					}

					netcomm->parameters.addParameter(std::move(pmap));
				}
			}

			if (netstate && state == RPI::Running)
			{
				result.push_back(std::move(netstate));
			}

			if (netcomm && ncdata)
			{
				result.push_back(std::move(netcomm));
			}

			if (netstate && state != RPI::Running)
			{
				result.push_back(std::move(netstate));
			}
		}

		for (map<Session_ID_t, subscription_session_t>::iterator it = subscriptions_session.begin();
				it != subscriptions_session.end(); ++it)
		{

			vector<Net_ID_t> nets = Registry::getRegistry()->listSessionNets(it->first);
			map<Net_ID_t, NetState>& netstates = it->second.lastNetState;

			unique_ptr<DIOCommand> sdioc(new DIOCommand(it->second.tag, "st"));
			unique_ptr<DIOParameterMap> spm(new DIOParameterMap());

			bool havestates = false;

			for (vector<Net_ID_t>::iterator nit = nets.begin(); nit != nets.end(); ++nit)
			{
				auto net = Registry::getRegistry()->getNet(*nit);
				NetState currentstate = net->getNetState();
				if (netstates.find(*nit) == netstates.end() || netstates[*nit] != currentstate)
				{
					spm->addParameter(net->getName(),
							unique_ptr<DIOParameter>(new DIOString(HTTPServer_state_text(currentstate))));

					netstates[*nit] = currentstate;

					havestates = true;
				}
			}
			sdioc->parameters.addParameter(std::move(spm));

			if (havestates)
			{
				result.push_back(std::move(sdioc));
			}
		}

		for (map<Net_ID_t, subscription_t>::iterator it = subscriptions.begin(); it != subscriptions.end();)
		{

			// net disappeared ...
			if (!Registry::getRegistry()->getNet(it->first))
			{
				subscriptions.erase(it++);
			} else
			{
				++it;
			}
		}

		// Notify for changes in synchronization rule state
		for (auto it = syncSubscriptions.begin(); it != syncSubscriptions.end(); ++it)
		{
			auto curState = Registry::getRegistry()->getSyncState(it->first);
			if (curState != it->second.lastState)
			{
				it->second.lastState = curState;
				unique_ptr<DIOCommand> syncUpdate = unique_ptr<DIOCommand>(new DIOCommand(it->second.tag, "rs"));

				syncUpdate->parameters.addParameter(
						unique_ptr<DIOString>(new DIOString(SynchronizationRuleStateString[(int) curState])));

				result.push_back(std::move(syncUpdate));
			}
		}

		if (devTag != "")
		{
			// update devices
			devicelist_t devsReg = Registry::getRegistry()->getDevices();
			map<string, DeviceInfo> dimap;
			list<string> devRegList;
			list<string> devChanged;
			map<string, string> devTypes;
			for (devicelist_t::iterator it = devsReg.begin(); it != devsReg.end(); ++it)
			{
				dimap[it->name] = *it;
				devRegList.push_back(it->name);
				devTypes[it->name] = it->type;
				RPI::DeviceState state = Registry::getRegistry()->getDeviceState(it->name);
				RTT::os::TimeService::nsecs updated = Registry::getRegistry()->getDeviceLastUpdated(it->name);
				if(updated != 0 && devUpdated[it->name] != updated) {
					devKnown.remove(it->name);
					devUpdated[it->name] = updated;
				}

				if (devStates.find(it->name) == devStates.end() || devStates[it->name] != state)
				{
					devStates[it->name] = state;
					devChanged.push_back(it->name);
				}
			}

			devRegList.sort();
			//devKnown.sort();

			list<string> devToAdd, devToRemove;

			list<string>::iterator lit, rit;

			lit = devRegList.begin();
			rit = devKnown.begin();

			bool devlistfinished = false;

			while (!devlistfinished)
			{
				if (lit != devRegList.end() && rit != devKnown.end())
				{
					if (*lit == *rit)
					{
						++lit;
						++rit;
					} else if (*lit < *rit)
					{
						devToAdd.push_back(*lit);
						++lit;
					} else
					{
						devToRemove.push_back(*rit);
						++rit;
					}
				} else if (lit == devRegList.end() && rit != devKnown.end())
				{
					devToRemove.push_back(*rit);
					++rit;
				} else if (rit == devKnown.end() && lit != devRegList.end())
				{
					devToAdd.push_back(*lit);
					++lit;
				} else
				{
					devlistfinished = true;
				}
			}

			if (devToAdd.size() > 0 || devToRemove.size() > 0 || devChanged.size() > 0)
			{
				devKnown = devRegList;
				if (devToAdd.size() > 0)
				{
					unique_ptr<DIOCommand> dacmd = unique_ptr<DIOCommand>(new DIOCommand(devTag, "da"));
					unique_ptr<DIOParameterMap> damap = unique_ptr<DIOParameterMap>(new DIOParameterMap());

					for (list<string>::iterator it = devToAdd.begin(); it != devToAdd.end(); ++it)
					{
						auto devmap = unique_ptr<DIOParameterMap>(new DIOParameterMap);
						auto interfaces = unique_ptr<DIOParameterMap>(new DIOParameterMap);

						DeviceInfo di = dimap[*it];
						for (const auto& iface : di.implementedInterfaces)
						{
							auto ifaces = unique_ptr<DIOParameterMap>(new DIOParameterMap());
							for(const auto& prop: iface.second) {
								ifaces->addParameter(prop.first, unique_ptr<DIOString>(new DIOString(prop.second)));
							}
							interfaces->addParameter(iface.first, std::move(ifaces));
						}

						devmap->addParameter("type", unique_ptr<DIOParameter>(new DIOString(devTypes[*it])));
						devmap->addParameter("interfaces", move(interfaces));

						damap->addParameter(*it, move(devmap));
					}
					dacmd->parameters.addParameter(std::move(damap));
					result.push_back(std::move(dacmd));
				}

				if (devToRemove.size() > 0)
				{
					unique_ptr<DIOCommand> drcmd = unique_ptr<DIOCommand>(new DIOCommand(devTag, "dr"));
					unique_ptr<DIOParameterList> drlist = unique_ptr<DIOParameterList>(new DIOParameterList());

					for (list<string>::iterator it = devToRemove.begin(); it != devToRemove.end(); ++it)
					{
						drlist->addParameter(unique_ptr<DIOParameter>(new DIOString(*it)));
						devStates.erase(*it);
					}

					drcmd->parameters.addParameter(std::move(drlist));
					result.push_back(std::move(drcmd));
				}

				if (devChanged.size() > 0)
				{
					unique_ptr<DIOCommand> dscmd = unique_ptr<DIOCommand>(new DIOCommand(devTag, "ds"));
					unique_ptr<DIOParameterMap> dsmap = unique_ptr<DIOParameterMap>(new DIOParameterMap());

					for (list<string>::iterator it = devChanged.begin(); it != devChanged.end(); ++it)
					{
						RPI::DeviceState state = devStates[*it];
						if (state == DeviceState::OFFLINE)
						{
							dsmap->addParameter(*it, unique_ptr<DIOParameter>(new DIOString("off")));
						} else if (state == DeviceState::OPERATIONAL)
						{
							dsmap->addParameter(*it, unique_ptr<DIOParameter>(new DIOString("op")));
						} else if (state == DeviceState::SAFE_OPERATIONAL)
						{
							dsmap->addParameter(*it, unique_ptr<DIOParameter>(new DIOString("safeop")));
						}
					}

					dscmd->parameters.addParameter(move(dsmap));
					result.push_back(move(dscmd));
				}
			}
		}

		return result;
	}

	std::unique_ptr<DIOCommand> DIOProtocol::okCmd(const std::string& message, const std::string& tag) const
	{
		std::unique_ptr<DIOCommand> cmd(new DIOCommand());

		cmd->tag = tag;
		cmd->command = "ok";

		if (message != "")
			cmd->parameters.addParameter(std::unique_ptr<DIOParameter>(new DIOString(message)));

		return cmd;
	}

	std::unique_ptr<DIOCommand> DIOProtocol::errCmd(const std::string& message, const std::string& tag) const
	{
		std::unique_ptr<DIOCommand> cmd(new DIOCommand());

		cmd->tag = tag;
		cmd->command = "err";

		if (message != "")
			cmd->parameters.addParameter(std::unique_ptr<DIOParameter>(new DIOString(message)));

		return cmd;

	}

} /* namespace RPI */
