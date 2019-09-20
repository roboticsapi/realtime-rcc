/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <boost/lexical_cast.hpp>
#include "mongoose.hpp"

#include "HTTPHandlers.hpp"
#include "../SynchronizationRule.hpp"
#include "../Registry.hpp"
#include "../VirtualRCCDevice.hpp"
#include "../TypeKit.hpp"
#include "../ExtensionLoader.hpp"
#include "../scheduling/SchedulingCondition.hpp"
#include "../Net.hpp"


using namespace std;

namespace RPI
{
	string HTML_escape_attrib(string text)
	{
		string ret = "";
		for (unsigned int i = 0; i < text.length(); i++)
		{
			if (text[i] == '<')
				ret += "&lt;";
			else if (text[i] == '>')
				ret += "&gt;";
			else if (text[i] == '"')
				ret += "&quot;";
			else if (text[i] == '\n')
				ret += " ";
			else if (text[i] == '\r')
				ret += "";
			else if (text[i] == '&')
				ret += "&amp;";
			else
				ret += text[i];
		}
		return ret;
	}

	string NetsHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string ret = "";
		if (method == "POST")
		{
			string xml = getVar(data, "rpinet");
			string dio = getVar(data, "dionet");
			string errors;
			if (xml != "" || dio != "")
			{
				Net_ID_t netid = Registry::getRegistry()->createNetID();

				bool loadsuccess = false;

				if (getVar(data, "useparams") == "1")
				{
					try
					{
						double frequency = boost::lexical_cast<double>(getVar(data, "frequency"));
						bool realtime = boost::lexical_cast<bool>(getVar(data, "isrealtime"));

						if(xml != "")
							loadsuccess = Registry::getRegistry()->loadNetXML(getVar(data, "desc"), xml, netid, SESSION_NONE, frequency, realtime);
						else
							loadsuccess = Registry::getRegistry()->loadNetDIO(getVar(data, "desc"), dio, netid, SESSION_NONE, errors, frequency, realtime);

					} catch(boost::bad_lexical_cast&)
					{

					}
				} else
				{
					if(xml != "")
						loadsuccess = Registry::getRegistry()->loadNetXML(getVar(data, "desc"), xml, netid, SESSION_NONE);
					else
						loadsuccess = Registry::getRegistry()->loadNetDIO(getVar(data, "desc"), dio, netid, SESSION_NONE, errors);

				}

				if (loadsuccess)
				{
					auto net = Registry::getRegistry()->getNet(netid);
					string netname = net->getName();
					string uri = "/nets/" + netname + "/";
					ret = string("HTTP/1.1 302 Found\r\n") + "Location: " + uri.c_str() + "\r\n"
							+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body><h1>Found.</h1></body></html>";
				} else
				{
					ret = string("HTTP/1.1 503 Service Unavailable\r\n") + "Content-Type: text/html\r\n" + "\r\n"
							+ "<html><body><h1>Service Unavailable</h1></body></html>";
				}
			}

		} else
		{
			vector<Net_ID_t> nets = Registry::getRegistry()->listNets();
			ret
					= string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
							+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/nets.xsl\" ?>\r\n"
							+ "<nets>\r\n";
			for (vector<Net_ID_t>::iterator it = nets.begin(); it < nets.end(); ++it)
			{
				auto net = Registry::getRegistry()->getNet(*it);
				string netname = net->getName();
				if (netname == "INVALID")
					continue;
				NetState state = net->getNetState();
				if(state == Unloading)
					state = Terminated;
				string desc = net->getDescription();

				string status = HTTPServer_state_text(state);
				ret += "<net uri=\"/nets/" + netname + "/\" status=\"" + status + "\" desc=\"" + HTML_escape_attrib(desc) + "\"/>\r\n";
			}
			ret += "</nets>";
		}
		return ret;
	}

	string ExtensionsHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string ret = "";

		ExtensionLoaderMap* extensions = ExtensionLoader::getInstance()->getExtensionLoaderMap();
		if (method == "POST")
		{
			string extension = getVar(data, "extension");

			if (extension != "" && (extensions->find(extension) == extensions->end()))
			{
				bool created = ExtensionLoader::getInstance()->loadExtensionFile(extension);

				if (!created) {
					return string("HTTP/1.1 400 Bad request\r\n");
				}
			}
			string status = "/" + path[0] + "/" + extension + "/";
			return string("HTTP/1.1 303 See Other\r\n") + "Location: " + status + "\r\n"
					+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body><h1>See Other</h1></body></html>";
		} else
		{

			ret
					= string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
							+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/extensions.xsl\" ?>\r\n"
							+ "<extensions>\r\n";
			for (ExtensionLoaderMap::iterator it = extensions->begin(); it != extensions->end(); ++it)
			{
				ret += "<extension file=\"" + it->second->filename + "\" name=\"" + it->first + "\" />\r\n";
			}
			ret += "</extensions>";
		}
		return ret;
	}

	string DevicesHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string ret = "";

		devicelist_t devices = Registry::getRegistry()->getDevices();

		if (method == "POST")
		{
			string action = getVar(data, "action");
			string name = getVar(data, "name");

			if (action == "remove")
			{
				Registry::getRegistry()->removeDevice(name);

				string status = "/" + path[0] + "/";
				return string("HTTP/1.1 303 See Other\r\n") + "Location: " + status + "\r\n"
						+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body>Device successfully deleted.</body></html>";
			}
			else
			{
				// Create a new device

				string type = getVar(data, "type");

				map<string, string> parametermap = getVars(data);
				parameter_t parameter;

				for (map<string, string>::iterator it = parametermap.begin(); it != parametermap.end(); ++it)
				{
					string pname = it->first;
					if (pname != "type" && pname != "name")
						parameter[pname] = it->second;
				}

				bool created = Registry::getRegistry()->createDevice(type, name, parameter);

				if (created) {
					string status = "/" + path[0] + "/" + name + "/";
					return string("HTTP/1.1 201 Created\r\n") + "Location: " + status + "\r\n"
							+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body>Device successfully created.</body></html>";
				}
				else {
					return string("HTTP/1.1 400 Bad request\r\n");
				}
			}
		} else
		{

			ret
					= string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
							+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/devices.xsl\" ?>\r\n"
							+ "<devices>\r\n";
			for (devicelist_t::iterator it = devices.begin(); it != devices.end(); ++it)
			{
				RPI::DeviceState state = Registry::getRegistry()->getDeviceState(it->name);
				std::string statename = (state == DeviceState::OPERATIONAL ? "operational" : (state == DeviceState::SAFE_OPERATIONAL ? "safe-operational" : "offline"));
				ret += "<device name=\"" + it->name + "\" type=\"" + it->type + "\" state=\"" + statename + "\">\r\n";

				for(const auto& it2 : it->implementedInterfaces)
				{
					ret += "<interface name=\"" + it2.first + "\">";
					for(const auto& it3 : it2.second) {
						ret += "<parameter name=\"" + it3.first + "\">" + it3.second + "</parameter>";
					}
					ret += "</interface>\r\n";
				}

				ret += "</device>\r\n";
			}
			ret += "</devices>";
		}
		return ret;
	}


	string RCCDeviceHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string ret = "";

		VirtualRCCDevice* rcc = Registry::getRegistry()->getVirtualRCC();
		if(rcc==0) return "";
		ret
				= string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
						+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/rcc.xsl\" ?>\r\n"
						+ "<device name=\"rcc\" estop=\"" + (rcc->getEStop()?"true":"false") + "\"></device>\r\n";

		return ret;
	}

	string DeviceParameterHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string ret = "";

		Device* device = Registry::getRegistry()->getDevice(path[1]);

		if (!device)
			return "";

		set<string> parameters = device->getParameters();
		set<string> mutableparameters = device->getMutableParameters();

		if (method == "POST")
		{
			bool modified = false;
			for (set<string>::iterator it = mutableparameters.begin(); it != mutableparameters.end(); ++it)
			{
				string newvalue = getVar(data, *it);
				if (newvalue != "")
				{
					device->setParameter(*it, newvalue);
					modified = true;
				}
			}
			if (modified)
				device->updateParameters();

			string status = "/" + path[0] + "/" + path[1] + "/" + path[2];
			return string("HTTP/1.1 303 See Other\r\n") + "Location: " + status + "\r\n"
					+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body><h1>See Other</h1></body></html>";

		} else
		{
			ret
					= string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
							+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/driverparameters.xsl\" ?>\r\n"
							+ "<parameters>\r\n";

			for (set<string>::iterator it = parameters.begin(); it != parameters.end(); ++it)
			{
				string mut = (mutableparameters.find(*it) != mutableparameters.end()) ? "true" : "false";
				ret += "\t<parameter name=\"" + *it + "\" mutable=\"" + mut + "\">" + device->getParameter(*it)
						+ "</parameter>\r\n";
			}

			ret += "</parameters>\r\n";
		}

		return ret;
	}

	string NetHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string netname = path[1];

		auto net= Registry::getRegistry()->getNet(netname);
		if (!net)
			return "";

		map<string, string> commInProperties = net->readCommunicationInProperties();
		Netcomm_Map_t commOutProperties = net->readCommunicationOutProperties();

		if (method == "POST")
		{
			string action = getVar(data, "action");
			if (action == "START")
			{
				std::unique_ptr<SchedCond::SchedCondition> condition(new SchedCond::SchedTrue());
				set<Net_ID_t> empty;
				set<Net_ID_t> start;

				start.insert(Registry::getRegistry()->getNetID(netname));

				auto syncID = Registry::getRegistry()->createSyncID();

				Net::scheduleNets(syncID, std::move(condition), empty, empty, start, "net start");
			} else if (action == "CANCEL")
			{
				net->cancel();
			} else if (action == "ABORT")
			{
				net->abort();
			} else if (action == "SCHEDULE")
			{
				// TODO: Implement
			} else if (action == "UNLOAD")
			{
				net->unload();
				string status = "/" + path[0] + "/";
				return string("HTTP/1.1 303 See Other\r\n") + "Location: " + status + "\r\n"
						+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body><h1>See Other</h1></body></html>";
			}

			map<string, string> commWrite;
			for (map<string, string>::iterator it = commInProperties.begin(); it != commInProperties.end(); ++it)
			{
				string value = getVar(data, it->first);
				if (value != "")
				{
					commWrite[it->first] = value;
				}
			}
			if (!commWrite.empty())
				net->writeCommunicationInProperties(commWrite);

			string status = "/" + path[0] + "/" + path[1] + "/";
			return string("HTTP/1.1 303 See Other\r\n") + "Location: " + status + "\r\n"
					+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body><h1>See Other</h1></body></html>";
		}

		stringstream ret;
		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/net.xsl\" ?>\r\n"
				<< "<net name=\"" << netname << "\" status=\"" << HTTPServer_state_text(
				net->getNetState()) << "\" desc=\"" << HTML_escape_attrib(net->getDescription()) << "\">\r\n";

		for (Netcomm_Map_t::iterator it = commOutProperties.begin(); it != commOutProperties.end(); ++it)
		{
			ret << "\t<data key=\"" << it->first << "\">" << it->second.first << "</data>\r\n";
		}
		for (map<string, string>::iterator it = commInProperties.begin(); it != commInProperties.end(); ++it)
		{
			ret << "\t<data key=\"" << it->first << "\">" << it->second << "</data>\r\n";
		}

		list<NetErrorState> ne = net->netErrorState;

		for(list<NetErrorState>::iterator it = ne.begin(); it != ne.end(); ++it) {
		ret << "\t<errorState major=\"" << it->errorMajor << "\" minor=\"" << it->errorMinor << "\" block=\""
				<< it->errorBlock << "\">" << it->errorMessage << "</errorState>\n";
		}

		for(const auto& condition: net->getConditions()) {
			ret << "\t<rule id=\"" << condition->getSyncID() << "\" condition=\"" << condition->getConditionName() << "\">\n";
			for(const auto& stop: condition->getStopNames())
				ret << "\t\t<stop name=\"" << stop << "\" />\n";
			for(const auto& start: condition->getStartNames())
				ret << "\t\t<start name=\"" << start << "\" />\n";
			for(const auto& cancel: condition->getCancelNames())
				ret << "\t\t<cancel name=\"" << cancel << "\" />\n";
			ret << "\t</rule>\n";
		}

		ret << "</net>\r\n";

		return ret.str();

	}

	string EvalHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		stringstream ret;
		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"
				<< "<eval>\n";

		string xml = getVar(data, "rpinet");
		string dio = getVar(data, "dionet");
		map<string, string> result;
		if(xml != "")
			result = Registry::getRegistry()->evalFragmentXML(xml);
		else
			result = Registry::getRegistry()->evalFragmentDIO(dio);

		for(auto it = result.begin(); it != result.end(); ++it) {
			ret << "\t<data key=\"" << it->first << "\">" << it->second << "</data>\n";
		}
		ret << "</eval>\r\n";

		return ret.str();
	}


	string NetXmlHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string netname = path[1];

		auto net = Registry::getRegistry()->getNet(netname);
		if (!net)
			return "";

		stringstream ret;
		string xml = net->getNetXML();
		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n" << xml << "\r\n";

		return ret.str();

	}

	string NetDioHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string netname = path[1];

		auto net = Registry::getRegistry()->getNet(netname);
		if (!net)
			return "";

		stringstream ret;
		string dio = net->getNetDIO();
		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/plain\r\n" << "\r\n" << dio;

		return ret.str();

	}

	string NetDebugHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string netname = path[1];

		auto net = Registry::getRegistry()->getNet(netname);
		if (!net)
			return "";

		stringstream ret;
		string debug = net->getNetDebug();
		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/plain\r\n" << "\r\n" << debug;

		return ret.str();

	}


	bool NetDebugStreamHandler::handleRequest(vector<string> path, string method, string data, string get, mg_connection* conn)
	{
		string netname = path[1];

		auto net = Registry::getRegistry()->getNet(netname);
		if (!net)
		{
			return false;
		}

		int cycle = 0;
		string ret = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\n";
		mg_write(conn, ret.c_str(), ret.length());

		while(true) {
			string debug = net->getNetDebug(cycle);
			if(debug.length() > 0 && mg_write(conn, debug.c_str(), debug.length()) < 1)
				break;
			usleep(100000);
		}
		return true;
	}

	string NetStatsHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string netname = path[1];

		auto net = Registry::getRegistry()->getNet(netname);
		if (!net)
			return "";

		stringstream ret;
		ret << "HTTP/1.1 200 OK\r\n"
			<< "Content-Type: text/xml\r\n" << "\r\n"
			<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"
			<< "<net min=\"" << net->minTime/1000000.0
			<< "\" max=\"" << net->maxTime/1000000.0
			<< "\" mincycle=\"" << net->minCycleTime/1000000.0
			<< "\" maxcycle=\"" << net->maxCycleTime/1000000.0
			<< "\">\r\n";

		map<string,RTT::os::TimeService::nsecs> min = net->getNetBlockMinTime(),
				max = net->getNetBlockMaxTime(),
				total = net->getNetBlockTotalTime();

		for(map<string,RTT::os::TimeService::nsecs>::iterator it = min.begin(); it != min.end(); ++it) {
			ret << "<module id=\"" << it->first
					<< "\" min=\"" << it->second/1000000.0
					<< "\" max=\"" << max[it->first]/1000000.0
					<< "\" total=\"" << total[it->first]/1000000.0
					<< "\" />";
		}
		ret << "</net>";
		return ret.str();

	}

	string ModulesDetailsHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		ModuleFactoryMap* modules = ExtensionLoader::getInstance()->getModuleFactoryMap();
		string
				ret =
						string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
								+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"
								+ "<modules>\r\n";
		for (ModuleFactoryMap::iterator it = modules->begin(); it != modules->end(); ++it)
		{
			ret += "<module uri=\"/modules/" + it->first + "/\" name=\"" + it->first + "\" >\r\n";

						Module* module = (*modules)[it->first]->createInstance(it->first, 0);
						//ret += " description=\"" + HTML_escape_attrib(module->getDescription()) + "\">\r\n";
						std::vector<PropertyBase*> props = module->getProperties();
						for (PropertyBag::Properties::iterator it2 = props.begin(); it2 != props.end(); ++it2)
						{
							ret += "<parameter name=\"" + (*it2)->getName() + "\" type=\"" + (*it2)->getType()->getHumanName() + "\" value=\""
									+ module->getPropertyValue(*it2) + "\" description=\"" + HTML_escape_attrib(
									module->getPropertyDescription(*it2)) + "\" />\r\n";
						}
						std::vector<Port*> ports = module->getPorts();

						for (auto it2 = ports.begin(); it2 != ports.end(); ++it2)
						{
							string dir = (*it2)->isOutPort() ? "out" : "in";
							ret += "<" + dir + "port name=\"" + (*it2)->getName() + "\" type=\"" + (*it2)->getType()->getHumanName()
									+ "\" description=\"" + HTML_escape_attrib(module->getPortDescription(*it2)) + "\" />\r\n";
						}
						(*modules)[it->first]->destroyInstance(module);
						ret += "</module>";

		}
		ret += "</modules>";
		return ret;
	}


	string TypesHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		list<TypeKit*> typeKits = TypeKits::getInstance()->getTypeKits();
		string
				ret =
						string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
								+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/types.xsl\" ?>\r\n"
								+ "<types>\r\n";
		for (list<TypeKit*>::iterator it = typeKits.begin(); it != typeKits.end(); ++it)
		{
			string kind = "basic";
			if(dynamic_cast<ArrayType*>(*it) != 0) kind = "array";
			if(dynamic_cast<ComplexType*>(*it) != 0) kind = "complex";
			ret += "<type name=\"" + (*it)->getHumanName() + "\" kind=\"" + kind + "\" />\r\n";
		}
		ret += "</types>";
		return ret;
	}


	string TypeHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string name = path[1];
		list<TypeKit*> typeKits = TypeKits::getInstance()->getTypeKits();
		for (list<TypeKit*>::iterator it = typeKits.begin(); it != typeKits.end(); ++it)
		{
			if((*it)->getHumanName() == name) {
				string
						ret =
								string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
										+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/type.xsl\" ?>\r\n"
										+ "<type name=\"" + HTML_escape_attrib(name) + "\">";

				ComplexType* tk = dynamic_cast<ComplexType*>(*it);
				if(tk != 0) {
					ComplexType::members_t members = tk->getMemberTypeKits();
					for(ComplexType::members_t::iterator it = members.begin(); it != members.end(); ++it)
					{
						ret += "<member name=\"" + HTML_escape_attrib(it->name) + "\" type=\"" + HTML_escape_attrib(it->type->getHumanName()) + "\" description=\"" + HTML_escape_attrib(it->description) + "\" />\r\n";
					}
				}

				ArrayType* at = dynamic_cast<ArrayType*>(*it);
				if(at != 0) {
					ret += "<array type=\"" + HTML_escape_attrib(at->getBasicTypeKit()->getHumanName()) + "\" />\r\n";
				}

				ret += "</type>";
				return ret;
			}
		}
		return "";
	}


	string TypeSourceHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string name = path[1];
		list<TypeKit*> typeKits = TypeKits::getInstance()->getTypeKits();
		for (list<TypeKit*>::iterator it = typeKits.begin(); it != typeKits.end(); ++it)
		{
			if((*it)->getHumanName() == name) {
				string
						ret =
								string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
										+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/type.java.xsl\" ?>\r\n"
										+ "<type name=\"" + HTML_escape_attrib(name) + "\">";

				ComplexType* tk = dynamic_cast<ComplexType*>(*it);
				if(tk != 0) {
					ComplexType::members_t members = tk->getMemberTypeKits();
					for(ComplexType::members_t::iterator it = members.begin(); it != members.end(); ++it)
					{
						ret += "<member name=\"" + HTML_escape_attrib(it->name) + "\" type=\"" + HTML_escape_attrib(it->type->getHumanName()) + "\" description=\"" + HTML_escape_attrib(it->description) + "\" />\r\n";
					}
				}

				ArrayType* at = dynamic_cast<ArrayType*>(*it);
				if(at != 0) {
					ret += "<array type=\"" + HTML_escape_attrib(at->getBasicTypeKit()->getHumanName()) + "\" />\r\n";
				}

				ret += "</type>";
				return ret;
			}
		}
		return "";
	}


	string ModulesHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		ModuleFactoryMap* modules = ExtensionLoader::getInstance()->getModuleFactoryMap();
		string
				ret =
						string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
								+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/modules.xsl\" ?>\r\n"
								+ "<modules>\r\n";
		for (ModuleFactoryMap::iterator it = modules->begin(); it != modules->end(); ++it)
		{
			ret += "<module uri=\"/modules/" + it->first + "/\" name=\"" + it->first + "\" />\r\n";
		}
		ret += "</modules>";
		return ret;
	}

	string ModuleHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		RTT::os::MutexLock ml(mutex);
		ModuleFactoryMap* modules = ExtensionLoader::getInstance()->getModuleFactoryMap();
		string name = path[1];
		string
				ret =
						string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
								+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/module.xsl\" ?>\r\n"
								+ "<module name=\"" + name + "\"";
		if (modules->find(name) != modules->end())
		{
			Module* module = (*modules)[name]->createInstance(name, 0);
			ret += " description=\"" + HTML_escape_attrib(module->getDescription()) + "\">\r\n";
			std::vector<PropertyBase*> props = module->getProperties();
			for (PropertyBag::Properties::iterator it = props.begin(); it != props.end(); ++it)
			{
				ret += "<parameter name=\"" + (*it)->getName() + "\" type=\"" + (*it)->getType()->getHumanName() + "\" value=\""
						+ module->getPropertyValue(*it) + "\" description=\"" + HTML_escape_attrib(
						module->getPropertyDescription(*it)) + "\" />\r\n";
			}
			std::vector<Port*> ports = module->getPorts();

			for (std::vector<Port*>::iterator it = ports.begin(); it != ports.end(); ++it)
			{
				string dir = (*it)->isOutPort() ? "out" : "in";
				ret += "<" + dir + "port name=\"" + (*it)->getName() + "\" type=\"" + (*it)->getType()->getHumanName()
						+ "\" description=\"" + HTML_escape_attrib(module->getPortDescription(*it)) + "\" />\r\n";
			}
			(*modules)[name]->destroyInstance(module);
			ret += "</module>";
			return ret;
		}
		return "";
	}

	string ModuleSourceHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		ModuleFactoryMap* modules = ExtensionLoader::getInstance()->getModuleFactoryMap();
		string name = path[1];
		string
				ret =
						string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
								+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/module.java.xsl\" ?>\r\n"
								+ "<module name=\"" + name + "\"";
		if (modules->find(name) != modules->end())
		{
			Module* module = (*modules)[name]->createInstance(name, 0);

			ret += " description=\"" + HTML_escape_attrib(module->getDescription()) + "\">\r\n";
			std::vector<PropertyBase*> props = module->getProperties();
			for (PropertyBag::Properties::iterator it = props.begin(); it != props.end(); ++it)
			{
				ret += "<parameter name=\"" + (*it)->getName() + "\" type=\"" + (*it)->getType()->getHumanName() + "\" value=\""
						+ module->getPropertyValue(*it) + "\" description=\"" + HTML_escape_attrib(
						module->getPropertyDescription(*it)) + "\" />\r\n";
			}
			std::vector<Port*> ports = module->getPorts();

			for (std::vector<Port*>::iterator it = ports.begin(); it != ports.end(); ++it)
			{
				string dir = (*it)->isOutPort() ? "out" : "in";
				ret += "<" + dir + "port name=\"" + (*it)->getName() + "\" type=\"" + (*it)->getType()->getHumanName()
						+ "\" description=\"" + HTML_escape_attrib(module->getPortDescription(*it)) + "\" />\r\n";
			}
			(*modules)[name]->destroyInstance(module);
			ret += "</module>";
			return ret;
		}
		return "";
	}






	string SessionsHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string ret = "";
		if (method == "POST")
		{
			string desc = getVar(data, "desc");
			if (desc != "")
			{
				Session_ID_t sessionid = Registry::getRegistry()->createSessionID(desc);
				string name = Registry::getRegistry()->getSessionName(sessionid);
				string uri = "/sessions/" + name + "/";
				ret = string("HTTP/1.1 302 Found\r\n") + "Location: " + uri.c_str() + "\r\n"
						+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body><h1>Found.</h1></body></html>";
			}

		} else
		{
			vector<Session_ID_t> sessions = Registry::getRegistry()->listSessions();
			ret
					= string("HTTP/1.1 200 OK\r\n") + "Content-Type: text/xml\r\n" + "\r\n"
							+ "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/sessions.xsl\" ?>\r\n"
							+ "<sessions>\r\n";
			for (vector<Session_ID_t>::iterator it = sessions.begin(); it < sessions.end(); ++it)
			{
				string desc = Registry::getRegistry()->getSessionDesc(*it);
				string name = Registry::getRegistry()->getSessionName(*it);
				if (name == "INVALID")
					continue;
				ret += "<session uri=\"/sessions/" + name + "/\" desc=\"" + desc + "\" />\r\n";
			}
			ret += "</sessions>";
		}
		return ret;
	}

	string SessionHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		string sessionname = path[1];
		Session_ID_t sessionid = Registry::getRegistry()->getSessionForName(sessionname);

		if (sessionid == SESSION_INVALID)
			return "";

		vector<Net_ID_t> nets = Registry::getRegistry()->listSessionNets(sessionid);

		if (method == "POST")
		{

			string action = getVar(data, "action");
			if (action == "LOADNET")
			{
				Net_ID_t net = Registry::getRegistry()->createNetID();
				string rpinet = getVar(data, "rpinet"), errors;
				if(rpinet != "") {
					Registry::getRegistry()->loadNetXML(getVar(data, "desc"), rpinet, net, sessionid);
				} else {
					Registry::getRegistry()->loadNetDIO(getVar(data, "desc"), getVar(data,"dionet"), net, sessionid, errors);
				}

				string status = "/nets/" + Registry::getRegistry()->getNet(net)->getName() + "/";
				return string("HTTP/1.1 303 See Other\r\n") + "Location: " + status + "\r\n"
						+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body><h1>See Other</h1></body></html>";

			} else if (action == "LOADNETPARAM")
			{
				try
				{
					double frequency = boost::lexical_cast<double>(getVar(data, "frequency"));
					bool realtime = boost::lexical_cast<bool>(getVar(data, "isrealtime"));

					Net_ID_t net = Registry::getRegistry()->createNetID();
					string rpinet = getVar(data, "rpinet");
					std::string errors;
					if(rpinet != "") {
						Registry::getRegistry()->loadNetXML(getVar(data, "desc"), rpinet, net, sessionid, frequency, realtime);
					} else {
						Registry::getRegistry()->loadNetDIO(getVar(data, "desc"), getVar(data, "dionet"), net, sessionid, errors, frequency, realtime);
					}

					string status = "/nets/" + Registry::getRegistry()->getNet(net)->getName() + "/";
					return string("HTTP/1.1 303 See Other\r\n") + "Location: " + status + "\r\n"
							+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body><h1>See Other</h1></body></html>";

				} catch (boost::bad_lexical_cast&)
				{

				}

				return "";
			} else if (action == "UPDATE")
			{

				for(vector<Net_ID_t>::iterator ni = nets.begin(); ni != nets.end(); ++ni) {
					auto netptr = Registry::getRegistry()->getNet(*ni);
					string net = netptr->getName();
					if(netptr->getNetState() == Terminated)
						continue;

					map<string, string> commInProperties = netptr->readCommunicationInProperties();
					map<string, string> commWrite;
					for (map<string, string>::iterator it = commInProperties.begin(); it != commInProperties.end(); ++it)
					{
						string value = getVar(data, net + "." + it->first);
						if (value != "")
						{
							commWrite[it->first] = value;
						}
					}
					if (!commWrite.empty())
						netptr->writeCommunicationInProperties(commWrite);
				}

			} else if (action == "TERMINATE") {

				for(vector<Net_ID_t>::iterator ni = nets.begin(); ni != nets.end(); ++ni) {
					auto net = Registry::getRegistry()->getNet(*ni);
					net->abort();
					net->unload();
				}

				Registry::getRegistry()->endSession(sessionid);
				string status = "/" + path[0] + "/";
				return string("HTTP/1.1 303 See Other\r\n") + "Location: " + status + "\r\n"
						+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body><h1>See Other</h1></body></html>";

			}

			string status = "/" + path[0] + "/" + path[1] + "/";
			return string("HTTP/1.1 303 See Other\r\n") + "Location: " + status + "\r\n"
					+ "Content-Type: text/html\r\n" + "\r\n" + "<html><body><h1>See Other</h1></body></html>";
		}

		stringstream ret;
		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/xml\r\n" << "\r\n"
				<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/session.xsl\" ?>\r\n"
				<< "<session name=\"" << Registry::getRegistry()->getSessionName(sessionid) <<
				"\" desc=\"" << HTML_escape_attrib(Registry::getRegistry()->getSessionDesc(sessionid)) << "\">\r\n";


		for(vector<Net_ID_t>::iterator ni = nets.begin(); ni != nets.end(); ++ni) {
			auto net = Registry::getRegistry()->getNet(*ni);
			NetState state = net->getNetState();
			if(state == Unloading) continue;
			ret << "\t<n id=\"" << net->getName() << "\" s=\""
					<< HTTPServer_state_text(state) << "\">";

			Netcomm_Map_t commOutProperties = net->readCommunicationOutProperties();
			for (Netcomm_Map_t::iterator it = commOutProperties.begin(); it != commOutProperties.end(); ++it)
				ret << "<d k=\"" << it->first << "\">" << it->second.first << "</d>";

			list<NetErrorState> ne = net->netErrorState;
			if(ne.size()>0)
				for(list<NetErrorState>::iterator neit = ne.begin(); neit != ne.end(); ++neit)
					ret << "<e major=\"" << neit->errorMajor << "\" minor=\"" << neit->errorMinor << "\" block=\""
						<< neit->errorBlock << "\">" << neit->errorMessage << "</e>";

			ret << "</n>\r\n";
		}
		ret << "</session>";

		return ret.str();
	}

	string LogHandler::handleRequest(vector<string> path, string method, string data, string get)
	{
		stringstream ret;

		int begin = atoi(getVar(get, "start").c_str());
		string filter = getVar(get, "filter");

		boost::tokenizer<> filtertok(filter);

		ret << "HTTP/1.1 200 OK\r\n" << "Content-Type: text/plain\r\n" << "\r\n";

		std::list<std::string> loglines = Registry::getRegistry()->getLogEvents();

		for (std::list<std::string>::const_iterator it = loglines.begin(); it != loglines.end(); ++it)
		{
			begin--;
			if (begin >= 0)
				continue;
			if (filter != "")
			{
				bool found = false;
				for (boost::tokenizer<>::iterator beg = filtertok.begin(); beg != filtertok.end(); ++beg)
				{
					found |= (it->find(*beg) != string::npos);
					if (found)
						break;
				}
				if (!found)
					continue;
			}

			ret << *it << endl;
		}

		return ret.str();
	}
}
