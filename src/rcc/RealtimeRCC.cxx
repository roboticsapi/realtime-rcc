/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <rtt/os/main.h>
#include <signal.h>
#include <iostream>
#include <iomanip>

#include "RegistryImpl.hpp"
#include "Server/HTTPServer.hpp"
#include "VirtualRCCDevice.hpp"
#include "version_svn.h"
#include "version_bin.h"

#ifndef WIN32
#include "stacktrace.h"
#endif

#ifdef WITH_OCL
#include <orocos/ocl/TaskBrowser.hpp>
#endif

using namespace std;
using namespace RPI;

void segfault(int);
void sighup(int);
void usage();

bool doterminate;
bool headless;

int ORO_main(int argc, char** argv)
{
	headless = ((argc > 1) && ((string) argv[1] == "nobrowser"));

	signal(SIGSEGV, segfault);

#ifndef WIN32
	signal(SIGPIPE, SIG_IGN);
	if (headless)
	{
		signal(SIGHUP, sighup);
		signal(SIGINT, sighup);
		signal(SIGTERM, sighup);
	}
#endif

	doterminate = false;

	//RTT::log().allowRealTime();
	//RTT::log().setLogLevel(RTT::Logger::RealTime);

	if (RTT::log().getLogLevel() < RTT::Logger::Info)
	{
		RTT::log().setLogLevel(RTT::Logger::Info);
		/*RTT::log(RTT::Info) << argv[0] << " raises LogLevel to 'Info' (5). See also file 'orocos.log'."
		 << RTT::endlog();*/
	}

	// Log as error to force inclusion in web error log
	RTT::log(RTT::Error) << "RealtimeRCC version " << RCC_VERSION_MAJOR << "." << RCC_VERSION_MINOR << "."
			<< RCC_VERSION_PATCH << " starting ..." << RTT::endlog();

	RTT::log(RTT::Info) << "RealtimeRCC binary version 0x" << std::hex << rcc_bin_version_id << std::dec << RTT::endlog();

	if (RegistryImpl::getRegistry()->configure())
	{
		RegistryImpl::getRegistry()->start();

		if (headless)
		{
			cout << "RealtimeRCC is running, send SIGINT or SIGHUP to terminate ..." << endl;
			while (!doterminate)
			{
				usleep(500000);
			}
		} else
		{
#ifdef WITH_OCL
			OCL::TaskBrowser browser(RegistryImpl::getRegistry());

#ifdef WIN32
			browser.setColorTheme(OCL::TaskBrowser::ColorTheme::nocolors);
#else
			//if(getenv("TERM") == "")
			//browser.setColorTheme(OCL::ColorTheme::nocolors);
#endif

			browser.loop();
#else
			std::cout << std::endl << "RealtimeRCC version " << RCC_VERSION_MAJOR << "." << RCC_VERSION_MINOR << "."
					<< RCC_VERSION_PATCH << " ready:" << std::endl << std::endl;

			usage();

			bool exit = false;
			bool estop = false;
			std::string inp;
			while (!exit)
			{
				std::cout << "> ";

				if (cin.eof())
				{
					exit = true;
					continue;
				}

				cin >> inp;

				if (inp == "k" || inp == "kill")
				{
					RegistryImpl::getRegistry()->killAllNets();
					std::cout << "Killed all running nets" << std::endl;
				} else if (inp == "q" || inp == "quit")
				{
					exit = true;
				} else if (inp == "u" || inp == "usage")
				{
					usage();
				} else if (inp == "n" || inp == "nets")
				{
					vector<Net_ID_t> nets = Registry::getRegistry()->listNets();

					unsigned int col1 = 0;

					for (const auto& net : nets)
					{
						unsigned int len = Registry::getRegistry()->getNetName(net).length();
						if (len > col1)
							col1 = len;
					}

					for (const auto& net : nets)
					{
						string netname = Registry::getRegistry()->getNetName(net);
						if (netname == "INVALID")
							continue;
						string desc = Registry::getRegistry()->getNetDescription(net);

						string status = HTTPServer_state_text(Registry::getRegistry()->getNetState(net));
						cout << left << setw(col1) << netname + " " << setw(10) << status << "\t" << desc << endl;
					}

				} else if (inp == "d" || inp == "devices")
				{
					devicelist_t devices = Registry::getRegistry()->getDevices();

					int col1 = 0, col2 = 0;

					for (const auto& dev : devices)
					{
						if (dev.name.length() > col1)
							col1 = dev.name.length();
						if (dev.type.length() > col2)
							col2 = dev.type.length();
					}

					for (const auto& dev : devices)
					{
						RPI::DeviceState state = Registry::getRegistry()->getDeviceState(dev.name);
						std::string statename = (
								state == DeviceState::OPERATIONAL ? "operational" :
										(state == DeviceState::SAFE_OPERATIONAL ? "safe-operational" : "offline"));
						cout << left << setw(col1) << dev.name << " " << setw(col2) << dev.type << " " << statename
								<< endl;
					}
				} else if(inp == "c" || inp == "crashdump")
				{
					Registry::getRegistry()->triggerCrashDump();

				} else if(inp == "s"  || inp == "stop")
				{
					estop = !estop;
					Registry::getRegistry()->getVirtualRCC()->setParameter("needestop", estop?"true":"false");
					Registry::getRegistry()->getVirtualRCC()->updateParameters();
					if(estop)
						std::cout << "EMERGENCY STOP requirement activated" << std::endl;
					else
						std::cout << "Emergency stop requirement deactivated" << std::endl;

				} else
				{
					std::cout << "Unknown command: " << inp << std::endl;
				}
			}
#endif
		}

		// stop webserver
		RTT::log(RTT::Info) << "Shutting down web server..." << RTT::endlog();
		HTTPServer::getInstance()->shutdown();
		sleep(1);

		RTT::log(RTT::Info) << "Shutting down RPI registry..." << RTT::endlog();
		RegistryImpl::getRegistry()->stop();
	}

	RegistryImpl::getRegistry()->cleanup();
	RegistryImpl::clean();

	RTT::log(RTT::Info) << "Waiting for web server to complete..." << RTT::endlog();
	HTTPServer::cleanup();

	RTT::log(RTT::Info) << "RealtimeRCC ended." << RTT::endlog();

	return 0;
}

void segfault(int)
{
	cout << "------ Segfault detected ------" << endl;

#ifndef WIN32
	cout << "Segfault detected, printing stacktrace" << endl;

	print_stacktrace();
#endif

	exit(1);
}

void sighup(int)
{
	doterminate = true;
}

void usage()
{
	cout << "k[ill]\t\tKill all running nets" << endl;
	cout << "q[uit]\t\tQuit from RealtimeRCC" << endl;
	cout << "u[sage]\t\tPrint this usage info" << endl;
	cout << "n[ets]\t\tList all nets" << endl;
	cout << "d[evices]\tList all devices" << endl;
	cout << "c[rashdump]\tRequest crash dump" << endl;
	cout << "s[top]\t\tToggle emergency stop requirement" << endl;
}

