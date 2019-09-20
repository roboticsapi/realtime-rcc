/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <string>
#include <map>
#include <rtt/os/TimeService.hpp>
#include "../Net.hpp"
#include "../Registry.hpp"
#include "DIOProtocolParser/DIOProtocolP.hpp"

#ifndef DIOPROTOCOL_HPP_
#define DIOPROTOCOL_HPP_

#define DirectIOVersion "2.0"
//#define defaultUpdateFrequency 0.01;
const double defaultUpdateFrequency = 0.01;

namespace RPI
{

	/**
	 * \brief Encapsulation of DirectIO protocol
	 *
	 * The DirectIO protocol is currently used either on special DIO port (8081) or via
	 * standard HTTP using the websocket protocol
	 */
	class DIOProtocol
	{
	public:
		DIOProtocol();
		virtual ~DIOProtocol();

		/**
		 * \brief Determine status updates
		 * \return List of strings to be send to client
		 *
		 * Classes using the DirectIO protocol should call this method regularly to check
		 * for updated properties in running nets. The method will return a list of response
		 * commands for the client. The caller must provide markers that may be necessary
		 * for the client to recognize the end of a response.
		 */
		std::vector<std::unique_ptr<DIOProtocolP::DIOCommand> > updateStatus();
		/**
		 * \brief Handle new input data received from client
		 * \return Answer to client (without end marker)
		 * \param inp Input string from client (without end marker)
		 *
		 * Parses the given input data and returns response to client.
		 */
		std::unique_ptr<DIOProtocolP::DIOCommand> handleInput(std::string inp);

		/**
		 * This attribute is set to true (during handleInput) when a request for protocol
		 * termination was received from client
		 */
		bool dofinish;
	private:
		std::unique_ptr<DIOProtocolP::DIOCommand> evaluateCommand(
				const std::unique_ptr<DIOProtocolP::DIOCommand>& parseddata);

		bool handshake_ok;
		std::map<std::string, RTT::os::TimeService::nsecs> lastUpdate;
		std::map<Net_ID_t, NetState> lastNetState;

		struct subscription_t
		{
			std::string tag;
			double updateFrequency;
		};

		std::map<Net_ID_t, subscription_t> subscriptions;

		struct subscription_session_t
		{
			std::string tag;
			std::map<Net_ID_t, NetState> lastNetState;
		};

		std::map<Session_ID_t, subscription_session_t> subscriptions_session;

		std::string devTag;
		std::list<std::string> devKnown;
		std::map<std::string, RPI::DeviceState> devStates;
		std::map<std::string, RTT::os::TimeService::nsecs> devUpdated;

		RTT::os::Mutex dioprotocolmutex;

		struct syncsubscription_t
		{
			std::string tag;
			SynchronizationRuleState lastState;
		};

		// Subscriptions for syncrules
		// map: syncID -> tag
		std::map<Sync_ID_t, syncsubscription_t> syncSubscriptions;

		std::unique_ptr<DIOProtocolP::DIOCommand> errCmd(const std::string& message = "",
				const std::string& tag = "") const;
		std::unique_ptr<DIOProtocolP::DIOCommand> okCmd(const std::string& message = "",
				const std::string& tag = "") const;
	};

} /* namespace RPI */
#endif /* DIOPROTOCOL_HPP_ */
