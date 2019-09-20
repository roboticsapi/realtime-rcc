/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "CanOpenMaster.hpp"
#include <rtt/Activity.hpp>
//#include <initializer_list>

namespace canopen
{

	CanOpenMaster::CanOpenMaster(std::string name, RPI::parameter_t parameters) :
			RPI::Device(name, parameters), RTT::TaskContext(name, PreOperational), canopen(name, parameters)
	{
		RTT::Seconds cycleTime = getParameterT("cycleTime", 8000) / 1e6;
		this->setActivity(new RTT::Activity(RTT::os::HighestPriority, cycleTime, 0, name));
		canopen.addNodeStateListener(0, this);
	}

	CanOpenMaster::~CanOpenMaster()
	{
		canopen.removeNodeStateListener(0, this);
		/*for(std::list<PDOJob>::iterator it = TxPDOs.begin(); it != TxPDOs.end();) {
			canopen.removePDOListener(it->nodeId, it->pdoId, this);
		}
		TxPDOs.clear();
		RxPDOs.clear();*/

		stop();
	}

	CanOpenMaster* CanOpenMaster::createDevice(std::string name, RPI::parameter_t parameters)
	{
		CanOpenMaster* ret = new CanOpenMaster(name, parameters);
		ret->configure();
		ret->start();
		return ret;
	}

	void CanOpenMaster::updateParameters()
	{
		RTT::Seconds cycleTime = getParameterT("cycleTime", 8000);
		getActivity()->setPeriod(cycleTime);
	}

	std::set<std::string> CanOpenMaster::getMutableParameters() const
	{
		std::set<std::string> ret;
		ret.insert("cycleTime");
		return ret;
	}

	RPI::DeviceState CanOpenMaster::getDeviceState() const
	{
		return canopen.getDeviceState();
	}

	void CanOpenMaster::setEStop(bool estop)
	{
	}

	bool CanOpenMaster::configureHook() {
		return true;
	}

	bool CanOpenMaster::startHook() {
		return true;
	}

	void CanOpenMaster::updateHook() {
		for(std::map<int, std::list<CanOpenNodeHandler*> >::iterator it =
				nodeHandlers.begin(), end = nodeHandlers.end(); it != end; ++it) {
			for(std::list<CanOpenNodeHandler*>::iterator it2 = it->second.begin(), end2 = it->second.end();
					it2 != end2; ++it2) {
				(*it2)->processData(it->first);
			}
		}

		for(node_pdo_iterator_t it = rx_pdo_mappings.begin(), end = rx_pdo_mappings.end(); it != end; ++it) {
			pdo_list_t list = it->second;
			int nodeID = it->first;
			int pdoID = 0;
			for(pdo_list_iterator_t it2 = list.begin(), end2 = list.end(); it2 != end2; ++it2) {
				pdo_entry_list_t entries = (*it2);
				uint8_t data[8];
				int pos = 0;
				for(pdo_entry_iterator_t it3 = entries.begin(), end3 = entries.end(); it3 != end3; ++it3) {
					PDOitem item = (*it3);
					int length = item.getLength();
					memcpy(&data[pos], item.getDataPointer(), length);
					pos += length;
				}
				canopen.writePDO(nodeID, pdoID, data, pos);
			}
			pdoID++;
		}
		canopen.sendSYNC();
	}

	void CanOpenMaster::nodeStateChanged(int nodeId, NodeState state)
	{
		for(std::list<CanOpenNodeHandler*>::iterator it = nodeHandlers[nodeId].begin(); it != nodeHandlers[nodeId].end(); ++it)
		{
			(*it)->nodeState(nodeId, state);
		}
	}

	void CanOpenMaster::nodeEmcy(int nodeId, int errorCode, int errorRegister)
	{
	}

	void CanOpenMaster::pdoReceived(int nodeId, int pdoId, const uint8* data, int len)
	{
		pdo_entry_list_t list = tx_pdo_mappings[nodeId][pdoId-1];
		int pos = 0;
		for(pdo_entry_iterator_t it = list.begin(), end = list.end(); it != end; ++it) {
			PDOitem item = (*it);
			int length = item.getLength();
			memcpy(item.getDataPointer(), data + pos, length);
			pos += length;
		}
	}

	void CanOpenMaster::calcPDOlist (pdo_entry_list_t& pdos, pdo_list_t& pdo_list) {
		int i = 1;
		while (!pdos.empty()) {
			//std::cout << "Creating PDO #" << i << std::endl;
			pdo_entry_list_t temp_list;
			int aggregated_size = 0;
			PDOitem *front = &pdos.front();
			temp_list.push_back(*front);
			aggregated_size += front->getLength();
			//std::cout << "adding 0x" << std::hex << front->getPDOadress() << std::dec << " with " << front->getLength() << " bytes." << std::endl;
			pdos.pop_front();
			for (std::list<PDOitem>::iterator it = pdos.begin(); it != pdos.end(); ) {
				//PDOitem *item = &(*it);
				int item_size = it->getLength();
				if ((aggregated_size + item_size) <= 8) {
					temp_list.push_back(*it);
					aggregated_size += item_size;
					//std::cout << "adding 0x" << std::hex << it->getPDOadress() << std::dec << " with " << it->getLength() << " bytes. Total PDO size: " << aggregated_size << std::endl;
					it = pdos.erase(it);
				} else {
					it++;
				}
			}
			pdo_list.push_back(temp_list);
			i++;
		}
	}

#define COBID_DISABLE_MASK 0x80000000

	void CanOpenMaster::setPDOs(int nodeID, int type, pdo_entry_list_t pdos) {
		//node_pdo_map& map = rx_pdo_mappings;
		pdo_list_t pdo_list;

		int rx_tx_val = 0;
		int pdo_config_index = 0;
		int pdo_data_index = 0;

		if (type == RX_PDO) {
			//map = rx_pdo_mappings;
			rx_tx_val = 0x0;
			pdo_config_index = 0x1400;
			pdo_data_index = 0x1600;
		} else if (type == TX_PDO) {
			//map = tx_pdo_mappings;
			rx_tx_val = 0x80;
			pdo_config_index = 0x1800;
			pdo_data_index = 0x1A00;
		} else
			return;

		calcPDOlist(pdos, pdo_list);

		int count = 0;
		for(pdo_list_iterator_t it = pdo_list.begin(), end = pdo_list.end(); it != end; ++it) {
			pdo_entry_list_t pdo_entries = (*it);
			int cob_id = ((0x100 * (count + 1)) | rx_tx_val) + nodeID;

			//std::cout << "*** PDO MAPPING: " << std::endl;
			//std::cout << "*** " << (type == RX_PDO ? "RECEIVE" : "TRANSMIT") << " " << std::endl;

			//std::cout << "*** CONFIG ADRESS: " << std::hex << (pdo_config_index + count) << std::dec << std::endl;
			//std::cout << "*** DATA ADRESS: " << std::hex << (pdo_data_index + count) << std::dec << std::endl;

			// DISABLE PDO
			//std::cout << std::hex << "\tDISABLE: " << canopen.writeSDO(nodeID, pdo_config_index + count, 0x1, cob_id | COBID_DISABLE_MASK, 4) << std::endl;
			canopen.writeSDO(nodeID, pdo_config_index + count, 0x1, cob_id | COBID_DISABLE_MASK, 4);
			// SET COUNT TO ZERO
			//std::cout << "\tCOUNTZERO: " << canopen.writeSDO(nodeID, pdo_data_index + count, 0x0, 0x0, 1) << std::endl;
			canopen.writeSDO(nodeID, pdo_data_index + count, 0x0, 0x0, 1);

			int item_count = 1;
			for(pdo_entry_iterator_t it2 = pdo_entries.begin(), end2 = pdo_entries.end(); it2 != end2; ++it2) {
				PDOitem item = (*it2);
				//std::cout << "\t\tPDO ADRESS AT INDEX " << item_count << ": " << std::hex << item.getPDOadress() << std::dec << " :: ";
				//std::cout << std::hex << canopen.writeSDO(nodeID, pdo_data_index + count, item_count, item.getPDOadress(), 4) << std::dec << std::endl;
				canopen.writeSDO(nodeID, pdo_data_index + count, item_count, item.getPDOadress(), 4);
				item_count++;
			}

			// SET ACTUAL COUNT
			//std::cout << "\tACTUAL COUNT: " << (item_count - 1) << " :: " << canopen.writeSDO(nodeID, pdo_data_index + count, 0x0, (item_count - 1), 1) << std::endl;
			canopen.writeSDO(nodeID, pdo_data_index + count, 0x0, (item_count - 1), 1);
			// SET TRANSMIT TYPE
			//std::cout << std::hex << "\tTRANSMIT TYPE: " << canopen.writeSDO(nodeID, pdo_config_index + count, 0x2, 0x1, 1) << std::endl;
			canopen.writeSDO(nodeID, pdo_config_index + count, 0x2, 0x1, 1);
			// REENABLE PDO
			//std::cout << "\tREENABLE: " << canopen.writeSDO(nodeID, pdo_config_index + count, 0x1, cob_id, 4) << std::endl;
			canopen.writeSDO(nodeID, pdo_config_index + count, 0x1, cob_id, 4);

			count++;

		}


		if (count < 4) {
			for (int i=count; i<4; i++) {
				//std::cout << canopen.writeSDO(nodeID, pdo_config_index + i, 0x1, COBID_DISABLE_MASK, 4) << std::endl;
				canopen.writeSDO(nodeID, pdo_config_index + i, 0x1, COBID_DISABLE_MASK, 4);
			}
		}
		//0x80000000 (enable/disable)
					//0x00000100 * (i+1)
					// (|=0x80) + nodeId

		if(type == TX_PDO) {
			tx_pdo_mappings[nodeID] = pdo_list;
			for(int i=1; i<=count; i++) {
				canopen.addPDOListener(nodeID, i, this);
			}
		} else if (type == RX_PDO) {
			rx_pdo_mappings[nodeID] = pdo_list;
		}
		std::cout  << std::dec;
	}

	void CanOpenMaster::setNodeMode(int nodeId, NodeMode mode) {
		canopen.setNodeMode(nodeId, mode);
	}

	/*void CanOpenMaster::configureTxPDO(int nodeId, int pdoId, CanOpenPdoConfiguration& config, void* data, int len)
	{
		canopen.removePDOListener(nodeId, pdoId, this);
		canopen.addPDOListener(nodeId, pdoId, this);
		// TODO: configure node TxPDO according to config

		PDOJob job; job.nodeId = nodeId; job.pdoId = pdoId; job.data = data; job.len = len;
		TxPDOs.push_back(job);
	}

	void CanOpenMaster::clearTxPDO(int nodeId, int pdoId)
	{
		for(std::list<PDOJob>::iterator it = TxPDOs.begin(); it != TxPDOs.end();) {
			if(nodeId == it->nodeId && pdoId == it->pdoId) {
				it = TxPDOs.erase(it);
			} else {
				++it;
			}
		}
		canopen.removePDOListener(nodeId, pdoId, this);
	}


	void CanOpenMaster::configureRxPDO(int nodeId, int pdoId, CanOpenPdoConfiguration& config, void* data, int len)
	{
		// TODO: configure node RxPDO according to config

		PDOJob job; job.nodeId = nodeId; job.pdoId = pdoId; job.data = data; job.len = len;
		RxPDOs.push_back(job);
	}

	void CanOpenMaster::clearRxPDO(int nodeId, int pdoId)
	{
		for(std::list<PDOJob>::iterator it = RxPDOs.begin(); it != RxPDOs.end();) {
			if(nodeId == it->nodeId && pdoId == it->pdoId) {
				it = RxPDOs.erase(it);
			} else {
				++it;
			}
		}
	}*/


	int CanOpenMaster::writeSDO(int nodeId, int index, int subIndex, int value, int len)
	{
		return canopen.writeSDO(nodeId, index, subIndex, value, len);
	}
	int CanOpenMaster::readSDO(int nodeId, int index, int subIndex, int& value, int& len)
	{
		return canopen.readSDO(nodeId, index, subIndex, value, len);
	}

	void CanOpenMaster::addNodeHandler(int nodeId, CanOpenNodeHandler* handler)
	{
		nodeHandlers[nodeId].push_back(handler);
	}

	void CanOpenMaster::removeNodeHandler(int nodeId, CanOpenNodeHandler* handler)
	{
		nodeHandlers[nodeId].remove(handler);
	}


} /* namespace canopen */
