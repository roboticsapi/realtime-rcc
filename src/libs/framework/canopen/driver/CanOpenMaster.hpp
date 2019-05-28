/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CANOPENMASTER_HPP_
#define CANOPENMASTER_HPP_

#include <rtt/TaskContext.hpp>
#include <rcc/Device.hpp>
#include "CanOpenDevice.hpp"
#include "../interface/CanOpenMasterInterface.hpp"

namespace canopen
{
	const std::string dev_canopenmaster = "canopenmaster";

	struct PDOJob {
		int nodeId;
		int pdoId;
		void* data;
		int len;
	};

	class CanOpenMaster: public virtual RPI::Device, public RTT::TaskContext, public canopen::NodeStateListener, public canopen::PDOListener, public canopen::CanOpenMasterInterface
	{
	public:

		CanOpenMaster(std::string name, RPI::parameter_t parameters);
		virtual ~CanOpenMaster();

		// RPI::Device
		static CanOpenMaster* createDevice(std::string name, RPI::parameter_t parameters);
		void updateParameters();
		std::set<std::string> getMutableParameters() const;
		RPI::DeviceState getDeviceState() const;
		void setEStop(bool estop);

		virtual void addNodeHandler(int nodeId, CanOpenNodeHandler* handler);
		virtual void removeNodeHandler(int nodeId, CanOpenNodeHandler* handler);


		//virtual void clearTxPDO(int nodeId) = 0;
		//virtual void addTxPDO(int nodeId, int index, int subindex, int len, void* data) = 0;
		//virtual void clearRxPDO(int nodeId) = 0;
		//virtual void addRxPDO(int nodeId, int index, int subindex, int len, void* data) = 0;

		//virtual void clearPDOs(int nodeID, int type);
		virtual void setPDOs(int nodeID, int type, std::list<PDOitem> pdos);

//		virtual void configureTxPDO(int nodeId, int pdoId, CanOpenPdoConfiguration& config, void* data, int len);
//		virtual void clearTxPDO(int nodeId, int pdoId);
//		virtual void configureRxPDO(int nodeId, int pdoId, CanOpenPdoConfiguration& config, void* data, int len);
//		virtual void clearRxPDO(int nodeId, int pdoId);

		virtual int writeSDO(int nodeId, int index, int subIndex, int value, int len);
		virtual int readSDO(int nodeId, int index, int subIndex, int& value, int& len);
		virtual void setNodeMode(int nodeId, NodeMode mode);

		virtual void nodeStateChanged(int nodeId, NodeState state);
		virtual void nodeEmcy(int nodeId, int errorCode, int errorRegister);

		virtual void pdoReceived(int nodeId, int pdoId, const uint8* data, int len);

	protected:
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();

	private:
		CanOpenDevice canopen;
		void calcPDOlist (pdo_entry_list_t& pdos, pdo_list_t& pdo_list);
		std::map<int, std::list<CanOpenNodeHandler*> > nodeHandlers;
		//std::list<PDOJob> RxPDOs;
		//std::list<PDOJob> TxPDOs;
		node_pdo_map rx_pdo_mappings, tx_pdo_mappings;
	};

} /* namespace canopen */
#endif /* CANOPENMASTER_HPP_ */
