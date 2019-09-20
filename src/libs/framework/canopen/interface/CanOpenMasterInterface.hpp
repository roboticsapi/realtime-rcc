/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CANOPENMASTERINTERFACE_HPP_
#define CANOPENMASTERINTERFACE_HPP_

#include "../interface/CanOpenInterface.hpp"

namespace canopen {
	typedef unsigned char  uint8;

	const int RX_PDO = 0;
	const int TX_PDO = 1;


	class CanOpenNodeHandler
	{
	public:
		virtual ~CanOpenNodeHandler() {};
		virtual void nodeState(int nodeId, NodeState state) = 0;
		virtual void processData(int nodeId) = 0;
	};

	class PDOitem {
			public:
				PDOitem(int index, int subindex, int length, void *data) {
					this->index = index;
					this->subindex = subindex;
					this->length = length;
					this->data = data;
					this->pdo_adress = ((this->index << 16) | (this->subindex << 8) | (this->length << 3));
					//this->pdo_adress = (this->index << 16) | (1 + this->subindex << 4);
				}

				int getIndex () {
					return this->index;
				}

				int getSubIndex () {
					return this->subindex;
				}

				int getPDOadress () {
					return this->pdo_adress;
				}

				int getLength () {
					return this->length;
				}

				void * getDataPointer () {
					return this->data;
				}

			private:
				int index;
				int subindex;
				int pdo_adress;
				int length;
				void *data;
		};

	template <typename T>
	class PDOitem_T : public PDOitem {
		public:
			PDOitem_T(int index, int subindex, T *data) : PDOitem(index, subindex, sizeof(T), ((void *) data)) {
				return;
			}
	};

	typedef std::list<PDOitem> pdo_entry_list_t;
	typedef std::vector<pdo_entry_list_t > pdo_list_t;
	typedef std::map<int, pdo_list_t > node_pdo_map;
	typedef node_pdo_map::const_iterator node_pdo_iterator_t;
	typedef pdo_list_t::const_iterator pdo_list_iterator_t;
	typedef pdo_entry_list_t::const_iterator pdo_entry_iterator_t;

	class CanOpenMasterInterface {
		public:
			virtual ~CanOpenMasterInterface() {};

			virtual void addNodeHandler(int nodeId, CanOpenNodeHandler* handler) = 0;
			virtual void removeNodeHandler(int nodeId, CanOpenNodeHandler* handler) = 0;

			virtual void setPDOs(int nodeID, int type, pdo_entry_list_t pdos) = 0;

			virtual int writeSDO(int nodeId, int index, int subIndex, int value, int len) = 0;
			virtual int readSDO(int nodeId, int index, int subIndex, int& value, int& len) = 0;
			virtual void setNodeMode(int nodeId, NodeMode mode) = 0;
	};

}

#endif /* CANOPENMASTERINTERFACE_HPP_ */
