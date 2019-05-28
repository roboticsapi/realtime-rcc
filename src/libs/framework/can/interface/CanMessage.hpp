/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CANMESSAGE_HPP_
#define CANMESSAGE_HPP_

namespace can
{
	typedef unsigned char  uint8;
	typedef unsigned short uint16;

	static const uint8 CanMessageDataSize = 8;

	typedef struct CanMessage {
		uint8 length:4;
		uint8 rtr:1;
		uint16 message_id:11;
		uint8 data[CanMessageDataSize];
	} __attribute__((__packed__)) CanMessage;

} /* namespace can */
#endif /* CANMESSAGE_HPP_ */
