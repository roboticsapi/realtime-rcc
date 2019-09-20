/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef ETHERCATTYPES_HPP_
#define ETHERCATTYPES_HPP_

#include <stdint.h>

/* General types  for EtherCAT */
typedef int8_t              int8;
typedef int16_t             int16;
typedef int32_t             int32;
typedef uint8_t             uint8;
typedef uint16_t            uint16;
typedef uint32_t            uint32;
typedef int64_t             int64;
typedef uint64_t            uint64;
typedef float               float32;
typedef double              float64;


// TODO: Maybe SOEM specific?
/** max. mailbox size */
#define EC_MAXMBX         0x3ff
/** mailbox buffer array */
typedef uint8 ec_mbxbuft[EC_MAXMBX + 1];


#endif /* ETHERCATTYPES_HPP_ */
