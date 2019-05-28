/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "CANopenUtil.hpp"
#include <stdarg.h>

namespace schunk_lwa {

	CANopenUtil::CANopenUtil() {

	}

	bool CANopenUtil::bitVerify (int number, int bit, bool value) {
		int result = (number & (0x01 << bit));
		return (value ? result != 0 : result == 0);
	}

	bool CANopenUtil::bitsVerify (int number, int numBits, ...) {
		if (numBits) {
			va_list bits;
			int arg;
			int val;
			va_start(bits, numBits);
			while (numBits--) {
				/*int *p = &numBits;
								p++;*/
				arg = va_arg(bits, int);
				val = va_arg(bits, int);
				if (!bitVerify(number, arg, val))
					return false;
			}
			va_end(bits);
			return true;
		} else {
			return false;
		}
	}

	bool CANopenUtil::bitSet (int number, int bit) {
		return bitVerify(number, bit, 1);
	}

	bool CANopenUtil::bitsSet (int number, int numBits, ...) {
		if (numBits) {
			va_list bits;
			int arg;
			va_start(bits, numBits);
			while (numBits--) {
				/*int *p = &numBits;
							p++;*/
				arg = va_arg(bits, int);
				//if ((number & (0x01 << arg)) == 0)
				if (!bitVerify(number, arg, 1))
					return false;
			}
			va_end(bits);
			return true;
		} else {
			return false;
		}
	}

} /* namespace schunklwa */
