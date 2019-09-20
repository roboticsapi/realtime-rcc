/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef CANOPENUTIL_H_
#define CANOPENUTIL_H_

namespace schunk_lwa {

	class CANopenUtil {

		public:
			CANopenUtil();

			static bool bitVerify (int number, int bit, bool value);

			static bool bitsVerify (int number, int numBits, ...);

			static bool bitSet (int number, int bit) ;

			static bool bitsSet (int number, int numBits, ...);

	};

} /* namespace schunklwa */
#endif /* CANOPENUTIL_H_ */
