/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef STATUSCODE_HPP_
#define STATUSCODE_HPP_

namespace schunkwsg {

	//! Status codes
	typedef enum
	{
		E_SUCCESS = 0, //!< No error
		E_NOT_AVAILABLE, //!< Device, service or data is not available
		E_NO_SENSOR, //!< No sensor connected
		E_NOT_INITIALIZED, //!< The device is not initialized
		E_ALREADY_RUNNING, //!< Service is already running
		E_FEATURE_NOT_SUPPORTED, //!< The asked feature is not supported
		E_INCONSISTENT_DATA, //!< One or more dependent parameters mismatch
		E_TIMEOUT, //!< Timeout error
		E_READ_ERROR, //!< Error while reading from a device
		E_WRITE_ERROR, //!< Error while writing to a device
		E_INSUFFICIENT_RESOURCES, //!< No memory available
		E_CHECKSUM_ERROR, //!< Checksum error
		E_NO_PARAM_EXPECTED, //!< No parameters expected
		E_NOT_ENOUGH_PARAMS, //!< Not enough parameters
		E_CMD_UNKNOWN, //!< Unknown command
		E_CMD_FORMAT_ERROR, //!< Command format error
		E_ACCESS_DENIED, //!< Access denied
		E_ALREADY_OPEN, //!< The interface is already open
		E_CMD_FAILED, //!< Command failed
		E_CMD_ABORTED, //!< Command aborted
		E_INVALID_HANDLE, //!< invalid handle
		E_NOT_FOUND, //!< device not found
		E_NOT_OPEN, //!< device not open
		E_IO_ERROR, //!< I/O error
		E_INVALID_PARAMETER, //!< invalid parameter
		E_INDEX_OUT_OF_BOUNDS, //!< index out of bounds
		E_CMD_PENDING, //!< Command execution needs more time
		E_OVERRUN, //!< Data overrun
		E_RANGE_ERROR, //!< Range error
		E_AXIS_BLOCKED, //!< Axis is blocked
		E_FILE_EXISTS //!< File already exists
	} StatusCode;

	static const char* StatusCodeNames[E_FILE_EXISTS + 1] = {"E_SUCCESS", //!< No error
			"E_NOT_AVAILABLE", //!< Device, service or data is not available
			"E_NO_SENSOR", //!< No sensor connected
			"E_NOT_INITIALIZED", //!< The device is not initialized
			"E_ALREADY_RUNNING", //!< Service is already running
			"E_FEATURE_NOT_SUPPORTED", //!< The asked feature is not supported
			"E_INCONSISTENT_DATA", //!< One or more dependent parameters mismatch
			"E_TIMEOUT", //!< Timeout error
			"E_READ_ERROR", //!< Error while reading from a device
			"E_WRITE_ERROR", //!< Error while writing to a device
			"E_INSUFFICIENT_RESOURCES", //!< No memory available
			"E_CHECKSUM_ERROR", //!< Checksum error
			"E_NO_PARAM_EXPECTED", //!< No parameters expected
			"E_NOT_ENOUGH_PARAMS", //!< Not enough parameters
			"E_CMD_UNKNOWN", //!< Unknown command
			"E_CMD_FORMAT_ERROR", //!< Command format error
			"E_ACCESS_DENIED", //!< Access denied
			"E_ALREADY_OPEN", //!< The interface is already open
			"E_CMD_FAILED", //!< Command failed
			"E_CMD_ABORTED", //!< Command aborted
			"E_INVALID_HANDLE", //!< invalid handle
			"E_NOT_FOUND", //!< device not found
			"E_NOT_OPEN", //!< device not open
			"E_IO_ERROR", //!< I/O error
			"E_INVALID_PARAMETER", //!< invalid parameter
			"E_INDEX_OUT_OF_BOUNDS", //!< index out of bounds
			"E_CMD_PENDING", //!< Command execution needs more time
			"E_OVERRUN", //!< Data overrun
			"E_RANGE_ERROR", //!< Range error
			"E_AXIS_BLOCKED", //!< Axis is blocked
			"E_FILE_EXISTS"};

}  /* namespace schunkwsg */
#endif /* STATUSCODE_HPP_ */
