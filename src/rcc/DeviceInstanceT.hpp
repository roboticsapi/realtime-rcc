/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef DEVICEINSTANCET_HPP_
#define DEVICEINSTANCET_HPP_

#include "DeviceInstanceFwd.hpp"
#include <rtt/rtt-config.h>
#include "Device.hpp"

namespace RPI {

	/**
	 * \brief Abstract interface for DeviceInstance
	 * \see DeviceInstanceT
	 */
	class RTT_EXPORT DeviceInstance
	{
	public:
		virtual bool fetchInstance(const std::string& deviceid) = 0;

	protected:
		std::string name;

		DeviceInstance();
		virtual ~DeviceInstance();

		Device* getAndAcquireDevice();
		void releaseDevice();

	};

	/**
	 * \brief Template implementation for controlled device access
	 *
	 * Instead of directly accessing a device, an instance of DeviceInstanceT<Device> should be
	 * used. DeviceInstanceT takes care of registering an unregistering the usage of the Device.
	 * Without registering a device, it might be unloaded and any pointer to the device will be invalid.
	 * There sould be no Device* pointers kept, but DeviceInstanceT->getDevice() used instead.
	 * The device usage will be unregistered as soon as the instance of DeviceInstanceT is destroyed.
	 */
	template<class T>
	class RTT_EXPORT DeviceInstanceT: public DeviceInstance
	{
	private:
		T* device;

	public:
		/**
		 * \brief Create new, empty DeviceInstance
		 *
		 * Using this constructor, a new DeviceInstance will be created, but no device will
		 * be available before fetchInstance has been called. This constructor is intended for use,
		 * when the name of the desired device is not yet known in constructor (e.g. when name
		 * is extracted from property in RPI module)
		 */
		DeviceInstanceT() :
				DeviceInstance(), device(0)
		{

		}

		/**
		 * \brief Create new DeviceInstance with immediate device registering
		 * \param deviceid ID of the desired device
		 *
		 * Using this constructor, a new DeviceInstance will be created, and the desired
		 * device will be available immediately. However, if no device with the desired deviceid can
		 * be found, getDevice() will return null pointer.
		 */
		DeviceInstanceT(const std::string& deviceid) :
				DeviceInstance(), device(0)
		{
			name = deviceid;
			fetchInstance(deviceid);
		}

		virtual ~DeviceInstanceT()
		{
			if(device)
				releaseDevice();
		}

		/**
		 * \brief Fetch instance of device and register device usage
		 * \param deviceid ID of the desired device
		 * \return true, if desired device was found and usage registered
		 *
		 * This method must be called to acquire a device for usage if the DeviceInstance object
		 * has been created without specifying a deviceid. The return value specifies whether
		 * a device with the desired id has been found. If a device with the given name exists,
		 * but its type is not compatible with the DeviceInstance's type T, false will be returned
		 * as well.
		 */
		virtual bool fetchInstance(const std::string& deviceid)
		{
			// we already have a device, do not fetch a new one
			if (device != 0)
				return false;

			name = deviceid;
			device = dynamic_cast<T*>(getAndAcquireDevice());

			return (device != 0);
		}

		/**
		 * \brief access device
		 * \return pointer to device, should not be stored anywhere
		 *
		 * getDevice() returns a pointer to the desired device. In order to avoid invalid pointers,
		 * no copy of the returned pointer should be stored. If no device has been acquired, a null
		 * pointer is returned.
		 */
		T* getDevice() const
		{
			return device;
		}

		/**
		 * \brief access device
		 * \return pointer to device
		 *
		 * Does the same as getDevice(), but with less typing involved ...
		 */
		T* operator->() const
		{
			return device;
		}
	};

}

#endif /* DEVICEINSTANCET_HPP_ */
