/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#ifndef LBR_HPP_
#define LBR_HPP_

#include <boost/algorithm/string.hpp>

#include "../device/LwrDevice.hpp"

#include <rcc/Extension.hpp>
#include <rcc/DeviceFactory.hpp>
#include <rcc/DeviceInstanceT.hpp>

namespace kuka_lwr
{

	/**
	 * This module switches the current control strategy of the LWR
	 * This requires a special KRL program running on the LWR controller
	 */
	class ControlStrategy: public RPI::ActiveModule
	{

	public:
		ControlStrategy(std::string name, RPI::Net* net);

		~ControlStrategy();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::set<std::string> getResourceNames() const;
		std::string getDeviceName();
		bool isActuator();
		void updateActuator();

	protected:
		RPI::InPort<int> inStrategy;
		RPI::OutPort<bool> outCompleted;
		RPI::OutPort<int> outError;

		RPI::Property<std::string> propRobot;

	private:
		RPI::DeviceInstanceT<LwrDevice> devins;

	};

	class ToolParameters: public RPI::ActiveModule
	{
	public:
		ToolParameters(std::string name, RPI::Net* net);
		~ToolParameters();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::set<std::string> getResourceNames() const;
		std::string getDeviceName();
		bool isActuator();
		void updateActuator();
	protected:
		RPI::InPort<KDL::Frame> inTCP; ///< coordinates of tool center point in flange coordinate system

		RPI::InPort<double> inMass; ///< mass of load (in kg)
		RPI::InPort<KDL::Frame> inCOM; ///< coordinates of center of mass in flange coordinate system
		RPI::InPort<KDL::Vector> inMOI; ///< moment of intertia in x, y and z direction

		RPI::OutPort<bool> outCompleted;
		RPI::OutPort<int> outError;

		RPI::Property<std::string> propRobot;

	private:
		RPI::DeviceInstanceT<LwrDevice> devins;
	};

	/**
	 * This class implements a module to update joint stiffness, damping and additional torque parameters
	 */
	class JointImpParameters: public RPI::ActiveModule
	{

	public:
		JointImpParameters(std::string name, RPI::Net* net);
		~JointImpParameters();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::set<std::string> getResourceNames() const;
		std::string getDeviceName();
		bool isActuator();
		void updateActuator();
	protected:
		RPI::InPort<double> inStiffness;
		RPI::InPort<double> inDamping;
		RPI::InPort<double> inAddTorque;
		RPI::Property<std::string> propRobot;
		RPI::Property<int> propAxis;

	private:
		RPI::DeviceInstanceT<LwrDevice> devins;
		bool hasValues;
	};

	/**
	 * This class implements a module to update cartesian stiffness, damping and additional torque parameters
	 */
	class CartImpParameters: public RPI::ActiveModule
	{

	public:
		CartImpParameters(std::string name, RPI::Net* net);
		~CartImpParameters();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::set<std::string> getResourceNames() const;
		std::string getDeviceName();
		bool isActuator();
		void updateActuator();
	protected:
		RPI::InPort<KDL::Wrench> inStiffness;
		RPI::InPort<KDL::Wrench> inDamping;
		RPI::InPort<KDL::Wrench> inAddTorque;
		RPI::Property<std::string> propRobot;
	private:
		RPI::DeviceInstanceT<LwrDevice> devins;
		bool hasValues;
	};

	/**
	 * This class implements a Force monitor for the LWR robot
	 */
	class FMonitor: public RPI::Module
	{

	public:
		FMonitor(std::string name, RPI::Net* net);
		~FMonitor();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isSensor();
		void updateSensor();
	protected:
		RPI::OutPort<double> outTcpFx;
		RPI::OutPort<double> outTcpFy;
		RPI::OutPort<double> outTcpFz;
		RPI::OutPort<double> outTcpTz;
		RPI::OutPort<double> outTcpTy;
		RPI::OutPort<double> outTcpTx;
		RPI::Property<std::string> propRobot;
	private:
		RPI::DeviceInstanceT<LwrDevice> devins;
	};

	/**
	 * This class implements a Torque monitor for an axis of the LWR robot
	 */
	class TMonitor: public RPI::Module
	{

	public:
		TMonitor(std::string name, RPI::Net* net);
		~TMonitor();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isSensor();
		void updateSensor();
	protected:
		RPI::OutPort<double> outMsr;
		RPI::OutPort<double> outEst;
		RPI::Property<std::string> propRobot;
		RPI::Property<int> propAxis;
	private:
		RPI::DeviceInstanceT<LwrDevice> devins;
	};

	/**
	 * This class implements a ForceTorque monitor for the LBR robot
	 */
	class FTMonitor: public RPI::Module
	{

	public:
		FTMonitor(std::string name, RPI::Net* net);
		~FTMonitor();
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		std::string getDeviceName();
		bool isSensor();
		void updateSensor();
	protected:
		RPI::OutPort<double> outMsrJ1;
		RPI::OutPort<double> outMsrJ2;
		RPI::OutPort<double> outMsrJ3;
		RPI::OutPort<double> outMsrJ4;
		RPI::OutPort<double> outMsrJ5;
		RPI::OutPort<double> outMsrJ6;
		RPI::OutPort<double> outMsrJ7;
		RPI::OutPort<double> outEstJ1;
		RPI::OutPort<double> outEstJ2;
		RPI::OutPort<double> outEstJ3;
		RPI::OutPort<double> outEstJ4;
		RPI::OutPort<double> outEstJ5;
		RPI::OutPort<double> outEstJ6;
		RPI::OutPort<double> outEstJ7;
		RPI::OutPort<double> outTcpFx;
		RPI::OutPort<double> outTcpFy;
		RPI::OutPort<double> outTcpFz;
		RPI::OutPort<double> outTcpTz;
		RPI::OutPort<double> outTcpTy;
		RPI::OutPort<double> outTcpTx;
		RPI::Property<std::string> propRobot;
	private:
		RPI::DeviceInstanceT<LwrDevice> devins;
	};

	class InvKin: public RPI::ActiveModule
	{
	public:
		InvKin(std::string name, RPI::Net* net);

		/**
		 * This function is for the configuration code.
		 * Return false to abort configuration.
		 */
		bool configureHook();
		/**
		 * This function is for the application's startup code.
		 * Return false to abort startup.
		 */
		bool startHook();
		/**
		 * This function is periodically called.
		 */
		void updateHook();

		/**
		 * This function is called when the task is stopped.
		 */
		void stopHook();
		/**
		 * This function is called when the task is being deconfigured.
		 */
		void cleanupHook();
	protected:
		RPI::InPort<KDL::Frame> inFrame;
		RPI::InPort<double> inHintJ1, inHintJ2, inHintJ3, inHintJ4, inHintJ5, inHintJ6, inHintJ7;
		RPI::InPort<double> inNullspaceJ1, inNullspaceJ2, inNullspaceJ3, inNullspaceJ4, inNullspaceJ5, inNullspaceJ6,
				inNullspaceJ7;
		RPI::OutPort<double> outJ1, outJ2, outJ3, outJ4, outJ5, outJ6, outJ7;
		RPI::Property<int> propStrategy;
		RPI::Property<std::string> propRobot;
	private:
			RPI::DeviceInstanceT<LwrDevice> devins;

	};

	class Kin: public RPI::ActiveModule
	{
	public:
		Kin(std::string name, RPI::Net* net);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	protected:
		RPI::InPort<double> inJ1, inJ2, inJ3, inJ4, inJ5, inJ6, inJ7;
		RPI::OutPort<KDL::Frame> outFrame;
		RPI::OutPort<double> outAlpha;
		RPI::Property<std::string> propRobot;
	private:
			RPI::DeviceInstanceT<LwrDevice> devins;

	};

	class VelKin: public RPI::ActiveModule
	{
	public:
		VelKin(std::string name, RPI::Net* net);

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
	protected:
		RPI::InPort<double> inJ1, inJ2, inJ3, inJ4, inJ5, inJ6, inJ7;
		RPI::InPort<double> inV1, inV2, inV3, inV4, inV5, inV6, inV7;
		RPI::OutPort<KDL::Twist> outTwist;
		RPI::OutPort<double> outVAlpha;
		RPI::Property<std::string> propRobot;
	private:
			RPI::DeviceInstanceT<LwrDevice> devins;

	};

}

#endif /* LBR_HPP_ */
