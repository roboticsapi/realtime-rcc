/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef SCHUNKLWADRIVER_HPP_
#define SCHUNKLWADRIVER_HPP_

#include <rcc/Device.hpp>
#include <rcc/DeviceInstanceT.hpp>
#include <libs/framework/canopen/interface/CanOpenMasterInterface.hpp>
#include <libs/framework/robotarm/device/CyclicPositionRobotArm.hpp>
#include <libs/framework/armkinematics/interface/ArmKinematicsInterface.hpp>
#include <rtt/TaskContext.hpp>

#include "../kinematics/Schunk_KinGeo.hpp"

namespace schunk_lwa {

	const std::string dev_schunk_lwa_can = "schunk_lwa_can";
	const int NUM_AXES = 6;
	const int NODE_OFFSET = 3;

	typedef enum slave_state_t {
		SLAVE_UNKNOWN,
		SLAVE_START,
		SLAVE_NOT_READY_TO_SWITCH_ON,
		SLAVE_SWITCH_ON_DISABLED,
		SLAVE_READY_TO_SWITCH_ON,
		SLAVE_SWITCHED_ON,
		SLAVE_OPERATION_ENABLE,
		SLAVE_QUICK_STOP_ACTIVE,
		SLAVE_FAULT
	} slave_state;

	typedef enum node_next_hook_state_t {
		NODE_RESET,
		NODE_INTERPOLATED_POSITION,
		NODE_HOMING,
		NODE_SWITCH_OFF
	} node_next_hook_state;

	typedef enum node_current_state_t {
		NODE_CURRENT_NONE = 0,
		NODE_CURRENT_RESET_WAIT,
		NODE_CURRENT_RESET_ERROR,
		NODE_CURRENT_RESET,
		NODE_CURRENT_INTERPOLATED_POSITION_SETUP,
		NODE_CURRENT_INTERPOLATED_POSITION_ERROR,
		NODE_CURRENT_INTERPOLATED_POSITION,
		NODE_CURRENT_HOMING_WAIT,
		NODE_CURRENT_HOMING_ERROR,
		NODE_CURRENT_HOMING_DONE,
		NODE_CURRENT_SWITCHOFF_WAIT,
		NODE_CURRENT_SWITCHOFF_ERROR,
		NODE_CURRENT_SWITCHOFF
	} node_current_hook_state;

	class SchunkLwaDriver:
		public virtual RPI::Device,
		canopen::CanOpenNodeHandler,
		public robotarm::CyclicPositionRobotArm,
		virtual public armkinematics::ArmKinematicsInterface,
		RTT::TaskContext
	{

		friend class SchunkLWAHandler;

		public:
			SchunkLwaDriver(std::string name, RPI::parameter_t parameters);
			virtual ~SchunkLwaDriver();

			// RPI::Device
			static SchunkLwaDriver* createDevice(std::string name, RPI::parameter_t parameters);
			void updateParameters();
			std::set<std::string> getMutableParameters() const;
			RPI::DeviceState getDeviceState() const;
			void setEStop(bool estop);

			virtual void nodeState(int nodeId, canopen::NodeState state);
			virtual void processData(int nodeId);

			static double radValue (int milliDeg) {
				return ((double) milliDeg) / 1000 / 180 * KDL::PI;
			}

			static int milliDegValue (double rad) {
				return rad / KDL::PI * 1000 * 180;
			}

			// Interface ArmKinematics
			KDL::Frame Kin(const RPI::Array<double>& joints);
			void InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position, RPI::Array<double>& values);

			// Methods for CyclicPositionRobotArm
			virtual int getJointCount() const;
			virtual double getMeasuredJointPosition(int joint);
			virtual int getToolError(int axis) const;
			virtual void setToolMass(double mass, int axis);
			virtual double getMaximumVelocity(int joint) const;
			virtual int getJointError(int joint);
			virtual void setToolCOM(KDL::Vector com, int axis);
			virtual void setToolMOI(KDL::Vector moi, int axis);
			virtual double getMeasuredJointVelocity(int joint);
			virtual double getMaximumAcceleration(int joint) const;
			virtual robotarm::JointPositionError checkJointPosition(int joint, double position);
			virtual double getMeasuredJointAcceleration(int joint);
			virtual bool getToolFinished(int axis) const;

			// Methods for homing mode
			void homeJoints ();
			void homeJoint (int joint);

			// Methods for interpolated position mode
			void startInterpolatedPositionMode();
			void stopInterpolatedPositionMode();

			void setAllJointHookStates (node_next_hook_state state);

			template <typename T>
			void setAllJointPDOvalues (T array[], T value);

		protected:
			virtual bool configureHook();
			virtual bool startHook();
			virtual void updateHook();


		private:
			dh_parameters dh_param;
			Schunk_KinGeo_Calculation *kin_calc;

			uint8_t ip_counter;
			uint8_t homing_counter;

			void hookHandleReset(int i);
			void hookHandleChangeMode(int i);
			void hookSetOperational(int i);
			void hookHandleInterpolatedPosition(int i);
			void hookHandleHoming(int i);
			void hookHandleSwitchOff(int i);

			// RPI device instance of the CANopen driver
			RPI::DeviceInstanceT<canopen::CanOpenMasterInterface> schunklwa;

			node_next_hook_state commanded_hook_states[NUM_AXES];
			node_current_hook_state current_hook_states[NUM_AXES];

			slave_state last_known_state[NUM_AXES];

			// Storage Arrays for values of RxPDOs
			uint16_t statusword[NUM_AXES];
			uint16_t statusword_old[NUM_AXES];
			int16_t torque_actual[NUM_AXES];
			int16_t torque_actual_old[NUM_AXES];
			int32_t position_actual[NUM_AXES];
			int32_t position_actual_old[NUM_AXES];
			int32_t velocity_demand[NUM_AXES];
			int32_t velocity_demand_old[NUM_AXES];
			int32_t velocity_actual[NUM_AXES];
			int32_t velocity_actual_old[NUM_AXES];

			// Storage Arrays for values of TxPDOs
			uint16_t controlword[NUM_AXES];
			uint16_t controlword_old[NUM_AXES];
			int32_t interpolated_position[NUM_AXES];
			int32_t interpolated_position_old[NUM_AXES];
			//uint32_t profile_velocity[6];

			bool was_ip_mode_active[NUM_AXES];

			RPI::DeviceState current_state;

			bool EMGCY_STOP;
	};

} /* namespace schunksdh */
#endif /* SCHUNKLWADRIVER_HPP_ */
