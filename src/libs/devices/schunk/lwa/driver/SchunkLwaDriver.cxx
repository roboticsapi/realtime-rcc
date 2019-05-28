/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "SchunkLwaDriver.hpp"
#include "../util/CANopenUtil.hpp"

#include <rtt/Logger.hpp>
#include <rtt/Activity.hpp>

namespace schunk_lwa
{

	node_next_hook_state commanded_hook_states[NUM_AXES] = {
			NODE_RESET,
			NODE_RESET,
			NODE_RESET,
			NODE_RESET,
			NODE_RESET,
			NODE_RESET
	};

	node_current_hook_state current_hook_states[NUM_AXES] = {
			NODE_CURRENT_NONE,
			NODE_CURRENT_NONE,
			NODE_CURRENT_NONE,
			NODE_CURRENT_NONE,
			NODE_CURRENT_NONE,
			NODE_CURRENT_NONE
	};

	SchunkLwaDriver::SchunkLwaDriver(std::string name, RPI::parameter_t parameters) :
			RTT::TaskContext(name, PreOperational),
			RPI::Device(name, parameters), schunklwa(getParameter("schunklwa", "")),
			robotarm::CyclicPositionRobotArm(NUM_AXES, 0), EMGCY_STOP(false)
	{
		current_state = RPI::DeviceState::OFFLINE;
		std::cout << "************* Building Schunk LWA" << std::endl;
		canopen::CanOpenMasterInterface *can = schunklwa.getDevice();
		if(can == 0) return;
		this->setActivity(new RTT::Activity(RTT::os::LowestPriority, 0, 0, name));

		// Kinematics
		dh_param.dh_d = RPI::Module::parseString<double>(getParameter("dh_d"));
		dh_param.dh_t = RPI::Module::parseString<double>(getParameter("dh_t"));
		dh_param.dh_a = RPI::Module::parseString<double>(getParameter("dh_a"));
		dh_param.dh_al = RPI::Module::parseString<double>(getParameter("dh_al"));

		std::vector<double> minj, maxj;
		minj = RPI::Module::parseString<double>(getParameter("min_joint"));
		maxj = RPI::Module::parseString<double>(getParameter("max_joint"));

		KDL::JntArray q_min(minj.size()), q_max(maxj.size());

		for(int i = 0; i < minj.size(); ++i)
			q_min(i) = minj[i];
		for(int i = 0; i < maxj.size(); ++i)
			q_max(i) = maxj[i];


		kin_calc = new Schunk_KinGeo_Calculation(dh_param, q_min, q_max);
	}

	void SchunkLwaDriver::updateParameters() {
		// do nothing
	}

	std::set<std::string> SchunkLwaDriver::getMutableParameters() const {
		return std::set<std::string>();
	}

	RPI::DeviceState SchunkLwaDriver::getDeviceState() const {
		return current_state;
	}

	//bool may_quit;
	// TODO: Block destructor until last can message sent

	SchunkLwaDriver::~SchunkLwaDriver() {
		setAllJointPDOvalues<uint16_t>(controlword, 0);
		setAllJointHookStates(NODE_RESET);
		this->trigger();
		delete kin_calc;
		return;
	}

	void SchunkLwaDriver::setEStop(bool estop) {
		EMGCY_STOP = estop;
	}

	SchunkLwaDriver* SchunkLwaDriver::createDevice(std::string name, RPI::parameter_t parameters) {
		SchunkLwaDriver* ret = new SchunkLwaDriver(name, parameters);
		ret->configure();
		ret->start();
		return ret;
	}

	bool SchunkLwaDriver::configureHook() {
		return true;
	}

	bool SchunkLwaDriver::startHook() {
		return true;
	}

	int homing_counter = 0;

	void SchunkLwaDriver::updateHook() {
		for (int i=0; i<NUM_AXES; i++) {
			switch (commanded_hook_states[i]) {
				case NODE_RESET:
					// periodically retry reset of node
					if (current_hook_states[i] != NODE_CURRENT_RESET && current_hook_states[i] != NODE_CURRENT_RESET_WAIT) {
						std::cout << "Handling reset for node " << i+NODE_OFFSET << std::endl;
						hookHandleReset(i);
						commanded_hook_states[i] = NODE_INTERPOLATED_POSITION;
						this->trigger();
					}
					break;

				case NODE_INTERPOLATED_POSITION:
					if (current_hook_states[i] != NODE_CURRENT_INTERPOLATED_POSITION && current_hook_states[i] != NODE_CURRENT_INTERPOLATED_POSITION_SETUP) {
						setAllJointPDOvalues<bool>(was_ip_mode_active, false);
						std::cout << "Handling interpolated position mode for node " << i+NODE_OFFSET << std::endl;
						hookHandleChangeMode(i);
						hookHandleInterpolatedPosition(i);
						hookSetOperational(i);
						ip_counter++;
					}
					if (ip_counter >= NUM_AXES) {
						current_state = RPI::DeviceState::OPERATIONAL;
					}
					break;

				case NODE_HOMING:
					if (current_hook_states[i] != NODE_CURRENT_HOMING_DONE && current_hook_states[i] != NODE_CURRENT_HOMING_WAIT) {
						std::cout << "Handling homing mode for node " << i+NODE_OFFSET << std::endl;
						hookHandleChangeMode(i);
						hookHandleHoming(i);
						hookSetOperational(i);
					}
					break;

				case NODE_SWITCH_OFF:
					hookHandleSwitchOff(i);
					hookHandleReset(i);
					break;

			}
		}
		return;
	}

	void SchunkLwaDriver::hookHandleReset (int i) {
		current_hook_states[i] = NODE_CURRENT_RESET_WAIT;
		schunklwa->removeNodeHandler(i + NODE_OFFSET, this);

		current_state = RPI::DeviceState::OFFLINE;

		int id = i + NODE_OFFSET;

		schunklwa->setNodeMode(id, canopen::NM_RESET_APPLICATION);
		usleep(100000);

		hookHandleChangeMode(i);
		current_hook_states[i] = NODE_CURRENT_RESET;
	}

	void SchunkLwaDriver::hookHandleChangeMode (int i) {
		current_state = RPI::DeviceState::OFFLINE;
		schunklwa->setNodeMode(i + NODE_OFFSET, canopen::NM_PRE_OPERATIONAL);
		usleep(100000);

		statusword[i] = 0;
		controlword[i] = 0;
	}

	void SchunkLwaDriver::hookSetOperational (int i) {
		schunklwa->addNodeHandler(i+NODE_OFFSET, this);
		schunklwa->setNodeMode(i + NODE_OFFSET, canopen::NM_OPERATIONAL);
		usleep(100000);
	}

	void SchunkLwaDriver::hookHandleInterpolatedPosition (int i) {
		current_hook_states[i] = NODE_CURRENT_INTERPOLATED_POSITION_SETUP;
		current_hook_states[i] = schunklwa->writeSDO(i + NODE_OFFSET, 0x6060, 0, 0x07, 1) == 0 ? NODE_CURRENT_INTERPOLATED_POSITION : NODE_CURRENT_INTERPOLATED_POSITION_ERROR;

		// SET PDOS
		std::cout << "*** NODE ID: " << std::dec << i + NODE_OFFSET << std::endl;

		canopen::pdo_entry_list_t list;
		list.push_back(canopen::PDOitem_T<uint16_t>(0x6041, 0, &statusword[i]));
		list.push_back(canopen::PDOitem_T<int16_t>(0x6077, 0, &torque_actual[i]));
		list.push_back(canopen::PDOitem_T<int32_t>(0x6064, 0, &position_actual[i]));
		list.push_back(canopen::PDOitem_T<int32_t>(0x606B, 0, &velocity_demand[i]));
		list.push_back(canopen::PDOitem_T<int32_t>(0x606C, 0, &velocity_actual[i]));
		schunklwa->setPDOs(i + NODE_OFFSET, canopen::TX_PDO, list);

		list.clear();

		list.push_back(canopen::PDOitem_T<uint16_t>(0x6040, 0, &controlword[i]));
		list.push_back(canopen::PDOitem_T<int32_t>(0x60C1, 1, &interpolated_position[i]));
		schunklwa->setPDOs(i + NODE_OFFSET, canopen::RX_PDO, list);
	}

	void SchunkLwaDriver::hookHandleHoming (int i) {
		current_hook_states[i] = schunklwa->writeSDO(i + NODE_OFFSET, 0x6060, 0, 0x06, 1) == 0 ? NODE_CURRENT_HOMING_WAIT : NODE_CURRENT_HOMING_ERROR;

		// SET PDOS
		std::cout << "*** NODE ID: " << std::dec << i + NODE_OFFSET << std::endl;

		canopen::pdo_entry_list_t list;
		list.push_back(canopen::PDOitem_T<uint16_t>(0x6041, 0, &statusword[i]));
		schunklwa->setPDOs(i + NODE_OFFSET, canopen::TX_PDO, list);

		list.clear();

		list.push_back(canopen::PDOitem_T<uint16_t>(0x6040, 0, &controlword[i]));
		schunklwa->setPDOs(i + NODE_OFFSET, canopen::RX_PDO, list);
	}

	void SchunkLwaDriver::hookHandleSwitchOff (int i) {
		statusword[i] = 0;
		controlword[i] = 0;
	}

	void SchunkLwaDriver::nodeState(int nodeId, canopen::NodeState state) {
		std::cout << "Node " << nodeId << " state " << state << std::endl;
		return;
	}

	void SchunkLwaDriver::homeJoint(int joint) {
		commanded_hook_states[joint] = NODE_HOMING;
		this->trigger();
	}

	void SchunkLwaDriver::homeJoints() {
		setAllJointHookStates(NODE_HOMING);
		this->trigger();
	}

	void SchunkLwaDriver::startInterpolatedPositionMode() {
		setAllJointHookStates(NODE_INTERPOLATED_POSITION);
		this->trigger();
	}

	void SchunkLwaDriver::stopInterpolatedPositionMode() {
		setAllJointHookStates(NODE_RESET);
		this->trigger();
	}

	slave_state evalState (uint16_t status) {
		if (CANopenUtil::bitsVerify(status,5, 0,0, 1,0, 2,0, 3,0, 6,0))
			return SLAVE_NOT_READY_TO_SWITCH_ON;
		else if (CANopenUtil::bitsVerify(status,5, 0,0, 1,0, 2,0, 3,0, 6,1))
			return SLAVE_SWITCH_ON_DISABLED;
		else if (CANopenUtil::bitsVerify(status,6, 0,1, 1,0, 2,0, 3,0, 5,1, 6,0))
			return SLAVE_READY_TO_SWITCH_ON;
		else if (CANopenUtil::bitsVerify(status,6, 0,1, 1,1, 2,0, 3,0, 5,1, 6,0))
			return SLAVE_SWITCHED_ON;
		else if (CANopenUtil::bitsVerify(status,6, 0,1, 1,1, 2,1, 3,0, 5,1, 6,0))
			return SLAVE_OPERATION_ENABLE;
		else if (CANopenUtil::bitsVerify(status,6, 0,1, 1,1, 2,1, 3,0, 5,1, 6,0))
			return SLAVE_QUICK_STOP_ACTIVE;
		else if (CANopenUtil::bitVerify(status,3,1))
			return SLAVE_FAULT;
		else
			return SLAVE_UNKNOWN;
	}

	bool was_ip_mode_active[NUM_AXES] = {
			false,
			false,
			false,
			false,
			false,
			false
	};

	void SchunkLwaDriver::processData (int nodeId) {
		if (nodeId == 0 + NODE_OFFSET) {
			double positions_to_command[NUM_AXES];
			getValuesToCommand(positions_to_command, NULL);
			for (int i = 0; i < NUM_AXES; i++) {
				int currentNodeId = i + NODE_OFFSET;

				last_known_state[i] = evalState(statusword[i]);
				switch (last_known_state[i]) {
					case SLAVE_FAULT:
						controlword[i] = 128; /* clear fault flag */
						break;

					case SLAVE_SWITCH_ON_DISABLED:
						controlword[i] = 6; /* goto: Ready to Switch On (Transition 2) */
						break;

					case SLAVE_READY_TO_SWITCH_ON:
						controlword[i] = 7; /* goto: Switched On (Transition 3) */
						break;

					case SLAVE_SWITCHED_ON:
						controlword[i] = 15; /* goto: Operation Enable (Transition 4, defaults to Quick Stop Active, Transition 11) */
						break;

					case SLAVE_QUICK_STOP_ACTIVE:
						if (!EMGCY_STOP) {
							controlword[i] = 11; /* goto: Operation Enable (Transition 16) */
						}
						break;

					case SLAVE_OPERATION_ENABLE:
						if (EMGCY_STOP) {
							controlword[i] = 2;
							break;
						}

						// HOMING MODE STATEMACHINE
						if (current_hook_states[i] == NODE_CURRENT_HOMING_WAIT) {
							if (CANopenUtil::bitVerify(statusword[i], 13, 1)) {
								std::cout << "Error in homing mode!!" << std::endl;
							} else {
								if (CANopenUtil::bitVerify(statusword[i], 12, 0)) {
									if (controlword_old[i] != 31) {
										controlword[i] = 31;
										std::cout << "Activating homing mode!" << std::endl;
									}
								} else if (CANopenUtil::bitVerify(statusword[i], 12, 1)) {
									if (controlword_old[i] != 15) {
										controlword[i] = 15;
										std::cout << "Homing mode successfully completed!!!" << std::endl;
										current_hook_states[i] = NODE_CURRENT_HOMING_DONE;
										current_state = RPI::DeviceState::OPERATIONAL;
									}
								}
							}
						} else if (current_hook_states[i] == NODE_CURRENT_INTERPOLATED_POSITION) {

							// INTERPOLATED POSITION MODE STATEMACHINE
							if (CANopenUtil::bitVerify(statusword[i], 12, 0)) {  /* IP_MODE ACTIVE flag for Interpolated Position Mode  */
								std::cout << "Joint " << i << " :: was_ip_mode_active: " << (was_ip_mode_active[i] ? "true" : "false") << std::endl;
								if (!was_ip_mode_active[i]) {
									controlword[i] = 31;

									setJointPositionStatic(i, radValue(position_actual[i]));
									interpolated_position[i] = position_actual[i];

									std::cout << "Activating interpolation" << std::endl;
									//was_ip_mode_active[i] = true;
								} else {
									setAllJointHookStates(NODE_RESET);
									this->trigger();
								}
							} else {
								controlword[i] = 31;

								// Set desired position here
								interpolated_position[i] = milliDegValue(positions_to_command[i]);
							}
						}

						break;

					default:
						break;
				}

				position_actual_old[i] = position_actual[i];
				velocity_actual_old[i] = velocity_actual[i];

				controlword_old[i] = controlword[i];
				statusword_old[i] = statusword[i];
			}
		}
	}

	void SchunkLwaDriver::setAllJointHookStates (node_next_hook_state state) {
		for (int i=0; i<NUM_AXES; i++) {
			commanded_hook_states[i] = state;
		}
	}

	template <typename T>
	void SchunkLwaDriver::setAllJointPDOvalues (T array[], T value) {
		for (int i=0; i<NUM_AXES; i++) {
			array[i] = value;
		}
	}

	// Methods for CyclicPositionRobotArm
	int SchunkLwaDriver::getJointCount() const {
		return NUM_AXES;
	};

	double SchunkLwaDriver::getMeasuredJointPosition(int joint) {
		return radValue(position_actual_old[joint]);
		//return ((double)position_actual[joint]) / 1000 / 180 * KDL::PI;
	};

	int SchunkLwaDriver::getToolError(int axis) const {
		return 0;
	};

	 void SchunkLwaDriver::setToolMass(double mass, int axis) {
		return;
	};

	double SchunkLwaDriver::getMaximumVelocity(int joint) const {
		return 1.2566;
	};

	int SchunkLwaDriver::getJointError(int joint) {
		return last_known_state[joint] == SLAVE_OPERATION_ENABLE ? 0 : 1;
	};

	void SchunkLwaDriver::setToolCOM(KDL::Vector com, int axis) {

	};

	void SchunkLwaDriver::setToolMOI(KDL::Vector moi, int axis) {

	};

	double SchunkLwaDriver::getMeasuredJointVelocity(int joint) {
		return radValue(velocity_actual_old[joint]);
	};

	double SchunkLwaDriver::getMaximumAcceleration(int joint) const {
		return 5.2359;
	};

	robotarm::JointPositionError SchunkLwaDriver::checkJointPosition(int joint, double position) {
		 if (position == position)
			 return robotarm::JP_OK;
		 else
			 return robotarm::JP_INVALID;
	};

	double SchunkLwaDriver::getMeasuredJointAcceleration(int joint) {
		return 0;
	};

	bool SchunkLwaDriver::getToolFinished(int axis) const {
		return true;
	};

	// Interface ArmKinematics
	KDL::Frame SchunkLwaDriver::Kin(const RPI::Array<double>& joints)
	{
		return kin_calc->Kin(joints);
	}
	void SchunkLwaDriver::InvKin(const RPI::Array<double>& hintJoints, const KDL::Frame& position,
			RPI::Array<double>& values)
	{
		// We need exactly 6 axes
		if (hintJoints.getSize() != NUM_AXES || values.getSize() != NUM_AXES)
			return;

		kin_calc->InvKin(hintJoints, position, values);
	}


} /* namespace schunksdh */
