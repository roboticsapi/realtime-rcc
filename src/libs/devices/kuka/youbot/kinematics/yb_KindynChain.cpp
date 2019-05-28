/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include "yb_KindynChain.hpp"

namespace kuka_youbot
{

	yb_kindyn_chain::yb_kindyn_chain()
	{

	}

	yb_kindyn_chain::~yb_kindyn_chain()
	{

	}

	KDL::Chain yb_kindyn_chain::getYBChain()
	{
		// create chain
		KDL::Chain chain = KDL::Chain();


		// ---- arm_link_1 ----

		// name
		std::string linkOneName = "arm_link_1";

		// joint
		std::string linkOneJointName = "arm_joint_1";
		KDL::Vector linkOneJointOrigin = KDL::Vector(0.024,0.0,0.096);
		KDL::Vector linkOneJointAxis = KDL::Vector(0.0,0.0,1.0);
		KDL::Joint::JointType linkOneJointType = KDL::Joint::RotAxis;
		KDL::Joint linkOneJoint = KDL::Joint(linkOneJointName,linkOneJointOrigin,linkOneJointAxis,linkOneJointType);

		// frame to tip
		double linkOneFrameRotationRoll = 0.0;
		double linkOneFrameRotationPitch = -0.0;
		double linkOneFrameRotationYaw = 0.0;
		KDL::Rotation linkOneFrameM = KDL::Rotation::RPY(linkOneFrameRotationRoll,
				linkOneFrameRotationPitch, linkOneFrameRotationYaw);
		KDL::Vector linkOneFrameP = KDL::Vector(0.024,0.0,0.096);
		KDL::Frame linkOneFrame = KDL::Frame(linkOneFrameM,linkOneFrameP);

		// rigidBodyInertia
		double linkOneInertiaMass = 1.39;
		KDL::Vector linkOneInertiaCOG = KDL::Vector(0.0,0.0,0.0);
		KDL::RotationalInertia linkOneRotInertia = KDL::RotationalInertia(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);
		KDL::RigidBodyInertia linkOneInertia = KDL::RigidBodyInertia(linkOneInertiaMass,linkOneInertiaCOG,linkOneRotInertia);

		// segment
		KDL::Segment linkOneSegment = KDL::Segment(linkOneName, linkOneJoint, linkOneFrame, linkOneInertia);
		chain.addSegment(linkOneSegment);


		// ---- arm_link_2 ----

		// name
		std::string linkTwoName = "arm_link_2";

		// joint
		std::string linkTwoJointName = "arm_joint_2";
		KDL::Vector linkTwoJointOrigin = KDL::Vector(0.033,0.0,0.019);
		KDL::Vector linkTwoJointAxis = KDL::Vector(0.0, -1.0, 0.0);
		KDL::Joint::JointType linkTwoJointType = KDL::Joint::RotAxis;
		KDL::Joint linkTwoJoint = KDL::Joint(linkTwoJointName,linkTwoJointOrigin,linkTwoJointAxis,linkTwoJointType);

		// frame to tip
		double linkTwoFrameRotationRoll = 0.0;
		double linkTwoFrameRotationPitch = -0.0;
		double linkTwoFrameRotationYaw = 0.0;
		KDL::Rotation linkTwoFrameM = KDL::Rotation::RPY(linkTwoFrameRotationRoll,
				linkTwoFrameRotationPitch, linkTwoFrameRotationYaw);
		KDL::Vector linkTwoFrameP = KDL::Vector(0.033,0.0,0.019);
		KDL::Frame linkTwoFrame = KDL::Frame(linkTwoFrameM,linkTwoFrameP);

		// rigidBodyInertia
		double linkTwoInertiaMass = 1.318 ;
		KDL::Vector linkTwoInertiaCOG = KDL::Vector(0.015, -0.01903, 0.11397);
		KDL::RotationalInertia linkTwoRotInertia = KDL::RotationalInertia(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);
		KDL::RigidBodyInertia linkTwoInertia = KDL::RigidBodyInertia(linkTwoInertiaMass,linkTwoInertiaCOG,linkTwoRotInertia);

		// segment
		KDL::Segment linkTwoSegment = KDL::Segment(linkTwoName, linkTwoJoint, linkTwoFrame, linkTwoInertia);
		chain.addSegment(linkTwoSegment);


		// ---- arm_link_3 ----

		// name
		std::string linkThreeName = "arm_link_3";

		// joint
		std::string linkThreeJointName = "arm_joint_3";
		KDL::Vector linkThreeJointOrigin = KDL::Vector(0.0,0.0,0.155);
		KDL::Vector linkThreeJointAxis = KDL::Vector(0.0, 1.0, 0.0);
		KDL::Joint::JointType linkThreeJointType = KDL::Joint::RotAxis;
		KDL::Joint linkThreeJoint = KDL::Joint(linkThreeJointName,linkThreeJointOrigin,linkThreeJointAxis,linkThreeJointType);

		// frame to tip
		double linkThreeFrameRotationRoll = 0.0;
		double linkThreeFrameRotationPitch = -0.0;
		double linkThreeFrameRotationYaw = 0.0;
		KDL::Rotation linkThreeFrameM = KDL::Rotation::RPY(linkThreeFrameRotationRoll,
				linkThreeFrameRotationPitch, linkThreeFrameRotationYaw);
		KDL::Vector linkThreeFrameP = KDL::Vector(0.0,0.0,0.155);
		KDL::Frame linkThreeFrame = KDL::Frame(linkThreeFrameM,linkThreeFrameP);

		// rigidBodyInertia
		double linkThreeInertiaMass = 0.821;
		KDL::Vector linkThreeInertiaCOG = KDL::Vector(0.00013, 0.02022, 0.10441);
		KDL::RotationalInertia linkThreeRotInertia = KDL::RotationalInertia(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);
		KDL::RigidBodyInertia linkThreeInertia = KDL::RigidBodyInertia(linkThreeInertiaMass,linkThreeInertiaCOG,linkThreeRotInertia);

		// segment
		KDL::Segment linkThreeSegment = KDL::Segment(linkThreeName, linkThreeJoint, linkThreeFrame, linkThreeInertia);
		chain.addSegment(linkThreeSegment);


		// ---- arm_link_4 ----

		// name
		std::string linkFourName = "arm_link_4";

		// joint
		std::string linkFourJointName = "arm_joint_4";
		KDL::Vector linkFourJointOrigin = KDL::Vector(0.0,0.0,0.135);
		KDL::Vector linkFourJointAxis = KDL::Vector(0.0, -1.0, 0.0);
		KDL::Joint::JointType linkFourJointType = KDL::Joint::RotAxis;
		KDL::Joint linkFourJoint = KDL::Joint(linkFourJointName,linkFourJointOrigin,linkFourJointAxis,linkFourJointType);

		// frame to tip
		double linkFourFrameRotationRoll = 0.0;
		double linkFourFrameRotationPitch = -0.0;
		double linkFourFrameRotationYaw = 0.0;
		KDL::Rotation linkFourFrameM = KDL::Rotation::RPY(linkFourFrameRotationRoll,
				linkFourFrameRotationPitch, linkFourFrameRotationYaw);
		KDL::Vector linkFourFrameP = KDL::Vector(0.0,0.0,0.135);
		KDL::Frame linkFourFrame = KDL::Frame(linkFourFrameM,linkFourFrameP);

		// rigidBodyInertia
		double linkFourInertiaMass = 0.769;
		KDL::Vector linkFourInertiaCOG = KDL::Vector(0.00015, -0.02464, 0.05353);
		KDL::RotationalInertia linkFourRotInertia = KDL::RotationalInertia(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);
		KDL::RigidBodyInertia linkFourInertia = KDL::RigidBodyInertia(linkFourInertiaMass,linkFourInertiaCOG,linkFourRotInertia);

		// segment
		KDL::Segment linkFourSegment = KDL::Segment(linkFourName, linkFourJoint, linkFourFrame, linkFourInertia);
		chain.addSegment(linkFourSegment);


		// ---- arm_link_5 ----

		// name
		std::string linkFiveName = "arm_link_5";

		// joint
		std::string linkFiveJointName = "arm_joint_5";
		KDL::Vector linkFiveJointOrigin = KDL::Vector(0.0,0.0,0.13);
		KDL::Vector linkFiveJointAxis = KDL::Vector(0.0, 0.0, 1.0);
		KDL::Joint::JointType linkFiveJointType = KDL::Joint::RotAxis;
		KDL::Joint linkFiveJoint = KDL::Joint(linkFiveJointName,linkFiveJointOrigin,linkFiveJointAxis,linkFiveJointType);

		// frame to tip
		double linkFiveFrameRotationRoll = 0.0;
		double linkFiveFrameRotationPitch = -0.0;
		double linkFiveFrameRotationYaw = 0.0;
		KDL::Rotation linkFiveFrameM = KDL::Rotation::RPY(linkFiveFrameRotationRoll,
				linkFiveFrameRotationPitch, linkFiveFrameRotationYaw);
		KDL::Vector linkFiveFrameP = KDL::Vector(0.0,0.0,0.13);
		KDL::Frame linkFiveFrame = KDL::Frame(linkFiveFrameM,linkFiveFrameP);

		// rigidBodyInertia
		double linkFiveInertiaMass = 0.687;
		KDL::Vector linkFiveInertiaCOG = KDL::Vector(0.0012, 0.0, -0.01648);
		KDL::RotationalInertia linkFiveRotInertia = KDL::RotationalInertia(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);
		KDL::RigidBodyInertia linkFiveInertia = KDL::RigidBodyInertia(linkFiveInertiaMass,linkFiveInertiaCOG,linkFiveRotInertia);

		// segment
		KDL::Segment linkFiveSegment = KDL::Segment(linkFiveName, linkFiveJoint, linkFiveFrame, linkFiveInertia);
		chain.addSegment(linkFiveSegment);


		// ---- gripper_palm_link ----

		// name
		std::string linkGripperName = "gripper_palm_link";

		// joint
		std::string linkGripperJointName = "gripper_palm_link";
		KDL::Joint::JointType linkGripperJointType = KDL::Joint::None;
		KDL::Joint linkGripperJoint = KDL::Joint(linkGripperJointName,linkGripperJointType);

		// frame to tip
		double linkGripperFrameRotationRoll = 0.0;
		double linkGripperFrameRotationPitch = -0.0;
		double linkGripperFrameRotationYaw = 0.0;
		KDL::Rotation linkGripperFrameM = KDL::Rotation::RPY(linkGripperFrameRotationRoll,
				linkGripperFrameRotationPitch, linkGripperFrameRotationYaw);
		KDL::Vector linkGripperFrameP = KDL::Vector(0.0,0.0,0.041);
		KDL::Frame linkGripperFrame = KDL::Frame(linkGripperFrameM,linkGripperFrameP);

		// rigidBodyInertia
		double linkGripperInertiaMass = 0.119;
		KDL::Vector linkGripperInertiaCOG = KDL::Vector(0.0, 0.0, 0.026);
		KDL::RotationalInertia linkGripperRotInertia = KDL::RotationalInertia(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);
		KDL::RigidBodyInertia linkGripperInertia = KDL::RigidBodyInertia(linkGripperInertiaMass,linkGripperInertiaCOG,linkGripperRotInertia);

		// segment
		KDL::Segment linkGripperSegment = KDL::Segment(linkGripperName, linkGripperJoint, linkGripperFrame, linkGripperInertia);
		chain.addSegment(linkGripperSegment);


		// ---- arm_tool ----

		// name
		std::string toolName = "arm_tool";

		// joint
		std::string toolJointName = "arm_tool";
		KDL::Joint::JointType toolJointType = KDL::Joint::None;
		KDL::Joint toolJoint = KDL::Joint(toolJointName, toolJointType);

		// frame to tip
		double toolFrameRotationRoll = 0.0;
		double toolFrameRotationPitch = -0.0;
		double toolFrameRotationYaw = 0.0;
		KDL::Rotation toolFrameM = KDL::Rotation::RPY(toolFrameRotationRoll,
				toolFrameRotationPitch, toolFrameRotationYaw);
		KDL::Vector toolFrameP = KDL::Vector(0.0, 0.0, 0.0);
		KDL::Frame toolFrame = KDL::Frame(linkGripperFrameM, toolFrameP);

		// rigidBodyInertia
		double toolInertiaMass = 0;
		KDL::Vector toolInertiaCOG = KDL::Vector(0.0, 0.0, 0.0);
		KDL::RotationalInertia toolRotInertia = KDL::RotationalInertia(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);
		KDL::RigidBodyInertia toolInertia = KDL::RigidBodyInertia(toolInertiaMass, toolInertiaCOG,
				toolRotInertia);

		// segment
		KDL::Segment toolSegment = KDL::Segment(toolName, toolJoint, toolFrame,
				toolInertia);
		chain.addSegment(toolSegment);

		return chain;
	}
}




