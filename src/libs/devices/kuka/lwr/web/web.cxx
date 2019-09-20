/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include "web.hpp"
#include "../device/LwrDevice.hpp"
#include <rcc/DeviceInstanceT.hpp>
#include <rcc/Registry.hpp>
#include "../kinematics/Lbr_Kin.hpp"

namespace kuka_lwr
{
	using namespace std;
	using namespace RPI;

	string LBRFRIControllerHandler::handleRequest(vector<string> path, string method, string data, string get) {
		stringstream ret;
		DeviceInstanceT<LwrDevice> di(path[1]);

		if(di.getDevice()==0) return "";

		ret	<< "HTTP/1.1 200 OK\r\n"
			<< "Content-Type: text/xml\r\n"
			<< "\r\n"
			<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/kuka/lwr/controller.xsl\" ?>\r\n"
			<< "<controller name=\"" << path[1] << "\" quality=\"";

		int quality = di.getDevice()->getQuality();
		if(quality == 1) ret<<"BAD"; else if(quality == 2) ret<<"OK"; else if(quality == 3) ret<<"PERFECT"; else if(quality == 0) ret<<"UNACCEPTABLE";
		ret << "\" mode=\"";
		int state = di.getDevice()->getState();
		if(state == 2) ret<<"COMMAND"; else if(state==1) ret<<"MONITOR"; else if(state==0) ret<<"OFF";
		ret << "\" controller=\"";
		int ctrl = di.getDevice()->getControlScheme();
		if(ctrl == 1) ret<<"POSITION"; else if(ctrl==2) ret<<"CARTIMP"; else if(ctrl==3) ret<<"JNTIMP"; else if(ctrl == 0) ret<<"OTHER";
		ret<<"\" power=\"" << (di.getDevice()->getPower()?"ON":"OFF") << "\"";
		ret<< ">\r\n";

		float j1, j2, j3, j4, j5, j6, j7;
		double alpha; KDL::Frame frame;
		j1 = di.getDevice()->getMeasuredJointPosition(0);
		j2 = di.getDevice()->getMeasuredJointPosition(1);
		j3 = di.getDevice()->getMeasuredJointPosition(2);
		j4 = di.getDevice()->getMeasuredJointPosition(3);
		j5 = di.getDevice()->getMeasuredJointPosition(4);
		j6 = di.getDevice()->getMeasuredJointPosition(5);
		j7 = di.getDevice()->getMeasuredJointPosition(6);

		ret << "	<joints j1=\"" << j1 << "\" j2=\"" << j2 << "\" j3=\"" << j3 << "\" j4=\"" << j4 << "\" j5=\"" << j5 << "\" j6=\"" << j6 << "\" j7=\"" << j7 << "\" />\r\n";
		di->alphaKin(j1, j2, j3, j4, j5, j6, j7, frame, alpha);
		//Lbr_Kin::lbrKin(j1, j2, j3, j4, j5, j6, j7, frame, alpha, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7);
		double r, p, y; frame.M.GetRPY(r, p, y);
		ret << "	<frame x=\"" << frame.p.x() << "\" y=\"" << frame.p.y() << "\" z=\"" << frame.p.z() << "\" a=\"" << y << "\" b=\"" << p << "\" c=\"" << r << "\" alpha=\"" << alpha << "\" />\r\n";


		j1 = di.getDevice()->getCommandedJointPosition(0);
		j2 = di.getDevice()->getCommandedJointPosition(1);
		j3 = di.getDevice()->getCommandedJointPosition(2);
		j4 = di.getDevice()->getCommandedJointPosition(3);
		j5 = di.getDevice()->getCommandedJointPosition(4);
		j6 = di.getDevice()->getCommandedJointPosition(5);
		j7 = di.getDevice()->getCommandedJointPosition(6);
		ret << "	<cmdjoints j1=\"" << j1 << "\" j2=\"" << j2 << "\" j3=\"" << j3 << "\" j4=\"" << j4 << "\" j5=\"" << j5 << "\" j6=\"" << j6 << "\" j7=\"" << j7 << "\" />\r\n";
		di->alphaKin(j1, j2, j3, j4, j5, j6, j7, frame, alpha);
		//Lbr_Kin::lbrKin(j1, j2, j3, j4, j5, j6, j7, frame, alpha, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7); frame.M.GetRPY(r, p, y);
		ret << "	<cmdframe x=\"" << frame.p.x() << "\" y=\"" << frame.p.y() << "\" z=\"" << frame.p.z() << "\" a=\"" << y << "\" b=\"" << p << "\" c=\"" << r << "\" alpha=\"" << alpha << "\" />\r\n";

		float frix, friy, friz, fria, frib, fric;
		di.getDevice()->getCartPosition(frix, friy, friz, fria, frib, fric);
		ret << "	<friframe x=\"" << frix << "\" y=\"" << friy << "\" z=\"" << friz << "\" a=\"" << fria << "\" b=\"" << frib << "\" c=\"" << fric << "\" />\r\n";

		float jntstiff[7];
		float jntdamp[7];
		float cartstiff[6];
		float cartdamp[6];

		for(int i = 0; i < 7; i++)
		{
			jntstiff[i] = di.getDevice()->getJntImpStiffness(i);
			jntdamp[i] = di.getDevice()->getJntImpDamping(i);
		}

		di.getDevice()->getCartImpStiffness(cartstiff[0],cartstiff[1],cartstiff[2],cartstiff[3],cartstiff[4],cartstiff[5]);
		di.getDevice()->getCartImpDamping(cartdamp[0],cartdamp[1],cartdamp[2],cartdamp[3],cartdamp[4],cartdamp[5]);

		ret << "	<jntstiff j1=\"" << jntstiff[0] << "\" j2=\"" << jntstiff[1] << "\" j3=\"" << jntstiff[2] << "\" j4=\"" << jntstiff[3] << "\" j5=\"" << jntstiff[4] << "\" j6=\"" << jntstiff[5] << "\" j7=\"" << jntstiff[6] << "\"/>\r\n";
		ret << "	<jntdamp j1=\"" << jntdamp[0] << "\" j2=\"" << jntdamp[1] << "\" j3=\"" << jntdamp[2] << "\" j4=\"" << jntdamp[3] << "\" j5=\"" << jntdamp[4] << "\" j6=\"" << jntdamp[5] << "\" j7=\"" << jntdamp[6] << "\"/>\r\n";

		ret << "	<cartstiff x=\"" << cartstiff[0] << "\" y=\"" << cartstiff[1] << "\" z=\"" << cartstiff[2] << "\" a=\"" << cartstiff[3] << "\" b=\"" << cartstiff[4] << "\" c=\"" << cartstiff[5] << "\"/>\r\n";
		ret << "	<cartdamp x=\"" << cartdamp[0] << "\" y=\"" << cartdamp[1] << "\" z=\"" << cartdamp[2] << "\" a=\"" << cartdamp[3] << "\" b=\"" << cartdamp[4] << "\" c=\"" << cartdamp[5] << "\"/>\r\n";


		ret << "</controller>\r\n";
		return ret.str();
	}


	string LBRFRIKinematicsHandler::handleRequest(vector<string> path, string method, string data, string get) {
		stringstream ret;
		DeviceInstanceT<LwrDevice> di(path[1]);
		LwrDevice* controller = di.getDevice();

		if(controller==0) return "";

		ret	<< "HTTP/1.1 200 OK\r\n"
			<< "Content-Type: text/xml\r\n"
			<< "\r\n"
			<< "<?xml version=\"1.0\" encoding=\"UTF-8\"?><?xml-stylesheet type=\"text/xsl\" href=\"/xsl/kuka/lwr/kinematics.xsl\" ?>\r\n"
			<< "<kinematics>\r\n";
		float f1, f2, f3, f4, f5, f6, f7;
		f1 = di.getDevice()->getMeasuredJointPosition(0);
		f2 = di.getDevice()->getMeasuredJointPosition(1);
		f3 = di.getDevice()->getMeasuredJointPosition(2);
		f4 = di.getDevice()->getMeasuredJointPosition(3);
		f5 = di.getDevice()->getMeasuredJointPosition(4);
		f6 = di.getDevice()->getMeasuredJointPosition(5);
		f7 = di.getDevice()->getMeasuredJointPosition(6);

		double j1=f1, j2=f2, j3=f3, j4=f4, j5=f5, j6=f6, j7=f7;
		if(getVar(get, "j1")!="") j1=atof(getVar(get, "j1").c_str());
		if(getVar(get, "j2")!="") j2=atof(getVar(get, "j2").c_str());
		if(getVar(get, "j3")!="") j3=atof(getVar(get, "j3").c_str());
		if(getVar(get, "j4")!="") j4=atof(getVar(get, "j4").c_str());
		if(getVar(get, "j5")!="") j5=atof(getVar(get, "j5").c_str());
		if(getVar(get, "j6")!="") j6=atof(getVar(get, "j6").c_str());
		if(getVar(get, "j7")!="") j7=atof(getVar(get, "j7").c_str());

		if(getVar(get, "x")!="" || getVar(get, "y")!="" || getVar(get, "z")!="" || getVar(get, "a")!="" || getVar(get, "b")!="" || getVar(get, "c")!="") {
			double alpha; KDL::Frame pos;
			di->alphaKin(j1, j2, j3, j4, j5, j6, j7, pos, alpha);
			//Lbr_Kin::lbrKin(j1, j2, j3, j4, j5, j6, j7, pos, alpha, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7);
			double x = pos.p.x(), y = pos.p.y(), z = pos.p.z();
			double a, b, c; pos.M.GetRPY(c, b, a);

			if(getVar(get, "x")!="") x=atof(getVar(get, "x").c_str());
			if(getVar(get, "y")!="") y=atof(getVar(get, "y").c_str());
			if(getVar(get, "z")!="") z=atof(getVar(get, "z").c_str());
			if(getVar(get, "a")!="") a=atof(getVar(get, "a").c_str());
			if(getVar(get, "b")!="") b=atof(getVar(get, "b").c_str());
			if(getVar(get, "c")!="") c=atof(getVar(get, "c").c_str());
			if(getVar(get, "alpha")!="") alpha=atof(getVar(get, "alpha").c_str());

			pos = KDL::Frame(KDL::Rotation::RPY(c, b, a), KDL::Vector(x, y, z));
			di->alphaInvKin(pos, alpha, j1, j2, j3, j4, j5, j6, j7);
			//Lbr_Kin::lbrInvKin(pos, alpha, j1, j2, j3, j4, j5, j6, j7, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7);
		}

		ret << "	<joints j1=\"" << j1 << "\" j2=\"" << j2 << "\" j3=\"" << j3 << "\" j4=\"" << j4 << "\" j5=\"" << j5 << "\" j6=\"" << j6 << "\" j7=\"" << j7 << "\" />\r\n";
		double alpha; KDL::Frame frame;
		di->alphaKin(j1, j2, j3, j4, j5, j6, j7, frame, alpha);
		//Lbr_Kin::lbrKin(j1, j2, j3, j4, j5, j6, j7, frame, alpha, lwr_l0, lwr_l1, lwr_l2, lwr_l3, lwr_l4, lwr_l5, lwr_l6, lwr_l7);
		double r, p, y; frame.M.GetRPY(r, p, y);
		ret << "	<frame x=\"" << frame.p.x() << "\" y=\"" << frame.p.y() << "\" z=\"" << frame.p.z() << "\" a=\"" << y << "\" b=\"" << p << "\" c=\"" << r << "\" alpha=\"" << alpha << "\" />\r\n";

		ret << "</kinematics>\r\n";
		return ret.str();
	}

} /* namespace FRI */
