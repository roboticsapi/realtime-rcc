/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
#include <kdl/frames.hpp>

namespace Core
{
	/**
	 * This class implements a clock module
	 */
	class Clock: public RPI::StatefulModule
	{

	public:

		Clock(std::string name, RPI::Net* net) :
				StatefulModule(name, net), inIncrement("inIncrement", this), outValue("outValue", this), propIncrement(
						"Increment", "Increment per second", 1), current(0)
		{
			setDescription("A clock module (counts the duration the module has been active)");
			this->ports()->addPort(&inIncrement, "Increment per second");
			this->ports()->addPort(&outValue, "Duration the module has been active");

			this->properties()->addProperty(&propIncrement);
		}

		bool configureHook()
		{
			current = 0;
			return true;
		}

		bool startHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				if (reset())
				{
					current = 0;
				}
				double increment = inIncrement.Get(propIncrement);
				current += increment * inNet->getNetFrequency();
				outValue.Set(current);
			}
		}

		void stopHook()
		{
		}

		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<double> inIncrement;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propIncrement;

	private:
		double current;
	};



	/**
	 * This class implements a net time module
	 */
	class TimeNet: public RPI::Module
	{

	public:

		TimeNet(std::string name, RPI::Net* net) :
				Module(name, net), outValue("outValue", this)
		{
			setDescription("A time module, returns the current (logical, net) time");
			this->ports()->addPort(&outValue, "The current time in ticks");
		}

		bool configureHook()
		{
			return true;
		}

		bool startHook()
		{
			return true;
		}

		void updateHook()
		{
			outValue.Set(inNet->getTime());
		}

		void stopHook()
		{
		}

		void cleanupHook()
		{
		}

	protected:
		RPI::OutPort<RTT::os::TimeService::nsecs> outValue;
	};


	/**
	 * This class implements a wall time module
	 */
	class TimeWall: public RPI::Module
	{

	public:

		TimeWall(std::string name, RPI::Net* net) :
				Module(name, net), outValue("outValue", this)
		{
			setDescription("A time module, returns the current (physical, wall) time");
			this->ports()->addPort(&outValue, "The current time in ticks");
		}

		bool configureHook()
		{
			return true;
		}

		bool startHook()
		{
			return true;
		}

		void updateHook()
		{
			outValue.Set(RTT::os::TimeService::Instance()->getNSecs());
		}

		void stopHook()
		{
		}

		void cleanupHook()
		{
		}

	protected:
		RPI::OutPort<RTT::os::TimeService::nsecs> outValue;
	};

	/**
	 * This class implements a time difference
	 */
	class TimeDiff: public RPI::Module
	{

	public:

		TimeDiff(std::string name, RPI::Net* net) :
				Module(name, net), inFirst("inFirst", this),
				inSecond("inSecond", this), outValue("outValue", this)
		{
			setDescription("A time difference module (returns seconds between two times)");
			this->ports()->addPort(&inFirst, "The first timestamp in nsecs");
			this->ports()->addPort(&inSecond, "The second timestamp in nsecs");
			this->ports()->addPort(&outValue, "The time difference (second - first) in seconds");
		}

		bool configureHook()
		{
			return true;
		}

		bool startHook()
		{
			return true;
		}

		void updateHook()
		{
			outValue.Set((inSecond.Get() - inFirst.Get()) / 1e9);
		}

		void stopHook()
		{
		}

		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<RTT::os::TimeService::nsecs> inFirst;
		RPI::InPort<RTT::os::TimeService::nsecs> inSecond;
		RPI::OutPort<double> outValue;
	};


	/**
	 * This class implements a time difference
	 */
	class TimeAdd: public RPI::Module
	{

	public:

		TimeAdd(std::string name, RPI::Net* net) :
				Module(name, net), inFirst("inTime", this),
				inSecond("inDelta", this), outValue("outValue", this)
		{
			setDescription("A time add module (returns a time after applying a delta)");
			this->ports()->addPort(&inFirst, "The first timestamp [nsecs]");
			this->ports()->addPort(&inSecond, "The time difference [s]");
			this->ports()->addPort(&outValue, "The added time (time + delta) [nsecs]");
		}

		bool configureHook()
		{
			return true;
		}

		bool startHook()
		{
			return true;
		}

		void updateHook()
		{
			outValue.Set((inFirst.Get() + inSecond.Get()*1e9));
		}

		void stopHook()
		{
		}

		void cleanupHook()
		{
		}

	protected:

		RPI::InPort<RTT::os::TimeService::nsecs> inFirst;
		RPI::InPort<double> inSecond;
		RPI::OutPort<RTT::os::TimeService::nsecs> outValue;
	};




	/**
	 * This class implements a interval scaler module
	 */
	class Interval: public RPI::ActiveModule
	{

	public:
		Interval(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), inMin("inMin", this), inMax("inMax", this), outActive(
						"outActive", this), outValue("outValue", this), propMin("Min", "Start tick", 0), propMax("Max",
						"End tick", 1000)
		{
			setDescription("An interval scaler module (scales values from interval [Min,Max] to [0,1])");
			this->ports()->addPort(&outActive, "Follow-up activating (true if Min <= inValue <= Max)");
			this->ports()->addPort(&inValue, "Current value");
			this->ports()->addPort(&inMin, "Minimum value (mapped to 0)");
			this->ports()->addPort(&inMax, "Maximum value (mapped to 1)");
			this->ports()->addPort(&outValue, "Result");

			this->properties()->addProperty(&propMin);
			this->properties()->addProperty(&propMax);

		}

		bool configureHook()
		{
			if (!inValue.connected())
				return false;
			return true;
		}

		bool startHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				double min = inMin.Get(propMin), max = inMax.Get(propMax);
				double tick = inValue.Get();
				if (tick < min)
				{
					outValue.Set(0);
					outActive.Set(false);
				} else if (tick > max)
				{
					outValue.Set(1);
					outActive.Set(false);
				} else
				{
					if (min == max)
						max = min + 1;
					outValue.Set(1.0 * (tick - min) / (max - min));
					outActive.Set(true);
				}
			} else
			{
				outActive.Set(false);
			}
		}

		void stopHook()
		{
		}

		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<double> inValue;
		RPI::InPort<double> inMin;
		RPI::InPort<double> inMax;
		RPI::OutPort<bool> outActive;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propMin;
		RPI::Property<double> propMax;
	};

	/**
	 * This class implements a linear interpolation module
	 */
	class Lerp: public RPI::ActiveModule
	{
	public:

		/**
		 * Constructor. Sets up the interface of this TaskContext.
		 */
		Lerp(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inFrom("inFrom", this), inTo("inTo", this), inAmount("inAmount", this), outValue(
						"outValue", this), propFrom("From", "Start position", 0), propTo("To", "Destination position",
						0)
		{
			setDescription("Linear interpolation module (linearly interpolates between from and to)");
			this->ports()->addPort(&inFrom, "Start value");
			this->ports()->addPort(&inTo, "Destination value");
			this->ports()->addPort(&inAmount, "Amount (0 = from, 1 = to, 0.5 = in between)");
			this->ports()->addPort(&outValue, "Interpolated value");

			this->properties()->addProperty(&propFrom);
			this->properties()->addProperty(&propTo);

		}

		bool configureHook()
		{
			if (!inAmount.connected())
				return false;
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				double from = inFrom.Get(propFrom), to = inTo.Get(propTo);
				double value = from + inAmount.Get() * (to - from);
				outValue.Set(value);
			}
		}

		bool startHook()
		{
			return true;
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<double> inFrom;
		RPI::InPort<double> inTo;
		RPI::InPort<double> inAmount;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propFrom;
		RPI::Property<double> propTo;
	};

	/**
	 * This class implements a trapezoidal modifier module
	 */
	class Rampify: public RPI::ActiveModule
	{
	public:
		Rampify(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), outValue("outValue", this), propConstant("Constant",
						"Amount of time to apply constant motion (0 <= Constant <= 1)", 0.5), cons(0), acc(0), v(0), cs(
						0), ds(0)
		{
			setDescription(
					"A trapezoidal modifier module (applies a trapezoidal profile to an input value ranging from 0 to 1)");
			this->ports()->addPort(&inValue, "Input value (0 <= inValue <= 1)");
			this->ports()->addPort(&outValue, "Value with trapezoidal profile (0 <= outValue <= 1)");
			this->properties()->addProperty(&propConstant);

		}

		bool configureHook()
		{
			if (!inValue.connected())
				return false;
			cons = propConstant.get();
			if (cons > 1 || cons < 0)
				return false;

			acc = (1.0 - cons) / 2.0;
			v = 1.0 / (cons + acc);
			cs = acc / 2 * v;
			ds = cs + cons * v;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				double tRel = inValue.Get(), tOut = 0;
				if (tRel < 0)
				{
					tOut = 0;
				} else if (tRel < acc)
				{
					tOut = v / acc * tRel * tRel / 2;
				} else if (tRel < acc + cons)
				{
					tOut = cs + (tRel - acc) * v;
				} else if (tRel <= 1)
				{
					tOut = 1 - (v / acc * (1 - tRel) * (1 - tRel) / 2);
				} else
				{
					tOut = 1;
				}
				outValue.Set(tOut);
			}
		}

		bool startHook()
		{
			return true;
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<double> inValue;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propConstant;

	private:
		double cons, acc, v, cs, ds;

	};

	/**
	 * This class implements a bezier module for a double value
	 */
	class DoubleBezier: public RPI::Module
	{
	public:
		DoubleBezier(std::string name, RPI::Net* net) :
				Module(name, net), inValue("inValue", this), inFrom("inFrom", this), inFromVel(
						"inFromVel", this), inToVel("inToVel", this), inTo("inTo", this), outValue("outValue", this), propFrom(
						"From", "Start position", 0), propFromVel("FromVel", "Velocity at start position", 0), propToVel(
						"ToVel", "Velocity at destination position", 0), propTo("To", "Destination position", 0)
		{
			setDescription(
					"A bezier interpolation module for a double value (Interpolates between from and to with given velocities)");
			this->ports()->addPort(&inValue, "Progress value (0 <= inValue <= 1)");
			this->ports()->addPort(&inFrom, "Start position");
			this->ports()->addPort(&inTo, "Destination position");
			this->ports()->addPort(&inFromVel, "Velocity at start position");
			this->ports()->addPort(&inToVel, "Velocity at destination position");
			this->ports()->addPort(&outValue, "Interpolated value");

			this->properties()->addProperty(&propFrom);
			this->properties()->addProperty(&propTo);
			this->properties()->addProperty(&propFromVel);
			this->properties()->addProperty(&propToVel);

		}

		bool configureHook()
		{
			if (!inValue.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			double from = inFrom.Get(propFrom), to = inTo.Get(propTo), fromVel = inFromVel.Get(propFromVel),
					toVel = inToVel.Get(propToVel);
			double control1 = from + fromVel / 3,
					control2 = to - toVel / 3;
			double a = from;
			double b = -3 * from + 3 * control1;
			double c = 3 * from - 6 * control1 + 3 * control2;
			double d = -from + 3 * control1 - 3 * control2 + to;

			double t = inValue.Get();
			double value = a + t * (b + t * (c + d * t));
			outValue.Set(value);
		}

		bool startHook()
		{
			return true;
		}

		void stopHook()
		{
		}

		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<double> inValue;
		RPI::InPort<double> inFrom;
		RPI::InPort<double> inFromVel;
		RPI::InPort<double> inToVel;
		RPI::InPort<double> inTo;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propFrom;
		RPI::Property<double> propFromVel;
		RPI::Property<double> propToVel;
		RPI::Property<double> propTo;

	};

//
//	/**
//	 * This class implements a bezier module for a double value
//	 */
//	class RotationBezier: public RPI::Module
//	{
//	public:
//		RotationBezier(std::string name, RPI::Net* net) :
//				Module(name, net), inValue("inValue", this), inFrom("inFrom", this), inFromVel(
//						"inFromVel", this), inToVel("inToVel", this), inTo("inTo", this), outValue("outValue", this)
//		{
//			setDescription(
//					"A bezier interpolation module for a rotation (Interpolates between from and to with given rotational velocities)");
//			this->ports()->addPort(&inValue, "Progress value (0 <= inValue <= 1)");
//			this->ports()->addPort(&inFrom, "Start rotation");
//			this->ports()->addPort(&inTo, "Destination rotation");
//			this->ports()->addPort(&inFromVel, "Velocity at start position");
//			this->ports()->addPort(&inToVel, "Velocity at destination position");
//			this->ports()->addPort(&outValue, "Interpolated value");
//
//		}
//
//		bool configureHook()
//		{
//			if (!inValue.connected())
//				return false;
//			if (!inFrom.connected())
//				return false;
//			if (!inTo.connected())
//				return false;
//			if (!inFromVel.connected())
//				return false;
//			if (!inToVel.connected())
//				return false;
//
//			return true;
//		}
//
//		void updateHook()
//		{
//			KDL::Rotation from = inFrom.Get(), to = inTo.Get();
//			KDL::Vector fromVel = inFromVel.Get(), toVel = inToVel.Get();
//			KDL::Rotation fromPlus = KDL::addDelta(from, fromVel, 1.0/3);
//			KDL::Rotation toMinus = KDL::addDelta(to, toVel, -1.0/3);
//
//			double fr[4], tr[4], fp[4], tm[4], val[4];
//			from.GetQuaternion(fr[0], fr[1], fr[2], fr[3]);
//			fromPlus.GetQuaternion(fp[0], fp[1], fp[2], fp[3]);
//			toMinus.GetQuaternion(tm[0], tm[1], tm[2], tm[3]);
//			to.GetQuaternion(tr[0], tr[1], tr[2], tr[3]);
//			double t = inValue.Get(), len = 0;
//			for(int i=0; i<4; i++) {
//
//				double a = fr[i];
//				double b = -3 * fr[i] + 3 * fp[i];
//				double c = 3 * fr[i] - 6 * fp[i] + 3 * tm[i];
//				double d = -fr[i] + 3 * fp[i] - 3 * tm[i] + tr[i];
//
//				val[i] = a + t * (b + t * (c + d * t));
//				len += val[i] * val[i];
//			}
//			len = sqrt(len);
//			outValue.Set(KDL::Rotation::Quaternion(val[0]/len, val[1]/len, val[2]/len, val[3]/len));
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//
//		void stopHook()
//		{
//		}
//
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<double> inValue;
//		RPI::InPort<KDL::Rotation> inFrom;
//		RPI::InPort<KDL::Vector> inFromVel;
//		RPI::InPort<KDL::Vector> inToVel;
//		RPI::InPort<KDL::Rotation> inTo;
//		RPI::OutPort<KDL::Rotation> outValue;
//
//	};

	/**
	 * This class implements a cubic bezier module
	 */
	class CubicBezier: public RPI::ActiveModule
	{
	public:
		CubicBezier(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), inFrom("inFrom", this), inControl1(
						"inControl1", this), inControl2("inControl2", this), inTo("inTo", this), outValue("outValue",
						this), propFrom("From", "Start position", 0), propControl1("Control1", "First control position",
						0), propControl2("Control2", "Second control position", 0), propTo("To", "Destination position",
						0)
		{
			setDescription(
					"A cubic bezier interpolation module (Interpolates between from and to using two control points)");
			this->ports()->addPort(&inValue, "Progress value (0 <= inValue <= 1)");
			this->ports()->addPort(&inFrom, "Start position");
			this->ports()->addPort(&inTo, "Destination position");
			this->ports()->addPort(&inControl1, "First control position");
			this->ports()->addPort(&inControl2, "Second control position");
			this->ports()->addPort(&outValue, "Interpolated value");

			this->properties()->addProperty(&propFrom);
			this->properties()->addProperty(&propTo);
			this->properties()->addProperty(&propControl1);
			this->properties()->addProperty(&propControl2);

		}

		bool configureHook()
		{
			if (!inValue.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				double from = inFrom.Get(propFrom), to = inTo.Get(propTo), control1 = inControl1.Get(propControl1),
						control2 = inControl2.Get(propControl2);
				double a = from;
				double b = -3 * from + 3 * control1;
				double c = 3 * from - 6 * control1 + 3 * control2;
				double d = -from + 3 * control1 - 3 * control2 + to;

				double t = inValue.Get();
				double value = a + t * (b + t * (c + d * t));
				outValue.Set(value);
			}
		}

		bool startHook()
		{
			return true;
		}

		void stopHook()
		{
		}

		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<double> inValue;
		RPI::InPort<double> inFrom;
		RPI::InPort<double> inControl1;
		RPI::InPort<double> inControl2;
		RPI::InPort<double> inTo;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propFrom;
		RPI::Property<double> propControl1;
		RPI::Property<double> propControl2;
		RPI::Property<double> propTo;

	};
//
//	/**
//	 * This class implements a vector combiner module
//	 */
//	class VectorFromXYZ: public RPI::ActiveModule
//	{
//
//	public:
//		VectorFromXYZ(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inX("inX", this), inY("inY", this), inZ("inZ", this), outValue(
//						"outValue", this), propX("X", "X position", 0), propY("Y", "Y position", 0), propZ("Z",
//						"Z position", 0)
//		{
//			setDescription("A vector from XYZ module (combines values for X, Y, Z into a vector datatype)");
//			this->ports()->addPort(&inX, "X position");
//			this->ports()->addPort(&inY, "Y position");
//			this->ports()->addPort(&inZ, "Z position");
//			this->ports()->addPort(&outValue, "Combined vector");
//
//			this->properties()->addProperty(&propX);
//			this->properties()->addProperty(&propY);
//			this->properties()->addProperty(&propZ);
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				double x = inX.Get(propX);
//				double y = inY.Get(propY);
//				double z = inZ.Get(propZ);
//				outValue.Set(KDL::Vector(x, y, z));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<double> inX;
//		RPI::InPort<double> inY;
//		RPI::InPort<double> inZ;
//		RPI::OutPort<KDL::Vector> outValue;
//
//		RPI::Property<double> propX;
//		RPI::Property<double> propY;
//		RPI::Property<double> propZ;
//
//	};
//
//	/**
//	 * This class implements a vector splitting module
//	 */
//	class VectorToXYZ: public RPI::ActiveModule
//	{
//
//	public:
//		VectorToXYZ(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), outX("outX", this), outY("outY", this), outZ(
//						"outZ", this)
//		{
//			setDescription("A vector to XYZ module (extracts values for X, Y, Z from a vector datatype)");
//			this->ports()->addPort(&outX, "X position");
//			this->ports()->addPort(&outY, "Y position");
//			this->ports()->addPort(&outZ, "Z position");
//			this->ports()->addPort(&inValue, "Input vector");
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Vector frame = inValue.Get();
//
//				outX.Set(frame.x());
//				outY.Set(frame.y());
//				outZ.Set(frame.z());
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Vector> inValue;
//
//		RPI::OutPort<double> outX;
//		RPI::OutPort<double> outY;
//		RPI::OutPort<double> outZ;
//	};
//
//	/**
//	 * This class implements a twist rotation module
//	 */
//	class VectorRotate: public RPI::ActiveModule
//	{
//
//	public:
//		VectorRotate(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), inRot("inRot", this), outValue("outValue",
//						this)
//		{
//			setDescription("A vector rotation module");
//			this->ports()->addPort(&outValue, "Rotated vector");
//			this->ports()->addPort(&inRot, "Input rotation");
//			this->ports()->addPort(&inValue, "Input vector");
//		}
//
//		bool configureHook()
//		{
//			if (!inValue.connected())
//				return false;
//			if (!inRot.connected())
//				return false;
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Vector twist = inValue.Get();
//				KDL::Rotation rot = inRot.Get();
//				outValue.Set(rot * twist);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Vector> inValue;
//		RPI::InPort<KDL::Rotation> inRot;
//
//		RPI::OutPort<KDL::Vector> outValue;
//	};
//
//	/**
//	 * This class implements a vector addition module
//	 */
//	class VectorAdd: public RPI::ActiveModule
//	{
//
//	public:
//
//		VectorAdd(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
//						"outValue", this)
//		{
//			setDescription("A vector addition module");
//			this->ports()->addPort(&inFirst, "First vector");
//			this->ports()->addPort(&inSecond, "Second vector");
//			this->ports()->addPort(&outValue, "Combined vector (First + Second)");
//		}
//
//		bool configureHook()
//		{
//			if (!inFirst.connected() || !inSecond.connected())
//				return false;
//
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Vector first = inFirst.Get();
//				KDL::Vector second = inSecond.Get();
//				KDL::Vector ret = KDL::Vector(first + second);
//				outValue.Set(ret);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Vector> inFirst;
//		RPI::InPort<KDL::Vector> inSecond;
//		RPI::OutPort<KDL::Vector> outValue;
//	};
//
//	/**
//	 * This class implements a vector dot product module
//	 */
//	class VectorDot: public RPI::ActiveModule
//	{
//
//	public:
//
//		VectorDot(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
//						"outValue", this)
//		{
//			setDescription("A vector dot product module");
//			this->ports()->addPort(&inFirst, "First vector");
//			this->ports()->addPort(&inSecond, "Second vector");
//			this->ports()->addPort(&outValue, "Dot product (First * Second)");
//		}
//
//		bool configureHook()
//		{
//			if (!inFirst.connected() || !inSecond.connected())
//				return false;
//
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Vector first = inFirst.Get();
//				KDL::Vector second = inSecond.Get();
//				outValue.Set(KDL::dot(first, second));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Vector> inFirst;
//		RPI::InPort<KDL::Vector> inSecond;
//		RPI::OutPort<double> outValue;
//	};
//
//	/**
//	 * This class implements a vector scale module
//	 */
//	class VectorScale: public RPI::ActiveModule
//	{
//
//	public:
//
//		VectorScale(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), inFactor("inFactor", this), outValue(
//						"outValue", this), propFactor("Factor", "Scale factor", 1)
//		{
//			setDescription("A vector scale module");
//			this->ports()->addPort(&inValue, "Input vector");
//			this->ports()->addPort(&inFactor, "Scale factor");
//			this->ports()->addPort(&outValue, "Scaled vector (Vector * Factor)");
//			this->properties()->addProperty(&propFactor);
//		}
//
//		bool configureHook()
//		{
//			if (!inValue.connected())
//				return false;
//
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Vector vector = inValue.Get();
//				double factor = inFactor.Get(propFactor);
//				outValue.Set(vector * factor);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Vector> inValue;
//		RPI::InPort<double> inFactor;
//		RPI::OutPort<KDL::Vector> outValue;
//		RPI::Property<double> propFactor;
//	};
//
//	/**
//	 * This class implements a vector cross product module
//	 */
//	class VectorCross: public RPI::ActiveModule
//	{
//
//	public:
//
//		VectorCross(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
//						"outValue", this)
//		{
//			setDescription("A vector cross product module");
//			this->ports()->addPort(&inFirst, "First vector");
//			this->ports()->addPort(&inSecond, "Second vector");
//			this->ports()->addPort(&outValue, "Combined vector (First x Second)");
//		}
//
//		bool configureHook()
//		{
//			if (!inFirst.connected() || !inSecond.connected())
//				return false;
//
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Vector first = inFirst.Get();
//				KDL::Vector second = inSecond.Get();
//				outValue.Set(first * second);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Vector> inFirst;
//		RPI::InPort<KDL::Vector> inSecond;
//		RPI::OutPort<KDL::Vector> outValue;
//	};
//
//	/**
//	 * This class implements a frame combiner module
//	 */
//	class FrameFromXYZ: public RPI::ActiveModule
//	{
//
//	public:
//		FrameFromXYZ(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inX("inX", this), inY("inY", this), inZ("inZ", this), inRot("inRot",
//						this), outValue("outValue", this), propX("X", "X position", 0), propY("Y", "Y position", 0), propZ(
//						"Z", "Z position", 0), propA("A", "A rotation", 0), propB("B", "B rotation", 0), propC("C",
//						"C rotation", 0)
//		{
//			setDescription(
//					"A frame from XYZ module (combines values for X, Y, Z and rotation into a frame datatype) [deprecated]");
//			this->ports()->addPort(&inX, "X position");
//			this->ports()->addPort(&inY, "Y position");
//			this->ports()->addPort(&inZ, "Z position");
//			this->ports()->addPort(&inRot, "Rotation");
//			this->ports()->addPort(&outValue, "Combined frame");
//
//			this->properties()->addProperty(&propX);
//			this->properties()->addProperty(&propY);
//			this->properties()->addProperty(&propZ);
//			this->properties()->addProperty(&propA);
//			this->properties()->addProperty(&propB);
//			this->properties()->addProperty(&propC);
//
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				double x = inX.Get(propX);
//				double y = inY.Get(propY);
//				double z = inZ.Get(propZ);
//				KDL::Rotation rot = inRot.Get(KDL::Rotation::RPY(propC.get(), propB.get(), propA.get()));
//				outValue.Set(KDL::Frame(rot, KDL::Vector(x, y, z)));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<double> inX;
//		RPI::InPort<double> inY;
//		RPI::InPort<double> inZ;
//		RPI::InPort<KDL::Rotation> inRot;
//		RPI::OutPort<KDL::Frame> outValue;
//
//		RPI::Property<double> propX;
//		RPI::Property<double> propY;
//		RPI::Property<double> propZ;
//		RPI::Property<double> propA;
//		RPI::Property<double> propB;
//		RPI::Property<double> propC;
//
//	};
//
//	/**
//	 * This class implements a frame combiner module
//	 */
//	class FrameFromPosRot: public RPI::ActiveModule
//	{
//
//	public:
//		FrameFromPosRot(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inPos("inPos", this), inRot("inRot", this), outValue("outValue", this), propX(
//						"X", "X position", 0), propY("Y", "Y position", 0), propZ("Z", "Z position", 0), propA("A",
//						"A rotation", 0), propB("B", "B rotation", 0), propC("C", "C rotation", 0)
//		{
//			setDescription("A frame combiner module (combines position and rotation into a frame datatype)");
//			this->ports()->addPort(&inPos, "Position");
//			this->ports()->addPort(&inRot, "Rotation");
//			this->ports()->addPort(&outValue, "Combined frame");
//
//			this->properties()->addProperty(&propX);
//			this->properties()->addProperty(&propY);
//			this->properties()->addProperty(&propZ);
//			this->properties()->addProperty(&propA);
//			this->properties()->addProperty(&propB);
//			this->properties()->addProperty(&propC);
//
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Vector pos = inPos.Get(KDL::Vector(propX.get(), propY.get(), propZ.get()));
//				KDL::Rotation rot = inRot.Get(KDL::Rotation::RPY(propC.get(), propB.get(), propA.get()));
//				outValue.Set(KDL::Frame(rot, pos));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Vector> inPos;
//		RPI::InPort<KDL::Rotation> inRot;
//		RPI::OutPort<KDL::Frame> outValue;
//
//		RPI::Property<double> propX;
//		RPI::Property<double> propY;
//		RPI::Property<double> propZ;
//		RPI::Property<double> propA;
//		RPI::Property<double> propB;
//		RPI::Property<double> propC;
//
//	};
//
//	/**
//	 * This class implements a frame splitting module
//	 */
//	class FrameToPosRot: public RPI::ActiveModule
//	{
//
//	public:
//		FrameToPosRot(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), outPosition("outPosition", this), outRotation(
//						"outRotation", this)
//		{
//			setDescription("A frame splitter module (extracts position and rotation form a frame datatype)");
//			this->ports()->addPort(&outPosition, "Position");
//			this->ports()->addPort(&outRotation, "Rotation");
//			this->ports()->addPort(&inValue, "Combined frame");
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Frame frame = inValue.Get();
//				outPosition.Set(frame.p);
//				outRotation.Set(frame.M);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Frame> inValue;
//
//		RPI::OutPort<KDL::Vector> outPosition;
//		RPI::OutPort<KDL::Rotation> outRotation;
//	};
//
//	/**
//	 * This class implements a frame splitting module
//	 */
//	class FrameToXYZ: public RPI::ActiveModule
//	{
//
//	public:
//		FrameToXYZ(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), outX("outX", this), outY("outY", this), outZ(
//						"outZ", this), outRot("outRot", this)
//		{
//			setDescription(
//					"A frame to XYZ module (extracts values for X, Y, Z and rotation from a frame datatype) [deprecated]");
//			this->ports()->addPort(&outX, "X position");
//			this->ports()->addPort(&outY, "Y position");
//			this->ports()->addPort(&outZ, "Z position");
//			this->ports()->addPort(&outRot, "Rotation");
//			this->ports()->addPort(&inValue, "Combined frame");
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Frame frame = inValue.Get();
//
//				outX.Set(frame.p.x());
//				outY.Set(frame.p.y());
//				outZ.Set(frame.p.z());
//
//				outRot.Set(frame.M);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Frame> inValue;
//
//		RPI::OutPort<double> outX;
//		RPI::OutPort<double> outY;
//		RPI::OutPort<double> outZ;
//		RPI::OutPort<KDL::Rotation> outRot;
//	};
//
//	/**
//	 * This class implements a frame linear interpolation module
//	 */
//	class FrameLerp: public RPI::ActiveModule
//	{
//
//	public:
//		FrameLerp(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inFrom("inFrom", this), inTo("inTo", this), inAmount("inAmount", this), outValue(
//						"outValue", this)
//		{
//			setDescription("Linear frame interpolation module");
//			this->ports()->addPort(&inFrom, "Start frame");
//			this->ports()->addPort(&inTo, "Destination frame");
//			this->ports()->addPort(&inAmount, "Amount (0 = inFrom, 1 = inTo)");
//			this->ports()->addPort(&outValue, "Interpolated frame");
//
//		}
//
//		bool configureHook()
//		{
//			if (!inFrom.connected())
//				return false;
//			if (!inTo.connected())
//				return false;
//			if (!inAmount.connected())
//				return false;
//
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Frame first = inFrom.Get(), second = inTo.Get();
//				double b = inAmount.Get();
//				double a = 1 - b;
//				KDL::Rotation aim = first.M.Inverse() * second.M;
//				KDL::Vector axis = aim.GetRot();
//				KDL::Rotation rot = first.M * aim.Rot(axis, axis.Norm() * b);
//
//				outValue.Set(
//						KDL::Frame(
//								rot,
//								KDL::Vector(first.p.x() * a + second.p.x() * b, first.p.y() * a + second.p.y() * b,
//										first.p.z() * a + second.p.z() * b)));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Frame> inFrom;
//		RPI::InPort<KDL::Frame> inTo;
//		RPI::InPort<double> inAmount;
//		RPI::OutPort<KDL::Frame> outValue;
//
//	};
//
//	/**
//	 * This class implements applies a twist to a frame.
//	 */
//	class FrameAddTwist: public RPI::ActiveModule
//	{
//
//	public:
//		FrameAddTwist(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inFrame("inFrame", this), inTwist("inTwist", this), outValue(
//						"outValue", this)
//		{
//			setDescription("Applies a twist to a frame");
//			this->ports()->addPort(&inFrame, "Frame");
//			this->ports()->addPort(&inTwist, "Twist to add");
//			this->ports()->addPort(&outValue, "Changed frame");
//
//		}
//
//		bool configureHook()
//		{
//			if (!inFrame.connected())
//				return false;
//			if (!inTwist.connected())
//				return false;
//
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Frame frame = inFrame.Get();
//				KDL::Twist twist = inTwist.Get();
//
//				outValue.Set(KDL::addDelta(frame, twist, inNet->getNetFrequency()));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Frame> inFrame;
//		RPI::InPort<KDL::Twist> inTwist;
//		RPI::OutPort<KDL::Frame> outValue;
//
//	};
//
//	/**
//	 * This class implements a frame inversion module
//	 */
//	class FrameInvert: public RPI::ActiveModule
//	{
//
//	public:
//		FrameInvert(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), outValue("outValue", this)
//		{
//			setDescription("Frame inversion module");
//			this->ports()->addPort(&inValue, "input Frame");
//			this->ports()->addPort(&outValue, "inverted Frame");
//		}
//
//		bool configureHook()
//		{
//			if (!inValue.connected())
//				return false;
//
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				outValue.Set(inValue.Get().Inverse());
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Frame> inValue;
//		RPI::OutPort<KDL::Frame> outValue;
//	};
//
//	/**
//	 * This class implements a rotation combiner module
//	 */
//	class RotationFromABC: public RPI::ActiveModule
//	{
//
//	public:
//
//		RotationFromABC(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inA("inA", this), inB("inB", this), inC("inC", this), outValue(
//						"outValue", this), propA("A", "A rotation", 0), propB("B", "B rotation", 0), propC("C",
//						"C rotation", 0)
//		{
//			setDescription("A rotation combiner (creates a rotation data flow from A, B, C angles)");
//			this->ports()->addPort(&inA, "A rotation (around Z)");
//			this->ports()->addPort(&inB, "B rotation (around Y)");
//			this->ports()->addPort(&inC, "C rotation (around X)");
//			this->ports()->addPort(&outValue, "Combined rotation");
//
//			this->properties()->addProperty(&propA);
//			this->properties()->addProperty(&propB);
//			this->properties()->addProperty(&propC);
//
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				double a = inA.Get(propA);
//				double b = inB.Get(propB);
//				double c = inC.Get(propC);
//				outValue.Set(KDL::Rotation::RPY(c, b, a));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<double> inA;
//		RPI::InPort<double> inB;
//		RPI::InPort<double> inC;
//		RPI::OutPort<KDL::Rotation> outValue;
//
//		RPI::Property<double> propA;
//		RPI::Property<double> propB;
//		RPI::Property<double> propC;
//
//	};
//
//	/**
//	 * This class implements a rotation combiner module
//	 */
//	class RotationFromAxisAngle: public RPI::ActiveModule
//	{
//
//	public:
//		RotationFromAxisAngle(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inAxis("inAxis", this), inAngle(
//						"inAngle", this), outValue("outValue", this), propX("X", "Axis x value", 1), propY("Y",
//						"Axis y value", 0), propZ("Z", "Axis z value", 0), propAngle("Angle", "Rotation angle", 0)
//		{
//			setDescription(
//					"A rotation combiner module (creates a rotation data flow from an axis and an angle to rotate around)");
//			this->ports()->addPort(&inAxis, "Rotation axis (normalized)");
//			this->ports()->addPort(&inAngle, "Angle to rotate");
//			this->ports()->addPort(&outValue, "Combined rotation");
//
//			this->properties()->addProperty(&propX);
//			this->properties()->addProperty(&propY);
//			this->properties()->addProperty(&propZ);
//			this->properties()->addProperty(&propAngle);
//
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Vector axis = inAxis.Get(KDL::Vector(propX.get(), propY.get(), propZ.get()));
//				double angle = inAngle.Get(propAngle);
//				outValue.Set(KDL::Rotation::Rot2(axis, angle));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Vector> inAxis;
//		RPI::InPort<double> inAngle;
//		RPI::OutPort<KDL::Rotation> outValue;
//
//		RPI::Property<double> propX;
//		RPI::Property<double> propY;
//		RPI::Property<double> propZ;
//		RPI::Property<double> propAngle;
//
//	};
//
//
//	/**
//	 * This class implements a rotation splitter module
//	 */
//	class RotationToAxisAngle: public RPI::ActiveModule
//	{
//
//	public:
//		RotationToAxisAngle(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), outAxis("outAxis", this),
//				outAngle("outAngle", this)
//		{
//			setDescription(
//					"A rotation splitter module (calculates normalized axis and angle from a rotation data flow)");
//			this->ports()->addPort(&inValue, "Rotation");
//			this->ports()->addPort(&outAxis, "Rotation axis (normalized)");
//			this->ports()->addPort(&outAngle, "Angle to rotate");
//
//		}
//
//		bool configureHook()
//		{
//			if(!inValue.connected())
//				return false;
//
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Rotation rot = inValue.Get();
//				KDL::Vector axis;
//				outAngle.Set(rot.GetRotAngle(axis));
//				outAxis.Set(axis);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Rotation> inValue;
//		RPI::OutPort<KDL::Vector> outAxis;
//		RPI::OutPort<double> outAngle;
//	};
//
//
//	/**
//	 * This class implements a rotation splitter module
//	 */
//	class RotationInvert: public RPI::ActiveModule
//	{
//
//	public:
//		RotationInvert(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), outValue("outValue", this)
//		{
//			setDescription(
//					"A rotation inversion module (calculates the inverse rotation)");
//			this->ports()->addPort(&inValue, "Rotation");
//			this->ports()->addPort(&outValue, "Inverted rotation");
//
//		}
//
//		bool configureHook()
//		{
//			if(!inValue.connected())
//				return false;
//
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Rotation rot = inValue.Get();
//				outValue.Set(rot.Inverse());
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Rotation> inValue;
//		RPI::OutPort<KDL::Rotation> outValue;
//	};
//
//
//	/**
//	 * This class implements a rotation combiner module
//	 */
//	class RotationToABC: public RPI::ActiveModule
//	{
//
//	public:
//
//		RotationToABC(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), outA("outA", this), outB("outB", this), outC("outC", this), inValue(
//						"inValue", this)
//		{
//			setDescription("A rotation splitter (extracts A, B, C values from Rotation type)");
//			this->ports()->addPort(&outA, "A rotation (around Z)");
//			this->ports()->addPort(&outB, "B rotation (around Y)");
//			this->ports()->addPort(&outC, "C rotation (around X)");
//			this->ports()->addPort(&inValue, "Combined rotation");
//
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Rotation rotation = inValue.Get();
//				double a, b, c;
//				rotation.GetRPY(c, b, a);
//				outA.Set(a);
//				outB.Set(b);
//				outC.Set(c);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::OutPort<double> outA;
//		RPI::OutPort<double> outB;
//		RPI::OutPort<double> outC;
//		RPI::InPort<KDL::Rotation> inValue;
//	};
//
//	/**
//	 * This class implements a rotation combiner module
//	 */
//	class RotationTransform: public RPI::ActiveModule
//	{
//
//	public:
//
//		RotationTransform(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
//						"outValue", this)
//		{
//			setDescription("A rotation transformer (combines two rotations)");
//			this->ports()->addPort(&inFirst, "First rotation");
//			this->ports()->addPort(&inSecond, "Second rotation");
//			this->ports()->addPort(&outValue, "Combined rotation (First * Second)");
//
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				outValue.Set(inFirst.Get() * inSecond.Get());
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Rotation> inFirst;
//		RPI::InPort<KDL::Rotation> inSecond;
//		RPI::OutPort<KDL::Rotation> outValue;
//	};
//
//	/**
//	 * This class implements a frame transformer module
//	 */
//	class FrameTransform: public RPI::ActiveModule
//	{
//
//	public:
//
//		FrameTransform(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
//						"outValue", this)
//		{
//			setDescription("A frame transformer module (combines two frame data flows)");
//			this->ports()->addPort(&inFirst, "First frame");
//			this->ports()->addPort(&inSecond, "Second frame");
//			this->ports()->addPort(&outValue, "Combined frame (First * Second)");
//
//		}
//
//		bool configureHook()
//		{
//			if (!inFirst.connected() || !inSecond.connected())
//				return false;
//
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Frame first = inFirst.Get();
//				KDL::Frame second = inSecond.Get();
//				KDL::Frame ret = KDL::Frame(first.M * second.M, first.M * second.p + first.p);
//				outValue.Set(ret);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Frame> inFirst;
//		RPI::InPort<KDL::Frame> inSecond;
//		RPI::OutPort<KDL::Frame> outValue;
//
//	};
//
//	/**
//	 * This class implements a wrench combiner module
//	 */
//	class WrenchFromXYZ: public RPI::ActiveModule
//	{
//
//	public:
//		WrenchFromXYZ(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inX("inX", this), inY("inY", this), inZ("inZ", this), inA("inA", this), inB(
//						"inB", this), inC("inC", this), outValue("outValue", this), propX("X", "X force", 0), propY("Y",
//						"Y force", 0), propZ("Z", "Z force", 0), propA("A", "A torque", 0), propB("B", "B torque", 0), propC(
//						"C", "C torque", 0)
//		{
//			setDescription(
//					"A wrench from XYZ module (combines values for X, Y, Z forces and A, B, C torques into a wrench datatype)");
//			this->ports()->addPort(&inX, "X force");
//			this->ports()->addPort(&inY, "Y force");
//			this->ports()->addPort(&inZ, "Z force");
//			this->ports()->addPort(&inA, "A torque");
//			this->ports()->addPort(&inB, "B torque");
//			this->ports()->addPort(&inC, "C torque");
//
//			this->ports()->addPort(&outValue, "Combined wrench");
//
//			this->properties()->addProperty(&propX);
//			this->properties()->addProperty(&propY);
//			this->properties()->addProperty(&propZ);
//			this->properties()->addProperty(&propA);
//			this->properties()->addProperty(&propB);
//			this->properties()->addProperty(&propC);
//
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				double x = inX.Get(propX);
//				double y = inY.Get(propY);
//				double z = inZ.Get(propZ);
//				double a = inA.Get(propA);
//				double b = inB.Get(propB);
//				double c = inC.Get(propC);
//				KDL::Vector force = KDL::Vector(x, y, z);
//				KDL::Vector torque = KDL::Vector(c, b, a);
//				outValue.Set(KDL::Wrench(force, torque));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<double> inX;
//		RPI::InPort<double> inY;
//		RPI::InPort<double> inZ;
//		RPI::InPort<double> inA;
//		RPI::InPort<double> inB;
//		RPI::InPort<double> inC;
//
//		RPI::OutPort<KDL::Wrench> outValue;
//
//		RPI::Property<double> propX;
//		RPI::Property<double> propY;
//		RPI::Property<double> propZ;
//		RPI::Property<double> propA;
//		RPI::Property<double> propB;
//		RPI::Property<double> propC;
//
//	};
//
//	/**
//	 * This class implements a wrench combiner module
//	 */
//	class TwistFromXYZ: public RPI::ActiveModule
//	{
//
//	public:
//		TwistFromXYZ(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inX("inX", this), inY("inY", this), inZ("inZ", this), inA("inA", this), inB(
//						"inB", this), inC("inC", this), outValue("outValue", this), propX("X", "X velocity", 0), propY(
//						"Y", "Y velocity", 0), propZ("Z", "Z velocity", 0), propA("A", "A velocity", 0), propB("B",
//						"B velocity", 0), propC("C", "C velocity", 0)
//		{
//			setDescription(
//					"A twist from XYZ module (combines values for X, Y, Z velocities and A, B, C velocities into a twist datatype) [deprecated]");
//			this->ports()->addPort(&inX, "X velocity");
//			this->ports()->addPort(&inY, "Y velocity");
//			this->ports()->addPort(&inZ, "Z velocity");
//			this->ports()->addPort(&inA, "A velocity");
//			this->ports()->addPort(&inB, "B velocity");
//			this->ports()->addPort(&inC, "C velocity");
//
//			this->ports()->addPort(&outValue, "Combined twist");
//
//			this->properties()->addProperty(&propX);
//			this->properties()->addProperty(&propY);
//			this->properties()->addProperty(&propZ);
//			this->properties()->addProperty(&propA);
//			this->properties()->addProperty(&propB);
//			this->properties()->addProperty(&propC);
//
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				double x = inX.Get(propX);
//				double y = inY.Get(propY);
//				double z = inZ.Get(propZ);
//				double a = inA.Get(propA);
//				double b = inB.Get(propB);
//				double c = inC.Get(propC);
//				KDL::Vector vel = KDL::Vector(x, y, z);
//				KDL::Vector rot = KDL::Vector(c, b, a);
//				outValue.Set(KDL::Twist(vel, rot));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<double> inX;
//		RPI::InPort<double> inY;
//		RPI::InPort<double> inZ;
//		RPI::InPort<double> inA;
//		RPI::InPort<double> inB;
//		RPI::InPort<double> inC;
//
//		RPI::OutPort<KDL::Twist> outValue;
//
//		RPI::Property<double> propX;
//		RPI::Property<double> propY;
//		RPI::Property<double> propZ;
//		RPI::Property<double> propA;
//		RPI::Property<double> propB;
//		RPI::Property<double> propC;
//
//	};
//
//	/**
//	 * This class implements a wrench combiner module
//	 */
//	class TwistFromVelocities: public RPI::ActiveModule
//	{
//
//	public:
//		TwistFromVelocities(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inTransVel("inTransVel", this), inRotVel("inRotVel", this), outValue(
//						"outValue", this), propX("X", "X velocity", 0), propY("Y", "Y velocity", 0), propZ("Z",
//						"Z velocity", 0), propRX("RX", "X rotation velocity", 0), propRY("RY", "Y rotation velocity",
//						0), propRZ("RZ", "Z rotation velocity", 0)
//		{
//			setDescription(
//					"A twist from velocity vector module (combines values for translational and rotational velocity into a twist datatype)");
//			this->ports()->addPort(&inTransVel, "Translational velocity");
//			this->ports()->addPort(&inRotVel, "Rotational velocity");
//
//			this->ports()->addPort(&outValue, "Combined twist");
//
//			this->properties()->addProperty(&propX);
//			this->properties()->addProperty(&propY);
//			this->properties()->addProperty(&propZ);
//			this->properties()->addProperty(&propRX);
//			this->properties()->addProperty(&propRY);
//			this->properties()->addProperty(&propRZ);
//
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Vector vel = inTransVel.Get(KDL::Vector(propX.get(), propY.get(), propZ.get()));
//				KDL::Vector rot = inRotVel.Get(KDL::Vector(propRX.get(), propRY.get(), propRZ.get()));
//				outValue.Set(KDL::Twist(vel, rot));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Vector> inTransVel;
//		RPI::InPort<KDL::Vector> inRotVel;
//
//		RPI::OutPort<KDL::Twist> outValue;
//
//		RPI::Property<double> propX;
//		RPI::Property<double> propY;
//		RPI::Property<double> propZ;
//		RPI::Property<double> propRX;
//		RPI::Property<double> propRY;
//		RPI::Property<double> propRZ;
//
//	};
//
//	/**
//	 * This class implements a wrench splitting module
//	 */
//	class TwistToVelocities: public RPI::ActiveModule
//	{
//
//	public:
//		TwistToVelocities(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), outTransVel("outTransVel", this), outRotVel(
//						"outRotVel", this)
//		{
//			setDescription(
//					"A twist to velocities module (extracts translational and rotational velocities from a twist data type)");
//			this->ports()->addPort(&outTransVel, "Translational velocity");
//			this->ports()->addPort(&outRotVel, "Rotational velocity");
//			this->ports()->addPort(&inValue, "Input twist");
//		}
//
//		bool configureHook()
//		{
//			if (!inValue.connected())
//				return false;
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Twist twist = inValue.Get();
//				outTransVel.Set(twist.vel);
//				outRotVel.Set(twist.rot);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Twist> inValue;
//
//		RPI::OutPort<KDL::Vector> outTransVel;
//		RPI::OutPort<KDL::Vector> outRotVel;
//	};
//
//	/**
//	 * This class implements a twist rotation module
//	 */
//	class TwistRotate: public RPI::ActiveModule
//	{
//
//	public:
//		TwistRotate(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), inRot("inRot", this), outValue("outValue",
//						this)
//		{
//			setDescription("A twist rotation module");
//			this->ports()->addPort(&outValue, "Rotated twist");
//			this->ports()->addPort(&inRot, "Input rotation");
//			this->ports()->addPort(&inValue, "Input twist");
//		}
//
//		bool configureHook()
//		{
//			if (!inValue.connected())
//				return false;
//			if (!inRot.connected())
//				return false;
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Twist twist = inValue.Get();
//				KDL::Rotation rot = inRot.Get();
//				outValue.Set(rot * twist);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Twist> inValue;
//		RPI::InPort<KDL::Rotation> inRot;
//
//		RPI::OutPort<KDL::Twist> outValue;
//	};
//
//	class TwistAdd: public RPI::ActiveModule
//	{
//
//		/**
//		 * This class implements a twist add module
//		 */
//	public:
//		TwistAdd(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
//						"outValue", this)
//		{
//			setDescription("A twist add module");
//			this->ports()->addPort(&outValue, "sum");
//			this->ports()->addPort(&inFirst, "summand one");
//			this->ports()->addPort(&inSecond, "summand two");
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Twist twist1 = inFirst.Get();
//				KDL::Twist twist2 = inSecond.Get();
//				outValue.Set(twist1 + twist2);
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Twist> inFirst;
//		RPI::InPort<KDL::Twist> inSecond;
//
//		RPI::OutPort<KDL::Twist> outValue;
//	};
//
//	/**
//	 * This class implements a wrench splitting module
//	 */
//	class TwistChangeCenter: public RPI::ActiveModule
//	{
//
//	public:
//		TwistChangeCenter(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), inVector("inVector", this), outValue(
//						"outValue", this)
//		{
//			setDescription("A twist change rotatation center module");
//			this->ports()->addPort(&outValue, "Changed twist");
//			this->ports()->addPort(&inVector, "Vector to move the rotation center");
//			this->ports()->addPort(&inValue, "Input twist");
//		}
//
//		bool configureHook()
//		{
//			if (!inValue.connected())
//				return false;
//			if (!inVector.connected())
//				return false;
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Twist twist = inValue.Get();
//				KDL::Vector ref = inVector.Get();
//				outValue.Set(twist.RefPoint(ref));
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Twist> inValue;
//		RPI::InPort<KDL::Vector> inVector;
//
//		RPI::OutPort<KDL::Twist> outValue;
//	};
//
//	/**
//	 * This class implements a wrench splitting module
//	 */
//	class TwistToXYZ: public RPI::ActiveModule
//	{
//
//	public:
//		TwistToXYZ(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), outX("outX", this), outY("outY", this), outZ(
//						"outZ", this), outA("outA", this), outB("outB", this), outC("outC", this)
//		{
//			setDescription(
//					"A twist to XYZ module (extracts values for X, Y, Z velocities and A, B, C velocities) [deprecated]");
//			this->ports()->addPort(&outX, "X velocity");
//			this->ports()->addPort(&outY, "Y velocity");
//			this->ports()->addPort(&outZ, "Z velocity");
//			this->ports()->addPort(&outA, "A velocity");
//			this->ports()->addPort(&outB, "B velocity");
//			this->ports()->addPort(&outC, "C velocity");
//			this->ports()->addPort(&inValue, "Combined twist");
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Twist twist = inValue.Get();
//
//				outX.Set(twist.vel.x());
//				outY.Set(twist.vel.y());
//				outZ.Set(twist.vel.z());
//				outA.Set(twist.rot.z());
//				outB.Set(twist.rot.y());
//				outC.Set(twist.rot.x());
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Twist> inValue;
//
//		RPI::OutPort<double> outX;
//		RPI::OutPort<double> outY;
//		RPI::OutPort<double> outZ;
//		RPI::OutPort<double> outA;
//		RPI::OutPort<double> outB;
//		RPI::OutPort<double> outC;
//	};
//
//	/**
//	 * This class implements a wrench splitting module
//	 */
//	class WrenchToXYZ: public RPI::ActiveModule
//	{
//
//	public:
//		WrenchToXYZ(std::string name, RPI::Net* net) :
//				ActiveModule(name, net), inValue("inValue", this), outX("outX", this), outY("outY", this), outZ(
//						"outZ", this), outA("outA", this), outB("outB", this), outC("outC", this)
//		{
//			setDescription("A wrench to XYZ module (extracts values for X, Y, Z forces and A, B, C torques)");
//			this->ports()->addPort(&outX, "X force");
//			this->ports()->addPort(&outY, "Y force");
//			this->ports()->addPort(&outZ, "Z force");
//			this->ports()->addPort(&outA, "A torque");
//			this->ports()->addPort(&outB, "B torque");
//			this->ports()->addPort(&outC, "C torque");
//			this->ports()->addPort(&inValue, "Combined wrench");
//		}
//
//		bool configureHook()
//		{
//			return true;
//		}
//
//		void updateHook()
//		{
//			if (active())
//			{
//				KDL::Wrench wrench = inValue.Get();
//
//				outX.Set(wrench.force.x());
//				outY.Set(wrench.force.y());
//				outZ.Set(wrench.force.z());
//				outA.Set(wrench.torque.z());
//				outB.Set(wrench.torque.y());
//				outC.Set(wrench.torque.x());
//			}
//		}
//
//		bool startHook()
//		{
//			return true;
//		}
//		void stopHook()
//		{
//		}
//		void cleanupHook()
//		{
//		}
//
//	protected:
//		RPI::InPort<KDL::Wrench> inValue;
//
//		RPI::OutPort<double> outX;
//		RPI::OutPort<double> outY;
//		RPI::OutPort<double> outZ;
//		RPI::OutPort<double> outA;
//		RPI::OutPort<double> outB;
//		RPI::OutPort<double> outC;
//	};

}

