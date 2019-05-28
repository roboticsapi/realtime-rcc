/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */
 
#include <kdl/frames.hpp>

namespace World
{

	/**
	 * This class implements a bezier module for a double value
	 */
	class RotationBezier: public RPI::Module
	{
	public:
		RotationBezier(std::string name, RPI::Net* net) :
				Module(name, net), inValue("inValue", this), inFrom("inFrom", this), inFromVel(
						"inFromVel", this), inToVel("inToVel", this), inTo("inTo", this), outValue("outValue", this)
		{
			setDescription(
					"A bezier interpolation module for a rotation (Interpolates between from and to with given rotational velocities)");
			this->ports()->addPort(&inValue, "Progress value (0 <= inValue <= 1)");
			this->ports()->addPort(&inFrom, "Start rotation");
			this->ports()->addPort(&inTo, "Destination rotation");
			this->ports()->addPort(&inFromVel, "Velocity at start position");
			this->ports()->addPort(&inToVel, "Velocity at destination position");
			this->ports()->addPort(&outValue, "Interpolated value");

		}

		bool configureHook()
		{
			if (!inValue.connected())
				return false;
			if (!inFrom.connected())
				return false;
			if (!inTo.connected())
				return false;
			if (!inFromVel.connected())
				return false;
			if (!inToVel.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			KDL::Rotation from = inFrom.Get(), to = inTo.Get();
			KDL::Vector fromVel = inFromVel.Get(), toVel = inToVel.Get();
			KDL::Rotation fromPlus = KDL::addDelta(from, fromVel, 1.0/3);
			KDL::Rotation toMinus = KDL::addDelta(to, toVel, -1.0/3);

			double fr[4], tr[4], fp[4], tm[4], val[4];
			from.GetQuaternion(fr[0], fr[1], fr[2], fr[3]);
			fromPlus.GetQuaternion(fp[0], fp[1], fp[2], fp[3]);
			toMinus.GetQuaternion(tm[0], tm[1], tm[2], tm[3]);
			to.GetQuaternion(tr[0], tr[1], tr[2], tr[3]);
			double t = inValue.Get(), len = 0;
			for(int i=0; i<4; i++) {

				double a = fr[i];
				double b = -3 * fr[i] + 3 * fp[i];
				double c = 3 * fr[i] - 6 * fp[i] + 3 * tm[i];
				double d = -fr[i] + 3 * fp[i] - 3 * tm[i] + tr[i];

				val[i] = a + t * (b + t * (c + d * t));
				len += val[i] * val[i];
			}
			len = sqrt(len);
			outValue.Set(KDL::Rotation::Quaternion(val[0]/len, val[1]/len, val[2]/len, val[3]/len));
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
		RPI::InPort<KDL::Rotation> inFrom;
		RPI::InPort<KDL::Vector> inFromVel;
		RPI::InPort<KDL::Vector> inToVel;
		RPI::InPort<KDL::Rotation> inTo;
		RPI::OutPort<KDL::Rotation> outValue;

	};


	/**
	 * This class implements a rotation sliding average filter
	 */
	class RotationAverage: public RPI::Module
	{
	public:
		RotationAverage(std::string name, RPI::Net* net) :
				RPI::Module(name, net),
				inValue("inValue", this),
				inReset("inReset", this),
				outValue("outValue", this),
				propDuration("Duration", "Sliding window size (s)", 1),
				history(0), pos(-1), len(0)
		{
			setDescription(
					"A Rotation average module (calculating the sliding average of a rotation in a certain time frame)");
			this->ports()->addPort(&inValue, "Value to use");
			this->ports()->addPort(&inReset, "Reset port");
			this->ports()->addPort(&outValue, "Sliding average of the value");
			this->properties()->addProperty(&propDuration);

		}

		bool configureHook()
		{
			if(!portConnected(&inValue))
				return false;
			len = propDuration.get() / inNet->getNetFrequency();
			if(len <= 0)
				return false;
			history = new KDL::Rotation[len];
			pos = -1;
			return true;
		}

		void updateHook()
		{
			if(inReset.Get()) {
				pos = -1;
			}

			KDL::Rotation rot = inValue.Get();
			pos++;
			history[pos%len] = rot;

			KDL::Rotation rotInv = rot.Inverse();
			KDL::Vector sum = KDL::Vector::Zero();
			for(int i=std::max(0, pos-len+1); i <= pos; i++) {
				KDL::Vector elem = (rotInv * history[i%len]).GetRot();
				if(elem.x() == elem.x()) sum += elem;
			}
			sum = sum / std::min(len, pos+1);


			outValue.Set(rot * KDL::Rot(sum));
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
			delete[] history;
			history = 0;
		}

	protected:
		RPI::InPort<KDL::Rotation> inValue;
		RPI::InPort<bool> inReset;
		RPI::OutPort<KDL::Rotation> outValue;
		RPI::Property<double> propDuration;

	private:
		KDL::Rotation* history;
		int len;
		int pos;
	};


	/**
	 * This class implements vector sliding average filter
	 */
	class VectorAverage: public RPI::Module
	{
	public:
		VectorAverage(std::string name, RPI::Net* net) :
				RPI::Module(name, net),
				inValue("inValue", this),
				inReset("inReset", this),
				outValue("outValue", this),
				propDuration("Duration", "Sliding window size (s)", 1),
				history(0), pos(-1), len(0), sum()
		{
			setDescription(
					"A Vector average module (calculating the sliding average of a vector in a certain time frame)");
			this->ports()->addPort(&inValue, "Value to use");
			this->ports()->addPort(&inReset, "Reset port");
			this->ports()->addPort(&outValue, "Sliding average of the value");
			this->properties()->addProperty(&propDuration);

		}

		bool configureHook()
		{
			if(!portConnected(&inValue))
				return false;
			len = propDuration.get() / inNet->getNetFrequency();
			if(len <= 0)
				return false;
			history = new KDL::Vector[len];
			pos = -1;
			return true;
		}

		void updateHook()
		{
			if(inReset.Get()) {
				pos = -1;
				sum = KDL::Vector::Zero();
			}

			KDL::Vector vec = inValue.Get();
			pos++;
			if(pos >= len) {
				sum -= history[pos%len];
			}
			history[pos%len] = vec;
			sum += vec;
			outValue.Set(sum / std::min(len, pos+1));
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
			delete[] history;
			history = 0;
		}

	protected:
		RPI::InPort<KDL::Vector> inValue;
		RPI::InPort<bool> inReset;
		RPI::OutPort<KDL::Vector> outValue;
		RPI::Property<double> propDuration;

	private:
		KDL::Vector* history;
		KDL::Vector sum;
		int len;
		int pos;
	};



	/**
	 * This class implements a vector combiner module
	 */
	class VectorFromXYZ: public RPI::ActiveModule
	{

	public:
		VectorFromXYZ(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inX("inX", this), inY("inY", this), inZ("inZ", this), outValue(
						"outValue", this), propX("X", "X position", 0), propY("Y", "Y position", 0), propZ("Z",
						"Z position", 0)
		{
			setDescription("A vector from XYZ module (combines values for X, Y, Z into a vector datatype)");
			this->ports()->addPort(&inX, "X position");
			this->ports()->addPort(&inY, "Y position");
			this->ports()->addPort(&inZ, "Z position");
			this->ports()->addPort(&outValue, "Combined vector");

			this->properties()->addProperty(&propX);
			this->properties()->addProperty(&propY);
			this->properties()->addProperty(&propZ);
		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				double x = inX.Get(propX);
				double y = inY.Get(propY);
				double z = inZ.Get(propZ);
				outValue.Set(KDL::Vector(x, y, z));
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
		RPI::InPort<double> inX;
		RPI::InPort<double> inY;
		RPI::InPort<double> inZ;
		RPI::OutPort<KDL::Vector> outValue;

		RPI::Property<double> propX;
		RPI::Property<double> propY;
		RPI::Property<double> propZ;

	};

	/**
	 * This class implements a vector splitting module
	 */
	class VectorToXYZ: public RPI::ActiveModule
	{

	public:
		VectorToXYZ(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), outX("outX", this), outY("outY", this), outZ(
						"outZ", this)
		{
			setDescription("A vector to XYZ module (extracts values for X, Y, Z from a vector datatype)");
			this->ports()->addPort(&outX, "X position");
			this->ports()->addPort(&outY, "Y position");
			this->ports()->addPort(&outZ, "Z position");
			this->ports()->addPort(&inValue, "Input vector");
		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Vector frame = inValue.Get();

				outX.Set(frame.x());
				outY.Set(frame.y());
				outZ.Set(frame.z());
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
		RPI::InPort<KDL::Vector> inValue;

		RPI::OutPort<double> outX;
		RPI::OutPort<double> outY;
		RPI::OutPort<double> outZ;
	};

	/**
	 * This class implements a twist rotation module
	 */
	class VectorRotate: public RPI::ActiveModule
	{

	public:
		VectorRotate(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), inRot("inRot", this), outValue("outValue",
						this)
		{
			setDescription("A vector rotation module");
			this->ports()->addPort(&outValue, "Rotated vector");
			this->ports()->addPort(&inRot, "Input rotation");
			this->ports()->addPort(&inValue, "Input vector");
		}

		bool configureHook()
		{
			if (!inValue.connected())
				return false;
			if (!inRot.connected())
				return false;
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Vector twist = inValue.Get();
				KDL::Rotation rot = inRot.Get();
				outValue.Set(rot * twist);
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
		RPI::InPort<KDL::Vector> inValue;
		RPI::InPort<KDL::Rotation> inRot;

		RPI::OutPort<KDL::Vector> outValue;
	};

	/**
	 * This class implements a vector addition module
	 */
	class VectorAdd: public RPI::ActiveModule
	{

	public:

		VectorAdd(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
						"outValue", this)
		{
			setDescription("A vector addition module");
			this->ports()->addPort(&inFirst, "First vector");
			this->ports()->addPort(&inSecond, "Second vector");
			this->ports()->addPort(&outValue, "Combined vector (First + Second)");
		}

		bool configureHook()
		{
			if (!inFirst.connected() || !inSecond.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Vector first = inFirst.Get();
				KDL::Vector second = inSecond.Get();
				KDL::Vector ret = KDL::Vector(first + second);
				outValue.Set(ret);
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
		RPI::InPort<KDL::Vector> inFirst;
		RPI::InPort<KDL::Vector> inSecond;
		RPI::OutPort<KDL::Vector> outValue;
	};

	/**
	 * This class implements a vector dot product module
	 */
	class VectorDot: public RPI::ActiveModule
	{

	public:

		VectorDot(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
						"outValue", this)
		{
			setDescription("A vector dot product module");
			this->ports()->addPort(&inFirst, "First vector");
			this->ports()->addPort(&inSecond, "Second vector");
			this->ports()->addPort(&outValue, "Dot product (First * Second)");
		}

		bool configureHook()
		{
			if (!inFirst.connected() || !inSecond.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Vector first = inFirst.Get();
				KDL::Vector second = inSecond.Get();
				outValue.Set(KDL::dot(first, second));
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
		RPI::InPort<KDL::Vector> inFirst;
		RPI::InPort<KDL::Vector> inSecond;
		RPI::OutPort<double> outValue;
	};

	/**
	 * This class implements a vector scale module
	 */
	class VectorScale: public RPI::ActiveModule
	{

	public:

		VectorScale(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), inFactor("inFactor", this), outValue(
						"outValue", this), propFactor("Factor", "Scale factor", 1)
		{
			setDescription("A vector scale module");
			this->ports()->addPort(&inValue, "Input vector");
			this->ports()->addPort(&inFactor, "Scale factor");
			this->ports()->addPort(&outValue, "Scaled vector (Vector * Factor)");
			this->properties()->addProperty(&propFactor);
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
				KDL::Vector vector = inValue.Get();
				double factor = inFactor.Get(propFactor);
				outValue.Set(vector * factor);
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
		RPI::InPort<KDL::Vector> inValue;
		RPI::InPort<double> inFactor;
		RPI::OutPort<KDL::Vector> outValue;
		RPI::Property<double> propFactor;
	};

	/**
	 * This class implements a vector cross product module
	 */
	class VectorCross: public RPI::ActiveModule
	{

	public:

		VectorCross(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
						"outValue", this)
		{
			setDescription("A vector cross product module");
			this->ports()->addPort(&inFirst, "First vector");
			this->ports()->addPort(&inSecond, "Second vector");
			this->ports()->addPort(&outValue, "Combined vector (First x Second)");
		}

		bool configureHook()
		{
			if (!inFirst.connected() || !inSecond.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Vector first = inFirst.Get();
				KDL::Vector second = inSecond.Get();
				outValue.Set(first * second);
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
		RPI::InPort<KDL::Vector> inFirst;
		RPI::InPort<KDL::Vector> inSecond;
		RPI::OutPort<KDL::Vector> outValue;
	};

	/**
	 * This class implements a frame combiner module
	 */
	class FrameFromXYZ: public RPI::ActiveModule
	{

	public:
		FrameFromXYZ(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inX("inX", this), inY("inY", this), inZ("inZ", this), inRot("inRot",
						this), outValue("outValue", this), propX("X", "X position", 0), propY("Y", "Y position", 0), propZ(
						"Z", "Z position", 0), propA("A", "A rotation", 0), propB("B", "B rotation", 0), propC("C",
						"C rotation", 0)
		{
			setDescription(
					"A frame from XYZ module (combines values for X, Y, Z and rotation into a frame datatype) [deprecated]");
			this->ports()->addPort(&inX, "X position");
			this->ports()->addPort(&inY, "Y position");
			this->ports()->addPort(&inZ, "Z position");
			this->ports()->addPort(&inRot, "Rotation");
			this->ports()->addPort(&outValue, "Combined frame");

			this->properties()->addProperty(&propX);
			this->properties()->addProperty(&propY);
			this->properties()->addProperty(&propZ);
			this->properties()->addProperty(&propA);
			this->properties()->addProperty(&propB);
			this->properties()->addProperty(&propC);

		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				double x = inX.Get(propX);
				double y = inY.Get(propY);
				double z = inZ.Get(propZ);
				KDL::Rotation rot = inRot.Get(KDL::Rotation::RPY(propC.get(), propB.get(), propA.get()));
				outValue.Set(KDL::Frame(rot, KDL::Vector(x, y, z)));
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
		RPI::InPort<double> inX;
		RPI::InPort<double> inY;
		RPI::InPort<double> inZ;
		RPI::InPort<KDL::Rotation> inRot;
		RPI::OutPort<KDL::Frame> outValue;

		RPI::Property<double> propX;
		RPI::Property<double> propY;
		RPI::Property<double> propZ;
		RPI::Property<double> propA;
		RPI::Property<double> propB;
		RPI::Property<double> propC;

	};

	/**
	 * This class implements a frame combiner module
	 */
	class FrameFromPosRot: public RPI::ActiveModule
	{

	public:
		FrameFromPosRot(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inPos("inPos", this), inRot("inRot", this), outValue("outValue", this), propX(
						"X", "X position", 0), propY("Y", "Y position", 0), propZ("Z", "Z position", 0), propA("A",
						"A rotation", 0), propB("B", "B rotation", 0), propC("C", "C rotation", 0)
		{
			setDescription("A frame combiner module (combines position and rotation into a frame datatype)");
			this->ports()->addPort(&inPos, "Position");
			this->ports()->addPort(&inRot, "Rotation");
			this->ports()->addPort(&outValue, "Combined frame");

			this->properties()->addProperty(&propX);
			this->properties()->addProperty(&propY);
			this->properties()->addProperty(&propZ);
			this->properties()->addProperty(&propA);
			this->properties()->addProperty(&propB);
			this->properties()->addProperty(&propC);

		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Vector pos = inPos.Get(KDL::Vector(propX.get(), propY.get(), propZ.get()));
				KDL::Rotation rot = inRot.Get(KDL::Rotation::RPY(propC.get(), propB.get(), propA.get()));
				outValue.Set(KDL::Frame(rot, pos));
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
		RPI::InPort<KDL::Vector> inPos;
		RPI::InPort<KDL::Rotation> inRot;
		RPI::OutPort<KDL::Frame> outValue;

		RPI::Property<double> propX;
		RPI::Property<double> propY;
		RPI::Property<double> propZ;
		RPI::Property<double> propA;
		RPI::Property<double> propB;
		RPI::Property<double> propC;

	};

	/**
	 * This class implements a frame splitting module
	 */
	class FrameToPosRot: public RPI::ActiveModule
	{

	public:
		FrameToPosRot(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), outPosition("outPosition", this), outRotation(
						"outRotation", this)
		{
			setDescription("A frame splitter module (extracts position and rotation form a frame datatype)");
			this->ports()->addPort(&outPosition, "Position");
			this->ports()->addPort(&outRotation, "Rotation");
			this->ports()->addPort(&inValue, "Combined frame");
		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Frame frame = inValue.Get();
				outPosition.Set(frame.p);
				outRotation.Set(frame.M);
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
		RPI::InPort<KDL::Frame> inValue;

		RPI::OutPort<KDL::Vector> outPosition;
		RPI::OutPort<KDL::Rotation> outRotation;
	};

	/**
	 * This class implements a frame splitting module
	 */
	class FrameToXYZ: public RPI::ActiveModule
	{

	public:
		FrameToXYZ(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), outX("outX", this), outY("outY", this), outZ(
						"outZ", this), outRot("outRot", this)
		{
			setDescription(
					"A frame to XYZ module (extracts values for X, Y, Z and rotation from a frame datatype) [deprecated]");
			this->ports()->addPort(&outX, "X position");
			this->ports()->addPort(&outY, "Y position");
			this->ports()->addPort(&outZ, "Z position");
			this->ports()->addPort(&outRot, "Rotation");
			this->ports()->addPort(&inValue, "Combined frame");
		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Frame frame = inValue.Get();

				outX.Set(frame.p.x());
				outY.Set(frame.p.y());
				outZ.Set(frame.p.z());

				outRot.Set(frame.M);
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
		RPI::InPort<KDL::Frame> inValue;

		RPI::OutPort<double> outX;
		RPI::OutPort<double> outY;
		RPI::OutPort<double> outZ;
		RPI::OutPort<KDL::Rotation> outRot;
	};

	/**
	 * This class implements a frame linear interpolation module
	 */
	class FrameLerp: public RPI::ActiveModule
	{

	public:
		FrameLerp(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inFrom("inFrom", this), inTo("inTo", this), inAmount("inAmount", this), outValue(
						"outValue", this)
		{
			setDescription("Linear frame interpolation module");
			this->ports()->addPort(&inFrom, "Start frame");
			this->ports()->addPort(&inTo, "Destination frame");
			this->ports()->addPort(&inAmount, "Amount (0 = inFrom, 1 = inTo)");
			this->ports()->addPort(&outValue, "Interpolated frame");

		}

		bool configureHook()
		{
			if (!inFrom.connected())
				return false;
			if (!inTo.connected())
				return false;
			if (!inAmount.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Frame first = inFrom.Get(), second = inTo.Get();
				double b = inAmount.Get();
				double a = 1 - b;
				KDL::Rotation aim = first.M.Inverse() * second.M;
				KDL::Vector axis = aim.GetRot();
				KDL::Rotation rot = first.M * aim.Rot(axis, axis.Norm() * b);

				outValue.Set(
						KDL::Frame(
								rot,
								KDL::Vector(first.p.x() * a + second.p.x() * b, first.p.y() * a + second.p.y() * b,
										first.p.z() * a + second.p.z() * b)));
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
		RPI::InPort<KDL::Frame> inFrom;
		RPI::InPort<KDL::Frame> inTo;
		RPI::InPort<double> inAmount;
		RPI::OutPort<KDL::Frame> outValue;

	};

	/**
	 * This class implements applies a twist to a frame.
	 */
	class FrameAddTwist: public RPI::ActiveModule
	{

	public:
		FrameAddTwist(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inFrame("inFrame", this), inTwist("inTwist", this), outValue(
						"outValue", this)
		{
			setDescription("Applies a twist to a frame");
			this->ports()->addPort(&inFrame, "Frame");
			this->ports()->addPort(&inTwist, "Twist to add");
			this->ports()->addPort(&outValue, "Changed frame");

		}

		bool configureHook()
		{
			if (!inFrame.connected())
				return false;
			if (!inTwist.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Frame frame = inFrame.Get();
				KDL::Twist twist = inTwist.Get();

				outValue.Set(KDL::addDelta(frame, twist, inNet->getNetFrequency()));
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
		RPI::InPort<KDL::Frame> inFrame;
		RPI::InPort<KDL::Twist> inTwist;
		RPI::OutPort<KDL::Frame> outValue;

	};


	/**
	 * This class calculates a twist from two frames
	 */
	class TwistFromFrames: public RPI::ActiveModule
	{

	public:
		TwistFromFrames(std::string name, RPI::Net* net) :
				ActiveModule(name, net),
				inPrevFrame("inPrevFrame", this),
				inFrame("inFrame", this),
				outValue("outValue", this)
		{
			setDescription("Calculates a twist from two frames");
			this->ports()->addPort(&inPrevFrame, "Previous frame");
			this->ports()->addPort(&inFrame, "Current frame");
			this->ports()->addPort(&outValue, "Twist applied between the two frames");

		}

		bool configureHook()
		{
			if (!inPrevFrame.connected())
				return false;
			if (!inFrame.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Frame prevframe = inPrevFrame.Get();
				KDL::Frame frame = inFrame.Get();
				outValue.Set(KDL::diff(prevframe, frame, inNet->getNetFrequency()));
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
		RPI::InPort<KDL::Frame> inPrevFrame;
		RPI::InPort<KDL::Frame> inFrame;
		RPI::OutPort<KDL::Twist> outValue;

	};

	/**
	 * This class implements a frame inversion module
	 */
	class FrameInvert: public RPI::ActiveModule
	{

	public:
		FrameInvert(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), outValue("outValue", this)
		{
			setDescription("Frame inversion module");
			this->ports()->addPort(&inValue, "input Frame");
			this->ports()->addPort(&outValue, "inverted Frame");
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
				outValue.Set(inValue.Get().Inverse());
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
		RPI::InPort<KDL::Frame> inValue;
		RPI::OutPort<KDL::Frame> outValue;
	};

	/**
	 * This class implements a rotation combiner module
	 */
	class RotationFromABC: public RPI::ActiveModule
	{

	public:

		RotationFromABC(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inA("inA", this), inB("inB", this), inC("inC", this), outValue(
						"outValue", this), propA("A", "A rotation", 0), propB("B", "B rotation", 0), propC("C",
						"C rotation", 0)
		{
			setDescription("A rotation combiner (creates a rotation data flow from A, B, C angles)");
			this->ports()->addPort(&inA, "A rotation (around Z)");
			this->ports()->addPort(&inB, "B rotation (around Y)");
			this->ports()->addPort(&inC, "C rotation (around X)");
			this->ports()->addPort(&outValue, "Combined rotation");

			this->properties()->addProperty(&propA);
			this->properties()->addProperty(&propB);
			this->properties()->addProperty(&propC);

		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				double a = inA.Get(propA);
				double b = inB.Get(propB);
				double c = inC.Get(propC);
				outValue.Set(KDL::Rotation::RPY(c, b, a));
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
		RPI::InPort<double> inA;
		RPI::InPort<double> inB;
		RPI::InPort<double> inC;
		RPI::OutPort<KDL::Rotation> outValue;

		RPI::Property<double> propA;
		RPI::Property<double> propB;
		RPI::Property<double> propC;

	};

	/**
	 * This class implements a rotation combiner module
	 */
	class RotationFromAxisAngle: public RPI::ActiveModule
	{

	public:
		RotationFromAxisAngle(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inAxis("inAxis", this), inAngle(
						"inAngle", this), outValue("outValue", this), propX("X", "Axis x value", 1), propY("Y",
						"Axis y value", 0), propZ("Z", "Axis z value", 0), propAngle("Angle", "Rotation angle", 0)
		{
			setDescription(
					"A rotation combiner module (creates a rotation data flow from an axis and an angle to rotate around)");
			this->ports()->addPort(&inAxis, "Rotation axis (normalized)");
			this->ports()->addPort(&inAngle, "Angle to rotate");
			this->ports()->addPort(&outValue, "Combined rotation");

			this->properties()->addProperty(&propX);
			this->properties()->addProperty(&propY);
			this->properties()->addProperty(&propZ);
			this->properties()->addProperty(&propAngle);

		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Vector axis = inAxis.Get(KDL::Vector(propX.get(), propY.get(), propZ.get()));
				double angle = inAngle.Get(propAngle);
				outValue.Set(KDL::Rotation::Rot2(axis, angle));
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
		RPI::InPort<KDL::Vector> inAxis;
		RPI::InPort<double> inAngle;
		RPI::OutPort<KDL::Rotation> outValue;

		RPI::Property<double> propX;
		RPI::Property<double> propY;
		RPI::Property<double> propZ;
		RPI::Property<double> propAngle;

	};

	/**
	 * This class implements a rotation combiner module
	 */
	class RotationFromQuaternion: public RPI::ActiveModule
	{

	public:
		RotationFromQuaternion(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inX("inX", this), inY(
						"inY", this), inZ("inZ", this), inW("inW", this),
						outValue("outValue", this), propX("X", "Quaternion X value", 1), propY("Y",
						"Quaternion Y value", 0), propZ("Z", "Quaternion Z value", 0), propW("W", "Quaternion W value", 0)
		{
			setDescription(
					"A rotation combiner module (creates a rotation data flow from a quaternion)");
			this->ports()->addPort(&inX, "Quaternion X value");
			this->ports()->addPort(&inY, "Quaternion Y value");
			this->ports()->addPort(&inZ, "Quaternion Z value");
			this->ports()->addPort(&inW, "Quaternion W value");
			this->ports()->addPort(&outValue, "Combined rotation");

			this->properties()->addProperty(&propX);
			this->properties()->addProperty(&propY);
			this->properties()->addProperty(&propZ);
			this->properties()->addProperty(&propW);

		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				outValue.Set(KDL::Rotation::Quaternion(inX.Get(propX), inY.Get(propY), inZ.Get(propZ), inW.Get(propW)));
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
		RPI::Property<double> propX, propY, propZ, propW;
		RPI::InPort<double> inX, inY, inZ, inW;
		RPI::OutPort<KDL::Rotation> outValue;

	};

	/**
	 * This class implements a rotation splitter module
	 */
	class RotationToAxisAngle: public RPI::ActiveModule
	{

	public:
		RotationToAxisAngle(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), outAxis("outAxis", this),
				outAngle("outAngle", this)
		{
			setDescription(
					"A rotation splitter module (calculates normalized axis and angle from a rotation data flow)");
			this->ports()->addPort(&inValue, "Rotation");
			this->ports()->addPort(&outAxis, "Rotation axis (normalized)");
			this->ports()->addPort(&outAngle, "Angle to rotate");

		}

		bool configureHook()
		{
			if(!inValue.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Rotation rot = inValue.Get();
				KDL::Vector axis;
				outAngle.Set(rot.GetRotAngle(axis));
				outAxis.Set(axis);
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
		RPI::InPort<KDL::Rotation> inValue;
		RPI::OutPort<KDL::Vector> outAxis;
		RPI::OutPort<double> outAngle;
	};


	/**
	 * This class implements a rotation splitter module
	 */
	class RotationToQuaternion: public RPI::ActiveModule
	{

	public:
		RotationToQuaternion(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), outX("outX", this),
				outY("outY", this), outZ("outZ", this), outW("outW", this)
		{
			setDescription(
					"A rotation splitter module (calculates a quaternion from a rotation)");
			this->ports()->addPort(&inValue, "Rotation");
			this->ports()->addPort(&outX, "Quaternion X value");
			this->ports()->addPort(&outY, "Quaternion Y value");
			this->ports()->addPort(&outZ, "Quaternion Z value");
			this->ports()->addPort(&outW, "Quaternion W value");
		}

		bool configureHook()
		{
			if(!inValue.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				double x, y, z, w;
				KDL::Rotation rot = inValue.Get();
				rot.GetQuaternion(x, y, z, w);
				outX.Set(x);
				outY.Set(y);
				outZ.Set(z);
				outW.Set(w);
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
		RPI::InPort<KDL::Rotation> inValue;
		RPI::OutPort<double> outX, outY, outZ, outW;
	};

	/**
	 * This class implements a rotation splitter module
	 */
	class RotationInvert: public RPI::ActiveModule
	{

	public:
		RotationInvert(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), outValue("outValue", this)
		{
			setDescription(
					"A rotation inversion module (calculates the inverse rotation)");
			this->ports()->addPort(&inValue, "Rotation");
			this->ports()->addPort(&outValue, "Inverted rotation");

		}

		bool configureHook()
		{
			if(!inValue.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Rotation rot = inValue.Get();
				outValue.Set(rot.Inverse());
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
		RPI::InPort<KDL::Rotation> inValue;
		RPI::OutPort<KDL::Rotation> outValue;
	};


	/**
	 * This class implements a rotation combiner module
	 */
	class RotationToABC: public RPI::ActiveModule
	{

	public:

		RotationToABC(std::string name, RPI::Net* net) :
				ActiveModule(name, net), outA("outA", this), outB("outB", this), outC("outC", this), inValue(
						"inValue", this)
		{
			setDescription("A rotation splitter (extracts A, B, C values from Rotation type)");
			this->ports()->addPort(&outA, "A rotation (around Z)");
			this->ports()->addPort(&outB, "B rotation (around Y)");
			this->ports()->addPort(&outC, "C rotation (around X)");
			this->ports()->addPort(&inValue, "Combined rotation");

		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Rotation rotation = inValue.Get();
				double a, b, c;
				rotation.GetRPY(c, b, a);
				outA.Set(a);
				outB.Set(b);
				outC.Set(c);
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
		RPI::OutPort<double> outA;
		RPI::OutPort<double> outB;
		RPI::OutPort<double> outC;
		RPI::InPort<KDL::Rotation> inValue;
	};

	/**
	 * This class implements a rotation combiner module
	 */
	class RotationTransform: public RPI::ActiveModule
	{

	public:

		RotationTransform(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
						"outValue", this)
		{
			setDescription("A rotation transformer (combines two rotations)");
			this->ports()->addPort(&inFirst, "First rotation");
			this->ports()->addPort(&inSecond, "Second rotation");
			this->ports()->addPort(&outValue, "Combined rotation (First * Second)");

		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				outValue.Set(inFirst.Get() * inSecond.Get());
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
		RPI::InPort<KDL::Rotation> inFirst;
		RPI::InPort<KDL::Rotation> inSecond;
		RPI::OutPort<KDL::Rotation> outValue;
	};

	/**
	 * This class implements a frame transformer module
	 */
	class FrameTransform: public RPI::ActiveModule
	{

	public:

		FrameTransform(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
						"outValue", this)
		{
			setDescription("A frame transformer module (combines two frame data flows)");
			this->ports()->addPort(&inFirst, "First frame");
			this->ports()->addPort(&inSecond, "Second frame");
			this->ports()->addPort(&outValue, "Combined frame (First * Second)");

		}

		bool configureHook()
		{
			if (!inFirst.connected() || !inSecond.connected())
				return false;

			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Frame first = inFirst.Get();
				KDL::Frame second = inSecond.Get();
				KDL::Frame ret = KDL::Frame(first.M * second.M, first.M * second.p + first.p);
				outValue.Set(ret);
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
		RPI::InPort<KDL::Frame> inFirst;
		RPI::InPort<KDL::Frame> inSecond;
		RPI::OutPort<KDL::Frame> outValue;

	};

	/**
	 * This class implements a wrench combiner module
	 */
	class WrenchFromXYZ: public RPI::ActiveModule
	{

	public:
		WrenchFromXYZ(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inX("inX", this), inY("inY", this), inZ("inZ", this), inA("inA", this), inB(
						"inB", this), inC("inC", this), outValue("outValue", this), propX("X", "X force", 0), propY("Y",
						"Y force", 0), propZ("Z", "Z force", 0), propA("A", "A torque", 0), propB("B", "B torque", 0), propC(
						"C", "C torque", 0)
		{
			setDescription(
					"A wrench from XYZ module (combines values for X, Y, Z forces and A, B, C torques into a wrench datatype)");
			this->ports()->addPort(&inX, "X force");
			this->ports()->addPort(&inY, "Y force");
			this->ports()->addPort(&inZ, "Z force");
			this->ports()->addPort(&inA, "A torque");
			this->ports()->addPort(&inB, "B torque");
			this->ports()->addPort(&inC, "C torque");

			this->ports()->addPort(&outValue, "Combined wrench");

			this->properties()->addProperty(&propX);
			this->properties()->addProperty(&propY);
			this->properties()->addProperty(&propZ);
			this->properties()->addProperty(&propA);
			this->properties()->addProperty(&propB);
			this->properties()->addProperty(&propC);

		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				double x = inX.Get(propX);
				double y = inY.Get(propY);
				double z = inZ.Get(propZ);
				double a = inA.Get(propA);
				double b = inB.Get(propB);
				double c = inC.Get(propC);
				KDL::Vector force = KDL::Vector(x, y, z);
				KDL::Vector torque = KDL::Vector(c, b, a);
				outValue.Set(KDL::Wrench(force, torque));
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
		RPI::InPort<double> inX;
		RPI::InPort<double> inY;
		RPI::InPort<double> inZ;
		RPI::InPort<double> inA;
		RPI::InPort<double> inB;
		RPI::InPort<double> inC;

		RPI::OutPort<KDL::Wrench> outValue;

		RPI::Property<double> propX;
		RPI::Property<double> propY;
		RPI::Property<double> propZ;
		RPI::Property<double> propA;
		RPI::Property<double> propB;
		RPI::Property<double> propC;

	};

	/**
	 * This class implements a wrench combiner module
	 */
	class TwistFromXYZ: public RPI::ActiveModule
	{

	public:
		TwistFromXYZ(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inX("inX", this), inY("inY", this), inZ("inZ", this), inA("inA", this), inB(
						"inB", this), inC("inC", this), outValue("outValue", this), propX("X", "X velocity", 0), propY(
						"Y", "Y velocity", 0), propZ("Z", "Z velocity", 0), propA("A", "A velocity", 0), propB("B",
						"B velocity", 0), propC("C", "C velocity", 0)
		{
			setDescription(
					"A twist from XYZ module (combines values for X, Y, Z velocities and A, B, C velocities into a twist datatype) [deprecated]");
			this->ports()->addPort(&inX, "X velocity");
			this->ports()->addPort(&inY, "Y velocity");
			this->ports()->addPort(&inZ, "Z velocity");
			this->ports()->addPort(&inA, "A velocity");
			this->ports()->addPort(&inB, "B velocity");
			this->ports()->addPort(&inC, "C velocity");

			this->ports()->addPort(&outValue, "Combined twist");

			this->properties()->addProperty(&propX);
			this->properties()->addProperty(&propY);
			this->properties()->addProperty(&propZ);
			this->properties()->addProperty(&propA);
			this->properties()->addProperty(&propB);
			this->properties()->addProperty(&propC);

		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				double x = inX.Get(propX);
				double y = inY.Get(propY);
				double z = inZ.Get(propZ);
				double a = inA.Get(propA);
				double b = inB.Get(propB);
				double c = inC.Get(propC);
				KDL::Vector vel = KDL::Vector(x, y, z);
				KDL::Vector rot = KDL::Vector(c, b, a);
				outValue.Set(KDL::Twist(vel, rot));
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
		RPI::InPort<double> inX;
		RPI::InPort<double> inY;
		RPI::InPort<double> inZ;
		RPI::InPort<double> inA;
		RPI::InPort<double> inB;
		RPI::InPort<double> inC;

		RPI::OutPort<KDL::Twist> outValue;

		RPI::Property<double> propX;
		RPI::Property<double> propY;
		RPI::Property<double> propZ;
		RPI::Property<double> propA;
		RPI::Property<double> propB;
		RPI::Property<double> propC;

	};

	/**
	 * This class implements a wrench combiner module
	 */
	class TwistFromVelocities: public RPI::ActiveModule
	{

	public:
		TwistFromVelocities(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inTransVel("inTransVel", this), inRotVel("inRotVel", this), outValue(
						"outValue", this), propX("X", "X velocity", 0), propY("Y", "Y velocity", 0), propZ("Z",
						"Z velocity", 0), propRX("RX", "X rotation velocity", 0), propRY("RY", "Y rotation velocity",
						0), propRZ("RZ", "Z rotation velocity", 0)
		{
			setDescription(
					"A twist from velocity vector module (combines values for translational and rotational velocity into a twist datatype)");
			this->ports()->addPort(&inTransVel, "Translational velocity");
			this->ports()->addPort(&inRotVel, "Rotational velocity");

			this->ports()->addPort(&outValue, "Combined twist");

			this->properties()->addProperty(&propX);
			this->properties()->addProperty(&propY);
			this->properties()->addProperty(&propZ);
			this->properties()->addProperty(&propRX);
			this->properties()->addProperty(&propRY);
			this->properties()->addProperty(&propRZ);

		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Vector vel = inTransVel.Get(KDL::Vector(propX.get(), propY.get(), propZ.get()));
				KDL::Vector rot = inRotVel.Get(KDL::Vector(propRX.get(), propRY.get(), propRZ.get()));
				outValue.Set(KDL::Twist(vel, rot));
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
		RPI::InPort<KDL::Vector> inTransVel;
		RPI::InPort<KDL::Vector> inRotVel;

		RPI::OutPort<KDL::Twist> outValue;

		RPI::Property<double> propX;
		RPI::Property<double> propY;
		RPI::Property<double> propZ;
		RPI::Property<double> propRX;
		RPI::Property<double> propRY;
		RPI::Property<double> propRZ;

	};

	/**
	 * This class implements a wrench splitting module
	 */
	class TwistToVelocities: public RPI::ActiveModule
	{

	public:
		TwistToVelocities(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), outTransVel("outTransVel", this), outRotVel(
						"outRotVel", this)
		{
			setDescription(
					"A twist to velocities module (extracts translational and rotational velocities from a twist data type)");
			this->ports()->addPort(&outTransVel, "Translational velocity");
			this->ports()->addPort(&outRotVel, "Rotational velocity");
			this->ports()->addPort(&inValue, "Input twist");
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
				KDL::Twist twist = inValue.Get();
				outTransVel.Set(twist.vel);
				outRotVel.Set(twist.rot);
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
		RPI::InPort<KDL::Twist> inValue;

		RPI::OutPort<KDL::Vector> outTransVel;
		RPI::OutPort<KDL::Vector> outRotVel;
	};

	/**
	 * This class implements a twist rotation module
	 */
	class TwistRotate: public RPI::ActiveModule
	{

	public:
		TwistRotate(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), inRot("inRot", this), outValue("outValue",
						this)
		{
			setDescription("A twist rotation module");
			this->ports()->addPort(&outValue, "Rotated twist");
			this->ports()->addPort(&inRot, "Input rotation");
			this->ports()->addPort(&inValue, "Input twist");
		}

		bool configureHook()
		{
			if (!inValue.connected())
				return false;
			if (!inRot.connected())
				return false;
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Twist twist = inValue.Get();
				KDL::Rotation rot = inRot.Get();
				outValue.Set(rot * twist);
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
		RPI::InPort<KDL::Twist> inValue;
		RPI::InPort<KDL::Rotation> inRot;

		RPI::OutPort<KDL::Twist> outValue;
	};

	class TwistAdd: public RPI::ActiveModule
	{

		/**
		 * This class implements a twist add module
		 */
	public:
		TwistAdd(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue(
						"outValue", this)
		{
			setDescription("A twist add module");
			this->ports()->addPort(&outValue, "sum");
			this->ports()->addPort(&inFirst, "summand one");
			this->ports()->addPort(&inSecond, "summand two");
		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Twist twist1 = inFirst.Get();
				KDL::Twist twist2 = inSecond.Get();
				outValue.Set(twist1 + twist2);
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
		RPI::InPort<KDL::Twist> inFirst;
		RPI::InPort<KDL::Twist> inSecond;

		RPI::OutPort<KDL::Twist> outValue;
	};

	/**
	 * This class implements a wrench splitting module
	 */
	class TwistChangeCenter: public RPI::ActiveModule
	{

	public:
		TwistChangeCenter(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), inVector("inVector", this), outValue(
						"outValue", this)
		{
			setDescription("A twist change rotatation center module");
			this->ports()->addPort(&outValue, "Changed twist");
			this->ports()->addPort(&inVector, "Vector to move the rotation center");
			this->ports()->addPort(&inValue, "Input twist");
		}

		bool configureHook()
		{
			if (!inValue.connected())
				return false;
			if (!inVector.connected())
				return false;
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Twist twist = inValue.Get();
				KDL::Vector ref = inVector.Get();
				outValue.Set(twist.RefPoint(ref));
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
		RPI::InPort<KDL::Twist> inValue;
		RPI::InPort<KDL::Vector> inVector;

		RPI::OutPort<KDL::Twist> outValue;
	};

	/**
	 * This class implements a wrench splitting module
	 */
	class TwistToXYZ: public RPI::ActiveModule
	{

	public:
		TwistToXYZ(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), outX("outX", this), outY("outY", this), outZ(
						"outZ", this), outA("outA", this), outB("outB", this), outC("outC", this)
		{
			setDescription(
					"A twist to XYZ module (extracts values for X, Y, Z velocities and A, B, C velocities) [deprecated]");
			this->ports()->addPort(&outX, "X velocity");
			this->ports()->addPort(&outY, "Y velocity");
			this->ports()->addPort(&outZ, "Z velocity");
			this->ports()->addPort(&outA, "A velocity");
			this->ports()->addPort(&outB, "B velocity");
			this->ports()->addPort(&outC, "C velocity");
			this->ports()->addPort(&inValue, "Combined twist");
		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Twist twist = inValue.Get();

				outX.Set(twist.vel.x());
				outY.Set(twist.vel.y());
				outZ.Set(twist.vel.z());
				outA.Set(twist.rot.z());
				outB.Set(twist.rot.y());
				outC.Set(twist.rot.x());
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
		RPI::InPort<KDL::Twist> inValue;

		RPI::OutPort<double> outX;
		RPI::OutPort<double> outY;
		RPI::OutPort<double> outZ;
		RPI::OutPort<double> outA;
		RPI::OutPort<double> outB;
		RPI::OutPort<double> outC;
	};

	/**
	 * This class implements a wrench splitting module
	 */
	class WrenchToXYZ: public RPI::ActiveModule
	{

	public:
		WrenchToXYZ(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inValue("inValue", this), outX("outX", this), outY("outY", this), outZ(
						"outZ", this), outA("outA", this), outB("outB", this), outC("outC", this)
		{
			setDescription("A wrench to XYZ module (extracts values for X, Y, Z forces and A, B, C torques)");
			this->ports()->addPort(&outX, "X force");
			this->ports()->addPort(&outY, "Y force");
			this->ports()->addPort(&outZ, "Z force");
			this->ports()->addPort(&outA, "A torque");
			this->ports()->addPort(&outB, "B torque");
			this->ports()->addPort(&outC, "C torque");
			this->ports()->addPort(&inValue, "Combined wrench");
		}

		bool configureHook()
		{
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				KDL::Wrench wrench = inValue.Get();

				outX.Set(wrench.force.x());
				outY.Set(wrench.force.y());
				outZ.Set(wrench.force.z());
				outA.Set(wrench.torque.z());
				outB.Set(wrench.torque.y());
				outC.Set(wrench.torque.x());
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
		RPI::InPort<KDL::Wrench> inValue;

		RPI::OutPort<double> outX;
		RPI::OutPort<double> outY;
		RPI::OutPort<double> outZ;
		RPI::OutPort<double> outA;
		RPI::OutPort<double> outB;
		RPI::OutPort<double> outC;
	};



	/**
	 * This class implements a frame online trajectory generator
	 */
	class FrameOTG: public RPI::ActiveModule
	{

	public:
		FrameOTG(std::string name, RPI::Net* net) :
				ActiveModule(name, net), inCurPos("inCurPos", this), inCurVel("inCurVel", this), inDestPos("inDestPos",
						this), inDestVel("inDestVel", this), inMaxTransVel("inMaxTransVel", this), inMaxTransAcc("inMaxTransAcc", this), inMaxRotVel(
						"inMaxRotVel", this), inMaxRotAcc("inMaxRotAcc", this), propMaxTransVel("MaxTransVel",
						"Maximum translational velocity", 1), propMaxTransAcc("MaxTransAcc",
						"Maximum translational acceleration", 1), propMaxRotVel("MaxRotVel",
						"Maximum rotational velocity", 1), propMaxRotAcc("MaxRotAcc", "Maximum rotational acceleration",
						1), outPos("outPos", this), outVel("outVel", this)
		{
			setDescription("An online trajectory generator for frames");
			this->ports()->addPort(&inCurPos, "Current position (transformation from reference frame to moving frame)");
			this->ports()->addPort(&inCurVel, "Current velocity (velocity of moving frame relative to reference frame, expressed in reference frame)");
			this->ports()->addPort(&inDestPos, "Destination position (transformation from reference frame to moving frame)");
			this->ports()->addPort(&inDestVel, "Destination velocity (velocity of target frame relative to reference frame, expressed in reference frame)");
			this->ports()->addPort(&outPos, "New position (position of moving frame relative to reference frame)");
			this->ports()->addPort(&outVel, "New velocity (velocity of moving frame relative to reference frame, expressed in reference frame)");

			this->ports()->addPort(&inMaxTransVel, "Maximum translational velocity");
			this->ports()->addPort(&inMaxTransAcc, "Maximum translational acceleration");
			this->ports()->addPort(&inMaxRotVel, "Maximum rotational velocity");
			this->ports()->addPort(&inMaxRotAcc, "Maximum rotational acceleration");

			this->properties()->addProperty(&propMaxTransVel);
			this->properties()->addProperty(&propMaxTransAcc);
			this->properties()->addProperty(&propMaxRotVel);
			this->properties()->addProperty(&propMaxRotAcc);

		}

		bool configureHook()
		{
			if(!inCurPos.connected()) return false;
			if(!inCurVel.connected()) return false;
			if(!inDestPos.connected()) return false;
			return true;
		}

		void updateHook()
		{
			if (active())
			{
				// read variables
				KDL::Frame curPos = inCurPos.Get();
				KDL::Frame destPos = inDestPos.Get();
				KDL::Twist curVel = inCurVel.Get();
				KDL::Twist destVel = inDestVel.Get();
				double maxTransVel = inMaxTransVel.Get(propMaxTransVel);
				double maxTransAcc = inMaxTransAcc.Get(propMaxTransAcc);
				double maxRotVel = inMaxRotVel.Get(propMaxRotVel);
				double maxRotAcc = inMaxRotAcc.Get(propMaxRotAcc);
				double dt = getInNet()->getNetFrequency();

				// reduce maximum velocity to allow decelerating to goal
				KDL::Frame diff = curPos.Inverse() * destPos;
				double transDist = diff.p.Norm(), rotDist = diff.M.GetRot().Norm();
				double decelTransVel = sqrt(2 * maxTransAcc * 0.9 * transDist);
				double decelRotVel = sqrt(2 * maxRotAcc * 0.9 * rotDist);
				double maxDecelTransVel = decelTransVel < maxTransVel ? decelTransVel : maxTransVel;
				double maxDecelRotVel = decelRotVel < maxRotVel ? decelRotVel : maxRotVel;

				// find out twist to goal
				KDL::Twist goalVel = KDL::diff(curPos, destPos, dt);
				double transVel = goalVel.vel.Norm();
				double rotVel = goalVel.rot.Norm();

				// reduce goal twist to given maximum values
				double transFactor = 1, rotFactor = 1;
				if(transVel > maxDecelTransVel)
					transFactor = transVel / maxDecelTransVel;
				if(rotVel > maxDecelRotVel)
					rotFactor = rotVel / maxDecelRotVel;

				double factor = rotFactor > transFactor ? rotFactor : transFactor;
				goalVel = goalVel / factor;
				goalVel = goalVel + destVel;

				transVel = goalVel.vel.Norm();
				rotVel = goalVel.rot.Norm();
				transFactor = 1; rotFactor = 1;
				if (transVel > maxTransVel)
					transFactor = transVel / maxTransVel;
				if (rotVel > maxRotVel)
					rotFactor = rotVel / maxRotVel;
				factor = rotFactor > transFactor ? rotFactor : transFactor;
				goalVel = goalVel / factor;

				// adapt goal twist to current velocity and maximum acceleration
				KDL::Twist goalAcc = KDL::diff(curVel, goalVel, dt);
				double transAcc = goalAcc.vel.Norm();
				double rotAcc = goalAcc.rot.Norm();
				transFactor = 1, rotFactor = 1;
				if(transAcc > maxTransAcc)
					transFactor = transAcc / maxTransAcc;
				if(rotAcc > maxRotAcc)
					rotFactor = rotAcc / maxRotAcc;
				factor = rotFactor > transFactor ? rotFactor : transFactor;
				goalAcc = goalAcc / factor;

				// calculate new twist
				KDL::Twist cmdVel = KDL::addDelta(curVel, goalAcc, dt);
				outVel.Set(cmdVel);
				// calculate corresponding position
				KDL::Frame cmdPos = KDL::addDelta(curPos, cmdVel, dt);
				outPos.Set(cmdPos);
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
		RPI::InPort<KDL::Twist> inCurVel;
		RPI::InPort<KDL::Frame> inCurPos;
		RPI::InPort<KDL::Twist> inDestVel;
		RPI::InPort<KDL::Frame> inDestPos;
		RPI::InPort<double> inMaxTransVel, inMaxTransAcc, inMaxRotVel, inMaxRotAcc;
		RPI::Property<double> propMaxTransVel, propMaxTransAcc, propMaxRotVel, propMaxRotAcc;
		RPI::OutPort<KDL::Frame> outPos;
		RPI::OutPort<KDL::Twist> outVel;
	};



}

