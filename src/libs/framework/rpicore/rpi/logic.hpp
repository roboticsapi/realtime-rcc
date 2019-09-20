/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

namespace Core
{
	/**
	 * This class implements a boolean and
	 */
	class BooleanAnd: public RPI::Module
	{

	public:
		BooleanAnd(std::string name, RPI::Net* net) :
			Module(name, net), inFirst("inFirst", this), inSecond("inSecond", this),
					outValue("outValue", this, false), propFirst("First", "First value", true),
					propSecond("Second", "Second value", true)
		{
			setDescription("A Boolean and module");
			this->ports()->addPort(&inFirst, "First value");
			this->ports()->addPort(&inSecond, "Second value");
			this->ports()->addPort(&outValue, "Result (First && Second)");

			this->properties()->addProperty(&propFirst);
			this->properties()->addProperty(&propSecond);
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
			bool first = inFirst.GetNoNull(propFirst);
			bool second = inSecond.GetNoNull(propSecond);
			outValue.Set(first && second);
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<bool> inFirst;
		RPI::InPort<bool> inSecond;
		RPI::OutPort<bool> outValue;

		RPI::Property<bool> propFirst;
		RPI::Property<bool> propSecond;
	};


	/**
	 * This class implements a double square root
	 */
	class IntFromDouble: public RPI::Module
	{

	public:
		IntFromDouble(std::string name, RPI::Net* net) :
			Module(name, net), inValue("inValue", this),
					outValue("outValue", this, false), propValue("Value", "Value", true)
		{
			setDescription("Casts double to int");
			this->ports()->addPort(&inValue, "Value");
			this->ports()->addPort(&outValue, "Result: int(Value)");

			this->properties()->addProperty(&propValue);
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
			double value = inValue.GetNoNull(propValue);
			outValue.Set(value);
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<double> inValue;
		RPI::OutPort<int> outValue;

		RPI::Property<double> propValue;
	};


	/**
	 * This class implements a double square root
	 */
	class DoubleFromInt: public RPI::Module
	{

	public:
		DoubleFromInt(std::string name, RPI::Net* net) :
			Module(name, net), inValue("inValue", this),
					outValue("outValue", this, false), propValue("Value", "Value", true)
		{
			setDescription("Casts int to double");
			this->ports()->addPort(&inValue, "Value");
			this->ports()->addPort(&outValue, "Result: double(Value)");

			this->properties()->addProperty(&propValue);
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
			double value = inValue.GetNoNull(propValue);
			outValue.Set(value);
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<int> inValue;
		RPI::OutPort<double> outValue;

		RPI::Property<int> propValue;
	};

	/**
	 * This class implements a double average module (fixed time window)
	 */
	class DoubleAverage: public RPI::StatefulModule
	{

	public:
		DoubleAverage(std::string name, RPI::Net* net) :
			StatefulModule(name, net), inValue("inValue", this), outValue("outValue", this),
					propDuration("Duration", "Sliding window size (s)", 1), history(),
					current(0), count(0), sum(0), numWritten(0)
		{
			setDescription("A Double average module (calculates the sliding average of a double value in a certain time frame)");
			this->ports()->addPort(&inValue, "Value to use");
			this->ports()->addPort(&outValue, "Sliding average of the value");

			this->properties()->addProperty(&propDuration);
		}

		bool configureHook()
		{
			count = propDuration.get() / inNet->getNetFrequency();
			current = 0;
			sum = 0;
			numWritten = 0;
			if (!inValue.connected())
				return false;
			if (count < 1)
				return false;

			history = std::vector<double>(count, false);
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
				if(reset()) {
					current = 0;
					sum = 0;
					numWritten = 0;
				}

				if (numWritten > current)
					sum -= history[current];
				history[current] = inValue.Get();
				sum += history[current];
				current = (current + 1) % count;
				numWritten++;
				outValue.Set(sum / (numWritten > current ? count : numWritten));
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
		RPI::OutPort<double> outValue;

		RPI::Property<double> propDuration;

	private:
		std::vector<double> history;
		int count, current, numWritten;
		double sum;
	};

	/**
	 * This class implements a double natural logarithm module
	 */
	class DoubleLog: public RPI::Module
	{

	public:
		DoubleLog(std::string name, RPI::Net* net) :
			Module(name, net), inValue("inValue", this), outValue("outValue", this),
					propValue("Value", "Value", 0)
		{
			setDescription("A Double natural logarithm module (calculates the natural logarithm)");
			this->ports()->addPort(&inValue, "Value");
			this->ports()->addPort(&outValue, "Result value");

			this->properties()->addProperty(&propValue);
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
			double value = inValue.Get(propValue);
			outValue.Set(log(value));
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

		RPI::Property<double> propValue;
	};



	/**
	 * This class implements a double power module
	 */
	class DoublePower: public RPI::Module
	{

	public:
		DoublePower(std::string name, RPI::Net* net) :
			Module(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue("outValue", this),
					propFirst("First", "First value (base)", 0), propSecond("Second", "Second value (exponent)", 0)

		{
			setDescription("A Double power module (calculates the specified number raised to the specified power)");
			this->ports()->addPort(&inFirst, "First value (base)");
			this->ports()->addPort(&inSecond, "Second value (exponent)");
			this->ports()->addPort(&outValue, "Result value (base raised to the power exponent)");

			this->properties()->addProperty(&propFirst);
			this->properties()->addProperty(&propSecond);
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
			double first = inFirst.Get(propFirst);
			double second = inSecond.Get(propSecond);
			outValue.Set(pow(first, second));
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<double> inFirst;
		RPI::InPort<double> inSecond;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propFirst;
		RPI::Property<double> propSecond;
	};


	/**
	 * This class implements a double square root
	 */
	class DoubleSquareRoot: public RPI::Module
	{

	public:
		DoubleSquareRoot(std::string name, RPI::Net* net) :
			Module(name, net), inValue("inValue", this),
					outValue("outValue", this, false), propValue("Value", "Value", true)
		{
			setDescription("A double square root");
			this->ports()->addPort(&inValue, "Value");
			this->ports()->addPort(&outValue, "Result (sqrt(Value))");

			this->properties()->addProperty(&propValue);
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
			double value = inValue.GetNoNull(propValue);
			outValue.Set(sqrt(value));
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

		RPI::Property<double> propValue;
	};

	/*
	 * This class implements the atan2 function
	 */
	class DoubleAtan2 : public RPI::Module
	{
	public:
		DoubleAtan2(std::string name, RPI::Net* net) : Module(name, net), inX("inX", this), inY("inY", this), outValue("outValue", this, false), propX("X", "X", true), propY("Y", "Y", true)
		{
			setDescription("Calculates the arc tangent of y/x");
			this->ports()->addPort(&inY, "y");
			this->ports()->addPort(&inX, "x");
			this->ports()->addPort(&outValue, "Result (atan2(y,x))");

			this->properties()->addProperty(&propY);
			this->properties()->addProperty(&propX);
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
			double y = inY.GetNoNull(propY);
			double x = inX.GetNoNull(propX);
			outValue.Set(atan2(y,x));
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}
	private:
		RPI::InPort<double> inY;
		RPI::InPort<double> inX;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propY;
		RPI::Property<double> propX;


	};

	/*
	 * This class implements the tan function
	 */
	class DoubleTan : public RPI::Module
	{
	public:
		DoubleTan(std::string name, RPI::Net* net) : Module(name, net), inValue("inValue", this), outValue("outValue", this, false), propValue("Value", "Value", true)
		{
			setDescription("Calculates the tangent");
			this->ports()->addPort(&inValue, "value");
			this->ports()->addPort(&outValue, "Result (tan(value))");

			this->properties()->addProperty(&propValue);
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
			double value = inValue.GetNoNull(propValue);
			outValue.Set(tan(value));
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}
	private:
		RPI::InPort<double> inValue;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propValue;

	};

	/*
	 * This class implements the sin function
	 */
	class DoubleSin : public RPI::Module
	{
	public:
		DoubleSin(std::string name, RPI::Net* net) : Module(name, net), inValue("inValue", this), outValue("outValue", this, false), propValue("Value", "Value", true)
		{
			setDescription("Calculates the sine");
			this->ports()->addPort(&inValue, "value");
			this->ports()->addPort(&outValue, "Result (sin(value))");

			this->properties()->addProperty(&propValue);
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
			double value = inValue.GetNoNull(propValue);
			outValue.Set(sin(value));
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}
	private:
		RPI::InPort<double> inValue;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propValue;

	};

	/*
	 * This class implements the cos function
	 */
	class DoubleCos : public RPI::Module
	{
	public:
		DoubleCos(std::string name, RPI::Net* net) : Module(name, net), inValue("inValue", this), outValue("outValue", this, false), propValue("Value", "Value", true)
		{
			setDescription("Calculates the cosine");
			this->ports()->addPort(&inValue, "value");
			this->ports()->addPort(&outValue, "Result (tan(value))");

			this->properties()->addProperty(&propValue);
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
			double value = inValue.GetNoNull(propValue);
			outValue.Set(cos(value));
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}
	private:
		RPI::InPort<double> inValue;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propValue;

	};

	/*
	 * This class implements the tan function
	 */
	class DoubleAcos : public RPI::Module
	{
	public:
		DoubleAcos(std::string name, RPI::Net* net) : Module(name, net), inValue("inValue", this), outValue("outValue", this, false), propValue("Value", "Value", true)
		{
			setDescription("Calculates the arc cosine");
			this->ports()->addPort(&inValue, "value");
			this->ports()->addPort(&outValue, "Result (acos(value))");

			this->properties()->addProperty(&propValue);
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
			double value = inValue.GetNoNull(propValue);
			outValue.Set(acos(value));
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}
	private:
		RPI::InPort<double> inValue;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propValue;

	};

	/*
	 * This class implements the asin function
	 */
	class DoubleAsin : public RPI::Module
	{
	public:
		DoubleAsin(std::string name, RPI::Net* net) : Module(name, net), inValue("inValue", this), outValue("outValue", this, false), propValue("Value", "Value", true)
		{
			setDescription("Calculates the arc sine");
			this->ports()->addPort(&inValue, "value");
			this->ports()->addPort(&outValue, "Result (asin(value))");

			this->properties()->addProperty(&propValue);
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
			double value = inValue.GetNoNull(propValue);
			outValue.Set(asin(value));
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}
	private:
		RPI::InPort<double> inValue;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propValue;

	};


	/*
	 * This class implements the modulo function
	 */
	class DoubleModulo : public RPI::Module
	{
	public:
		DoubleModulo(std::string name, RPI::Net* net) : Module(name, net), inFirst("inFirst", this), inSecond("inSecond", this), outValue("outValue", this, false), propFirst("First", "First value", true), propSecond("Second", "Second value", true)
		{
			setDescription("Calculates the division rest ");
			this->ports()->addPort(&inFirst, "First");
			this->ports()->addPort(&inSecond, "Second");
			this->ports()->addPort(&outValue, "Result (First % Second)");

			this->properties()->addProperty(&propFirst);
			this->properties()->addProperty(&propSecond);
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
			double first = inFirst.GetNoNull(propFirst);
			double second = inSecond.GetNoNull(propSecond);
			outValue.Set(::fmod(first, second));
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}
	private:
		RPI::InPort<double> inFirst;
		RPI::InPort<double> inSecond;
		RPI::OutPort<double> outValue;

		RPI::Property<double> propFirst;
		RPI::Property<double> propSecond;


	};

	/**
	 * This class implements a boolean not
	 */
	class BooleanNot: public RPI::Module
	{

	public:
		BooleanNot(std::string name, RPI::Net* net) :
			Module(name, net), inValue("inValue", this), outValue("outValue", this, false),
					propValue("Value", "Input value", false)
		{
			setDescription("A Boolean not module");
			this->ports()->addPort(&inValue, "Input value");
			this->ports()->addPort(&outValue, "Result (!Input)");

			this->properties()->addProperty(&propValue);
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
			bool value = inValue.GetNoNull(propValue);
			outValue.Set(!value);
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<bool> inValue;
		RPI::OutPort<bool> outValue;

		RPI::Property<bool> propValue;
	};

	/**
	 * This class implements a boolean or
	 */
	class BooleanOr: public RPI::Module
	{

	public:
		BooleanOr(std::string name, RPI::Net* net) :
			Module(name, net), inFirst("inFirst", this), inSecond("inSecond", this),
					outValue("outValue", this, true), propFirst("First", "First value", false),
					propSecond("Second", "Second value", false)
		{
			setDescription("A Boolean or module");
			this->ports()->addPort(&inFirst, "First value");
			this->ports()->addPort(&inSecond, "Second value");
			this->ports()->addPort(&outValue, "Result (First || Second)");

			this->properties()->addProperty(&propFirst);
			this->properties()->addProperty(&propSecond);
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
			bool first = inFirst.GetNoNull(propFirst);
			bool second = inSecond.GetNoNull(propSecond);
			outValue.Set(first || second);
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<bool> inFirst;
		RPI::InPort<bool> inSecond;
		RPI::OutPort<bool> outValue;

		RPI::Property<bool> propFirst;
		RPI::Property<bool> propSecond;
	};

	/**
	 * This class implements a boolean history module (fixed number of cycles)
	 */
	class BooleanLastN: public RPI::StatefulModule
	{

	public:
		BooleanLastN(std::string name, RPI::Net* net) :
			StatefulModule(name, net), inValue("inValue", this), outNone("outNone", this),
					outAll("outAll", this), outCount("outCount", this),
					propCount("Count", "Sliding window size (ticks)", 1), history(),
					current(0), count(0), numTrue(0), numWritten(0)
		{
			setDescription("A Boolean history module (counts the number of 'true' during a fixed number of cycles)");
			this->ports()->addPort(&inValue, "Value to count");
			this->ports()->addPort(&outNone, "None (Value has always been false)");
			this->ports()->addPort(&outAll, "All (Value has always been true)");
			this->ports()->addPort(&outCount, "Number of 'true's");

			this->properties()->addProperty(&propCount);
		}

		bool configureHook()
		{
			if (!inValue.connected())
				return false;
			count = propCount.get();
			current = 0;
			numTrue = 0;
			history = std::vector<bool>(count, false);
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
				if(reset())  {
					numWritten = 0;
					current = 0;
					numTrue = 0;
				}
				if (numWritten > current && history[current])
					numTrue--;
				history[current] = inValue.Get();
				if (history[current])
					numTrue++;

				numWritten++;
				current = (current + 1) % count;
				outCount.Set(numTrue);
				outAll.Set(numTrue == count);
				outNone.Set(numTrue == 0);
			}
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<bool> inValue;
		RPI::OutPort<bool> outNone;
		RPI::OutPort<bool> outAll;
		RPI::OutPort<int> outCount;

		RPI::Property<int> propCount;

	private:
		std::vector<bool> history;
		int current, count, numTrue, numWritten;
	};

	/**
	 * This class implements a boolean history module (fixed time window)
	 */
	class BooleanHistory: public RPI::StatefulModule
	{

	public:
		BooleanHistory(std::string name, RPI::Net* net) :
			StatefulModule(name, net), inValue("inValue", this), outNone("outNone", this),
					outAll("outAll", this), outAmount("outAmount", this),
					propDuration("Duration", "Sliding window size (s)", 1), history(),
					current(0), count(0), numTrue(0), numWritten(0)
		{
			setDescription("A Boolean history module (counts the amount of 'true' in a certain time frame)");
			this->ports()->addPort(&inValue, "Value to count");
			this->ports()->addPort(&outNone, "None (Value has always been false)");
			this->ports()->addPort(&outAll, "All (Values has always been true)");
			this->ports()->addPort(&outAmount, "Amount of 'true's (0..1)");

			this->properties()->addProperty(&propDuration);
		}

		bool configureHook()
		{
			count = propDuration.get() / inNet->getNetFrequency();
			current = 0;
			numTrue = 0;
			numWritten = 0;
			if (!inValue.connected())
				return false;
			if (count < 1)
				return false;

			history = std::vector<bool>(count, false);
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
				if(reset()) {
					current = 0;
					numTrue = 0;
					numWritten = 0;
				}

				if (numWritten > current && history[current])
					numTrue--;
				history[current] = inValue.Get();
				if (history[current])
					numTrue++;
				current = (current + 1) % count;
				numWritten++;
				outAmount.Set(numTrue / count);
				outAll.Set(numTrue == count);
				outNone.Set(numTrue == 0);
			}
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<bool> inValue;
		RPI::OutPort<bool> outNone;
		RPI::OutPort<bool> outAll;
		RPI::OutPort<double> outAmount;

		RPI::Property<double> propDuration;

	private:
		std::vector<bool> history;
		int current, count, numTrue, numWritten;
	};


	/**
	 * This class implements a time history module that can search for the occurance of a given time
	 */
	class TimeHistory: public RPI::StatefulModule
	{

	public:
		TimeHistory(std::string name, RPI::Net* net) :
			StatefulModule(name, net), inValue("inValue", this), inTime("inTime", this), outAge("outAge", this),
					propMaxAge("MaxAge", "Maximum age supported (s)", 1), history(), count(0), current(0)
		{
			setDescription("A Time history module (finds when a certain time was provided)");
			this->ports()->addPort(&inValue, "Time stamp provided (haystack)");
			this->ports()->addPort(&inTime, "Time stamp to search for (needle)");
			this->ports()->addPort(&outAge, "Time when the time stamp occured [s]");

			this->properties()->addProperty(&propMaxAge);
		}

		bool configureHook()
		{
			count = propMaxAge.get() / inNet->getNetFrequency();
			current = 0;
			if (!inValue.connected())
				return false;
			if (count < 1)
				return false;

			history = std::vector<RTT::os::TimeService::nsecs>(count, 0);
			step = 1; for (int i = count; i > 0; step <<= 1) i >>= 1;
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
				if(reset()) {
					current = 0;
				}
				history[current % count] = inValue.Get();

				int pos = current;
				int ret = -1;
				int step = this->step;
				while (true) {
					if (pos > current) {
						pos -= step;
					} else if (pos < 0 || pos < current - count) {
						pos += step;
					} else {
						if (history[pos % count] <= inTime.Get()) {
							if(ret < pos) ret = pos;
						}
						if (history[pos % count] <= inTime.Get()) {
							pos += step;
						} else {
							pos -= step;
						}
					}
					if (step > 0)
						step >>= 1;
					else
						break;
				}
				if(ret != -1)
					outAge.Set((current - ret) * inNet->getNetFrequency());

				current++;
			}
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<RTT::os::TimeService::nsecs> inValue;
		RPI::InPort<RTT::os::TimeService::nsecs> inTime;
		RPI::OutPort<double> outAge;
		RPI::Property<double> propMaxAge;

	private:
		std::vector<RTT::os::TimeService::nsecs> history;
		int count, current;
		int step;
	};



	/**
	 * This class implements a time history module for two time ranges, returning a time range containing a range of each of the inputs
	 */
	class TimeConsistentRange: public RPI::StatefulModule
	{

	private:
		int bisect(RTT::os::TimeService::nsecs start, RTT::os::TimeService::nsecs end, std::vector<RTT::os::TimeService::nsecs> starts, std::vector<RTT::os::TimeService::nsecs> ends, int startIndex, int endIndex) {
			int pos = startIndex;
			int ret = -1;
			int step = this->step;
			while (step > startIndex - endIndex)
				step >>= 1;
			while (true) {

				if (pos > startIndex) {
					pos -= step;
				} else if (pos < 0 || pos < endIndex) {
					pos += step;
				} else {
					if ((std::max(end, ends[pos % count]) - std::min(start, starts[pos % count])) / 1e9 < propMaxSize.get()) {
						if(ret < pos) ret = pos;
					}
					if (starts[pos % count] < start) {
						pos += step;
					} else {
						pos -= step;
					}
				}
				if (step > 0)
					step >>= 1;
				else
					break;
			}
			return ret;
		}


	public:
		TimeConsistentRange(std::string name, RPI::Net* net) :
			StatefulModule(name, net),
			inFirstStart("inFirstStart", this), inFirstEnd("inFirstEnd", this),
			inSecondStart("inSecondStart", this), inSecondEnd("inSecondEnd", this),
			outStart("outStart", this), outEnd("outEnd", this),
			propMaxSize("MaxSize", "Maximum duration of the returned time range (s)", 1),
			propMaxAge("MaxAge", "Maximum age supported (s)", 1),
			history1s(), history1e(), history2s(), history2e(), count(0), current(0), lastFirst(-1), lastSecond(-1), step(0)

		{
			setDescription("A Time range history module (from the history of the two given time ranges, finds a bounded time range that contains both)");
			this->ports()->addPort(&inFirstStart, "Start time of the first range");
			this->ports()->addPort(&inFirstEnd, "End time of the first range");
			this->ports()->addPort(&inSecondStart, "Start time of the second range");
			this->ports()->addPort(&inSecondEnd, "End time of the second range");

			this->ports()->addPort(&outStart, "Start time of the resulting range");
			this->ports()->addPort(&outEnd, "End time of the resulting range");

			this->properties()->addProperty(&propMaxSize);
			this->properties()->addProperty(&propMaxAge);
		}

		bool configureHook()
		{
			count = propMaxAge.get() / inNet->getNetFrequency();
			step = 1; for (int i = count; i > 0; step <<= 1) i >>= 1;
			current = 0;
			current = 0;

			if(!portConnected(&inFirstStart) || !portConnected(&inSecondStart) ||
					!portConnected(&inFirstEnd) || !portConnected(&inSecondEnd))
				return false;
			if (count < 1)
				return false;

			history1s = std::vector<RTT::os::TimeService::nsecs>(count, 0);
			history1e = std::vector<RTT::os::TimeService::nsecs>(count, 0);
			history2s = std::vector<RTT::os::TimeService::nsecs>(count, 0);
			history2e = std::vector<RTT::os::TimeService::nsecs>(count, 0);
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
				if(reset()) {
					current = 0;
				}

				int first = -1, second = -1;
				// value 1 modified
				if (current == 0 || history1s[(current - 1) % count] != history1s[current % count]
						|| history1e[(current - 1) % count] == history1e[current % count]) {
					int found = bisect(history1s[current % count], history1e[current % count], history2s, history2e, current,
							current - count); // lastSecond);
					if (found != -1) {
						second = found;
						first = current;
					}
				}

				// value 2 modified
				if (current == 0 || history2s[(current - 1) % count] != history2s[current % count]
						|| history2e[(current - 1) % count] == history2e[current % count]) {
					int found = bisect(history2s[current % count], history2e[current % count], history1s, history1e, current,
							current - count); // lastFirst);
					if (found != -1) {
						second = current;
						first = found;
					}
				}

				if (second == -1 || first == -1) {
					second = lastSecond;
					first = lastFirst;
				}

				if (second != -1 && first != -1) {
					RTT::os::TimeService::nsecs start = std::min(history1s[first % count], history2s[second % count]);
					RTT::os::TimeService::nsecs end = std::max(history1e[first % count], history2e[second % count]);
					lastFirst = first;
					lastSecond = second;
					outStart.Set(start);
					outEnd.Set(end);
				}

				current++;

//
//
//				history1s[current % count] = inFirstStart.Get();
//				history1e[current % count] = inFirstEnd.Get();
//				history2s[current % count] = inSecondStart.Get();
//				history2e[current % count] = inSecondEnd.Get();
//
//				int first = current, second = current;
//				while(first > current - count && first >= 0 && second > current - count && second >= 0) {
//					RTT::os::TimeService::nsecs start, end;
//					start = std::min(history1s[first % count], history2s[second % count]);
//					end = std::max(history1e[first % count], history2e[second % count]);
//					if((end - start) / 1e9 < propMaxSize.get()) {
//						outStart.Set(start);
//						outEnd.Set(end);
//						break;
//					}
//					if(history1e[first % count] > history2s[second % count]) {
//						first --;
//					} else {
//						second --;
//					}
//				}
//				current++;
//
			}
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<RTT::os::TimeService::nsecs> inFirstStart;
		RPI::InPort<RTT::os::TimeService::nsecs> inFirstEnd;
		RPI::InPort<RTT::os::TimeService::nsecs> inSecondStart;
		RPI::InPort<RTT::os::TimeService::nsecs> inSecondEnd;
		RPI::OutPort<RTT::os::TimeService::nsecs> outStart;
		RPI::OutPort<RTT::os::TimeService::nsecs> outEnd;
		RPI::Property<double> propMaxSize;
		RPI::Property<double> propMaxAge;

	private:
		std::vector<RTT::os::TimeService::nsecs> history1s, history1e, history2s, history2e;
		int count, current;
		int step, lastFirst, lastSecond;
	};


	/**
	 * This class implements a trigger
	 */
	class Trigger: public RPI::StatefulModule
	{

	public:
		Trigger(std::string name, RPI::Net* net) :
			StatefulModule(name, net), inOn("inOn", this), inOff("inOff", this), outActive("outActive", this),
					propOn("On", "Activation", false), propOff("Off", "Deactivation", false), enabled(false),
					done(false)
		{
			setDescription("A trigger module (providing a Boolean value that can be switched on or off)");
			this->ports()->addPort(&inOn, "Activation trigger");
			this->ports()->addPort(&inOff, "Deactivation trigger");
			this->ports()->addPort(&outActive, "Activation");

			this->properties()->addProperty(&propOn);
			this->properties()->addProperty(&propOff);
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
			if (active())
			{
				if(reset()) {
					enabled = false; done = false;
				}

				if (!done)
				{
					if (inOn.GetNoNull(propOn))
					{
						enabled = true;
					}

					if (inOff.GetNoNull(propOff))
					{
						enabled = false;
						done = true;
					}

				}

				outActive.Set(enabled);
			}
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<bool> inOn;
		RPI::InPort<bool> inOff;
		RPI::OutPort<bool> outActive;

		RPI::Property<bool> propOn;
		RPI::Property<bool> propOff;
	private:
		bool enabled, done;
	};

	/**
	 * This class implements a monitor which outputs the current cycle time
	 */
	class CycleTime: public RPI::Module
	{
	public:
		CycleTime(std::string name, RPI::Net* net) :
			Module(name, net), outValue("outValue", this)
		{
			this->ports()->addPort(&outValue, "cycle time in current net");
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
			outValue.Set(inNet->getNetFrequency());
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::OutPort<double> outValue;
	};


	/**
	 * This class implements a trigger
	 */
	class EdgeDetection: public RPI::StatefulModule
	{

	public:
		EdgeDetection(std::string name, RPI::Net* net) :
			StatefulModule(name, net), inValue("inValue", this), outValue("outValue", this),
					propDirection("Direction", "Direction of edge", true), last(false)
		{
			setDescription("A edge detection module (detects rising [Direction=true] or falling [Direction=false] edges)");
			this->ports()->addPort(&inValue, "Activation trigger");
			this->ports()->addPort(&outValue, "Deactivation trigger");

			this->properties()->addProperty(&propDirection);
		}

		bool configureHook()
		{
			if(!inValue.connected())
				return false;
			last = !propDirection.get();
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
				bool direction = propDirection.get();
				bool value = inValue.GetNoNull(!direction);
				if(reset()) {
					last = !direction;
				}

				if(value == direction && last != direction) {
					outValue.Set(true);
				} else {
					outValue.Set(false);
				}
				last = value;
			}
		}
		void stopHook()
		{
		}
		void cleanupHook()
		{
		}

	protected:
		RPI::InPort<bool> inValue;
		RPI::OutPort<bool> outValue;

		RPI::Property<bool> propDirection;
	private:
		bool last;
	};


}

