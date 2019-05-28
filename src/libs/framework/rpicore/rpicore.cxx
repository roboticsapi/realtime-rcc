/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <rcc/Extension.hpp>
#include <templates/TModuleGenerator.hpp>
#include "rtt/os/TimeService.hpp"
#include <rcc/TypeKitT.hpp>

#include "rpi/logic.hpp"
#include "rpi/motion.hpp"
#include "rpi/OTG.hpp"
#include "rpi/OTG2.hpp"
#include "rpi/ValueCoding.hpp"

#include <rcc/version_bin.h>

namespace Core
{
	using namespace RPI;

	class IntTool
	{
	public:
		static inline int abs(const int& a)
		{
			return ::abs(a);
		}
	};

	class DoubleTool
	{
	public:
		static inline double abs(const double& a)
		{
			return ::fabs(a);
		}
	};

	extern "C"
	{
		const std::string ext_rpicore = "rpicore";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			TModuleGenerator<bool, void>::generateModulesGeneral(info, "Core::Boolean", ext_rpicore);
			TModuleGenerator<double, DoubleTool>::generateModulesArithmetic(info, "Core::Double", ext_rpicore);
			TModuleGenerator<int, IntTool>::generateModulesArithmetic(info, "Core::Int", ext_rpicore);

			info.registerModuleFactory<BooleanAnd>("Core::BooleanAnd", ext_rpicore);
			info.registerModuleFactory<BooleanOr>("Core::BooleanOr", ext_rpicore);
			info.registerModuleFactory<BooleanNot>("Core::BooleanNot", ext_rpicore);
			info.registerModuleFactory<BooleanLastN>("Core::BooleanLastN", ext_rpicore);
			info.registerModuleFactory<BooleanHistory>("Core::BooleanHistory", ext_rpicore);
			info.registerModuleFactory<DoubleSquareRoot>("Core::DoubleSquareRoot", ext_rpicore);
			info.registerModuleFactory<DoubleModulo>("Core::DoubleModulo", ext_rpicore);
			info.registerModuleFactory<DoubleAtan2>("Core::DoubleAtan2", ext_rpicore);
			info.registerModuleFactory<DoubleSin>("Core::DoubleSin", ext_rpicore);
			info.registerModuleFactory<DoubleCos>("Core::DoubleCos", ext_rpicore);
			info.registerModuleFactory<DoubleTan>("Core::DoubleTan", ext_rpicore);
			info.registerModuleFactory<DoubleAsin>("Core::DoubleAsin", ext_rpicore);
			info.registerModuleFactory<DoubleAcos>("Core::DoubleAcos", ext_rpicore);
			info.registerModuleFactory<DoublePower>("Core::DoublePower", ext_rpicore);
			info.registerModuleFactory<DoubleLog>("Core::DoubleLog", ext_rpicore);
			info.registerModuleFactory<DoubleAverage>("Core::DoubleAverage", ext_rpicore);
			info.registerModuleFactory<Trigger>("Core::Trigger", ext_rpicore);
			info.registerModuleFactory<CycleTime>("Core::CycleTime", ext_rpicore);
			info.registerModuleFactory<EdgeDetection>("Core::EdgeDetection", ext_rpicore);
			info.registerModuleFactory<IntFromDouble>("Core::IntFromDouble", ext_rpicore);
			info.registerModuleFactory<DoubleFromInt>("Core::DoubleFromInt", ext_rpicore);

			info.registerModuleFactory<CubicBezier>("Core::CubicBezier", ext_rpicore);
			info.registerModuleFactory<Lerp>("Core::Lerp", ext_rpicore);
			info.registerModuleFactory<Rampify>("Core::Rampify", ext_rpicore);
			info.registerModuleFactory<Clock>("Core::Clock", ext_rpicore);
			info.registerModuleFactory<Interval>("Core::Interval", ext_rpicore);
			info.registerModuleFactory<OTG>("Core::OTG", ext_rpicore);
			info.registerModuleFactory<OTG2>("Core::OTG2", ext_rpicore);
			info.registerModuleFactory<DoubleBezier>("Core::DoubleBezier", ext_rpicore);

			info.registerModuleFactory<TPre<RTT::os::TimeService::nsecs> >("Core::TimePre", ext_rpicore);
			info.registerModuleFactory<TimeNet>("Core::TimeNet", ext_rpicore);
			info.registerModuleFactory<TimeWall>("Core::TimeWall", ext_rpicore);
			info.registerModuleFactory<TimeDiff>("Core::TimeDiff", ext_rpicore);
			info.registerModuleFactory<TimeAdd>("Core::TimeAdd", ext_rpicore);
			info.registerModuleFactory<TAtTime<RTT::os::TimeService::nsecs> >("Core::TimeAtTime", ext_rpicore);
			info.registerModuleFactory<TimeHistory>("Core::TimeHistory", ext_rpicore);
			info.registerModuleFactory<TimeConsistentRange>("Core::TimeConsistentRange", ext_rpicore);

			info.registerModuleFactory<DecToBin>("Core::DecToBin", ext_rpicore);
			info.registerModuleFactory<DecToBCD>("Core::DecToBCD", ext_rpicore);
			info.registerModuleFactory<BCDToDec>("Core::BCDToDec", ext_rpicore);
			info.registerModuleFactory<BinToDec>("Core::BinToDec", ext_rpicore);

			TypeKits::getInstance()->registerTypeKit(typeid(RTT::os::TimeService::nsecs), new TypeKitTStream<RTT::os::TimeService::nsecs>("Core::nsecs"));

		}

		void RTT_EXPORT unload(RPI::ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_rpicore);

			TypeKits::getInstance()->unregisterTypeKit(typeid(RTT::os::TimeService::nsecs));
		}
	}

}
