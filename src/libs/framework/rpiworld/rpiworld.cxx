/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#include <rcc/Extension.hpp>
#include <rcc/TypeKit.hpp>

#include <templates/TModuleGenerator.hpp>
#include <rcc/version_bin.h>

#include "rpi/typemodules.hpp"
#include "rpi/typekits.hpp"

namespace World
{
	using namespace RPI;

	extern "C"
	{
		const std::string ext_rpiworld = "rpiworld";

		rcc_bin_version_t RTT_EXPORT version()
		{
			return rcc_bin_version_id;
		}

		void RTT_EXPORT load(RPI::ExtensionLoaderInfo info)
		{
			TModuleGenerator<KDL::Frame, void>::generateModulesGeneral(info, "World::Frame", ext_rpiworld);
			TModuleGenerator<KDL::Twist, void>::generateModulesGeneral(info, "World::Twist", ext_rpiworld);
			TModuleGenerator<KDL::Wrench, void>::generateModulesGeneral(info, "World::Wrench", ext_rpiworld);
			TModuleGenerator<KDL::Vector, void>::generateModulesGeneral(info, "World::Vector", ext_rpiworld);
			TModuleGenerator<KDL::Rotation, void>::generateModulesGeneral(info, "World::Rotation", ext_rpiworld);

			info.registerModuleFactory<VectorAverage>("World::VectorAverage", ext_rpiworld);
			info.registerModuleFactory<VectorRotate>("World::VectorRotate", ext_rpiworld);
			info.registerModuleFactory<VectorDot>("World::VectorDot", ext_rpiworld);
			info.registerModuleFactory<VectorCross>("World::VectorCross", ext_rpiworld);
			info.registerModuleFactory<VectorScale>("World::VectorScale", ext_rpiworld);
			info.registerModuleFactory<VectorFromXYZ>("World::VectorFromXYZ", ext_rpiworld);
			info.registerModuleFactory<VectorToXYZ>("World::VectorToXYZ", ext_rpiworld);
			info.registerModuleFactory<VectorAdd>("World::VectorAdd", ext_rpiworld);

			info.registerModuleFactory<FrameFromPosRot>("World::FrameFromPosRot", ext_rpiworld);
			info.registerModuleFactory<FrameToPosRot>("World::FrameToPosRot", ext_rpiworld);
			info.registerModuleFactory<FrameFromXYZ>("World::FrameFromXYZ", ext_rpiworld);
			info.registerModuleFactory<FrameToXYZ>("World::FrameToXYZ", ext_rpiworld);
			info.registerModuleFactory<FrameLerp>("World::FrameLerp", ext_rpiworld);
			info.registerModuleFactory<FrameInvert>("World::FrameInvert", ext_rpiworld);
			info.registerModuleFactory<FrameAddTwist>("World::FrameAddTwist", ext_rpiworld);
			info.registerModuleFactory<FrameTransform>("World::FrameTransform", ext_rpiworld);
			info.registerModuleFactory<FrameOTG>("World::FrameOTG", ext_rpiworld);

			info.registerModuleFactory<RotationAverage>("World::RotationAverage", ext_rpiworld);
			info.registerModuleFactory<RotationFromABC>("World::RotationFromABC", ext_rpiworld);
			info.registerModuleFactory<RotationFromAxisAngle>("World::RotationFromAxisAngle", ext_rpiworld);
			info.registerModuleFactory<RotationToAxisAngle>("World::RotationToAxisAngle", ext_rpiworld);
			info.registerModuleFactory<RotationFromQuaternion>("World::RotationFromQuaternion", ext_rpiworld);
			info.registerModuleFactory<RotationToQuaternion>("World::RotationToQuaternion", ext_rpiworld);
			info.registerModuleFactory<RotationToABC>("World::RotationToABC", ext_rpiworld);
			info.registerModuleFactory<RotationInvert>("World::RotationInvert", ext_rpiworld);
			info.registerModuleFactory<RotationTransform>("World::RotationTransform", ext_rpiworld);
			info.registerModuleFactory<RotationBezier>("World::RotationBezier", ext_rpiworld);

			info.registerModuleFactory<WrenchFromXYZ>("World::WrenchFromXYZ", ext_rpiworld);
			info.registerModuleFactory<WrenchToXYZ>("World::WrenchToXYZ", ext_rpiworld);

			info.registerModuleFactory<TwistRotate>("World::TwistRotate", ext_rpiworld);
			info.registerModuleFactory<TwistChangeCenter>("World::TwistChangeCenter", ext_rpiworld);
			info.registerModuleFactory<TwistFromVelocities>("World::TwistFromVelocities", ext_rpiworld);
			info.registerModuleFactory<TwistToVelocities>("World::TwistToVelocities", ext_rpiworld);
			info.registerModuleFactory<TwistFromXYZ>("World::TwistFromXYZ", ext_rpiworld);
			info.registerModuleFactory<TwistToXYZ>("World::TwistToXYZ", ext_rpiworld);
			info.registerModuleFactory<TwistAdd>("World::TwistAdd", ext_rpiworld);
			info.registerModuleFactory<TwistFromFrames>("World::TwistFromFrames", ext_rpiworld);



			TypeKits::getInstance()->registerTypeKit(typeid(KDL::Vector), new VectorTypeKit());
			TypeKits::getInstance()->registerTypeKit(typeid(KDL::Rotation), new RotationTypeKit());
			TypeKits::getInstance()->registerTypeKit(typeid(KDL::Frame), new FrameTypeKit());
			TypeKits::getInstance()->registerTypeKit(typeid(KDL::Twist), new TwistTypeKit());
			TypeKits::getInstance()->registerTypeKit(typeid(KDL::Wrench), new WrenchTypeKit());
			TypeKits::getInstance()->registerTypeKit(typeid(Array<KDL::Vector>), new ArrayTypeKit<KDL::Vector>("World::Vector[]"));
			TypeKits::getInstance()->registerTypeKit(typeid(Array<KDL::Rotation>), new ArrayTypeKit<KDL::Rotation>("World::Rotation[]"));
			TypeKits::getInstance()->registerTypeKit(typeid(Array<KDL::Frame>), new ArrayTypeKit<KDL::Frame>("World::Frame[]"));
			TypeKits::getInstance()->registerTypeKit(typeid(Array<KDL::Twist>), new ArrayTypeKit<KDL::Twist>("World::Twist[]"));
			TypeKits::getInstance()->registerTypeKit(typeid(Array<KDL::Wrench>), new ArrayTypeKit<KDL::Wrench>("World::Wrench[]"));

		}

		void RTT_EXPORT unload(RPI::ExtensionLoaderInfo info)
		{
			info.unregisterExtension(ext_rpiworld);

			TypeKits::getInstance()->unregisterTypeKit(typeid(KDL::Vector));
			TypeKits::getInstance()->unregisterTypeKit(typeid(KDL::Rotation));
			TypeKits::getInstance()->unregisterTypeKit(typeid(KDL::Frame));
			TypeKits::getInstance()->unregisterTypeKit(typeid(KDL::Twist));
			TypeKits::getInstance()->unregisterTypeKit(typeid(KDL::Wrench));

		}
	}

}
