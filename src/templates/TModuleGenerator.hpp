/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2010-2017 ISSE, University of Augsburg 
 */

#ifndef TMODULEGENERATOR_HPP_
#define TMODULEGENERATOR_HPP_

#include <string>

#include "TValue.hpp"
#include "TArray.hpp"
#include "TArithmetic.hpp"
#include "TComparison.hpp"
#include "TSnapshot.hpp"
#include "TNetcomm.hpp"
#include "TInterNetcomm.hpp"
#include "TPre.hpp"

namespace RPI
{
	/**
	 * \brief Generator for standard template modules.
	 *
	 * Convenience method for generating type-specific variants of template modules,
	 * so developer only has to call one method per data type.
	 *
	 * There is a hierarchy of data types
	 * - arithmetic
	 * - comparable
	 * - general
	 *
	 * A data type supporting arithmetic operation also has to support compation and general operation,
	 * but not other way round
	 * \todo maybe think of better system for hierarchy
	 */
	template<class T, class HelperTool>
	class TModuleGenerator
	{
	public:
		/**
		 * \brief Generate module factories for general data types
		 *
		 * Generate module factories for modules that support any data type
		 */
		static void generateModulesGeneral(ExtensionLoaderInfo info, const std::string& type, const std::string& extension)
		{
			// generic modules
			info.registerModuleFactory<TValue<T> >(type + "Value", extension);
			info.registerModuleFactory<TIsNull<T> >(type + "IsNull", extension);
			info.registerModuleFactory<TSetNull<T> >(type + "SetNull", extension);
			info.registerModuleFactory<TAtTime<T> >(type + "AtTime", extension);
			info.registerModuleFactory<TSnapshot<T> >(type + "Snapshot", extension);
			info.registerModuleFactory<TPre<T> >(type + "Pre", extension);
			info.registerModuleFactory<TArray<T> >(type + "Array", extension);
			info.registerModuleFactory<TArrayGet<T> >(type + "ArrayGet", extension);
			info.registerModuleFactory<TArraySet<T> >(type + "ArraySet", extension);
			info.registerModuleFactory<TArrayIsNull<T> >(type + "ArrayIsNull", extension);
			info.registerModuleFactory<TArraySetNull<T> >(type + "ArraySetNull", extension);
			info.registerModuleFactory<TArraySlice<T> >(type + "ArraySlice", extension);
			info.registerModuleFactory<TConditional<T> >(type + "Conditional", extension);

			// Netcomm modules
			info.registerModuleFactory<TNetcomm_In<T> >(type + "NetcommIn", extension);
			info.registerModuleFactory<TArrayNetcomm_In<T> >(type + "ArrayNetcommIn", extension);
			info.registerModuleFactory<TNetcomm_Out<T> >(type + "NetcommOut", extension);
			info.registerModuleFactory<TArrayNetcomm_Out<T> >(type + "ArrayNetcommOut", extension);

			// InterNetcomm modules
			info.registerModuleFactory<TInterNetcomm_In<T> >(type + "InterNetcommIn", extension);
			info.registerModuleFactory<TInterArrayNetcomm_In<T> >(type + "ArrayInterNetcommIn", extension);
			info.registerModuleFactory<TInterNetcomm_Out<T> >(type + "InterNetcommOut", extension);
			info.registerModuleFactory<TInterArrayNetcomm_Out<T> >(type + "ArrayInterNetcommOut", extension);

		}

		/**
		 * \brief Generate module factories for comparable data types
		 *
		 * Generate module factories for modules that support any data type with
		 * support of the \> operator
		 */
		static void generateModulesComparable(ExtensionLoaderInfo info, const std::string& type,
				const std::string& extension)
		{
			generateModulesGeneral(info, type, extension);

			info.registerModuleFactory<TEquals<T, HelperTool> >(type + "Equals", extension);
			info.registerModuleFactory<TGreater<T> >(type + "Greater", extension);
		}

		/**
		 * \brief Generate module factories for data types with arithmetic support
		 *
		 *  Generate module factories for modules that support any data type which
		 *  supports all arithmetic operators such as +, -, *, /
		 */
		static void generateModulesArithmetic(ExtensionLoaderInfo info, const std::string& type, const std::string& extension)
		{
			generateModulesComparable(info, type, extension);

			info.registerModuleFactory<TAdd<T> >(type + "Add", extension);
			info.registerModuleFactory<TDivide<T> >(type + "Divide", extension);
			info.registerModuleFactory<TMultiply<T> >(type + "Multiply", extension);
		}

	};
}

#endif /* TMODULEGENERATOR_HPP_ */
