/*
 * rpixmlutil.hpp
 *
 *  Created on: 20.08.2013
 *      Author: visteimi
 */

#ifndef RPIXMLUTIL_HPP_
#define RPIXMLUTIL_HPP_

#include "rapidxml.hpp"

namespace rapidxml
{
	template<class Ch>
	std::string getAttribute(rapidxml::xml_node<Ch>* node, const std::string& name, const std::string& def = "")
	{
		xml_attribute<Ch>* attr = node->first_attribute(name.c_str(), 0, false);
		if(attr)
			return attr->value();
		else
			return def;
	}
}


#endif /* RPIXMLUTIL_HPP_ */
