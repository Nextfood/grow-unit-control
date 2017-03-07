#ifndef __JsonTools_h__
#define __JsonTools_h__


#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

class JsonTools {
public:


	static std::string ptreeToJson(const boost::property_tree::ptree& pt) {
	    std::stringstream s;
	    boost::property_tree::json_parser::write_json(s, pt, false);
	    return s.str();
	}

	static boost::property_tree::ptree jsonToPtree(const std::string& json) {
	    std::stringstream s;
	    s << json;
	    boost::property_tree::ptree pt;
	    boost::property_tree::json_parser::read_json(s, pt);
	    return pt;
	}

};

#endif 

