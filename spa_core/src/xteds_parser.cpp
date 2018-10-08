#include <iostream>
#include <fstream>
#include <sstream> // operation of string stream
#include <memory>

namespace spa
{

XtedsParser::XtedsDocPtr XtedsParser::deserialize(const std::string &data)
{
    // Load data form C++ format string.
    std::istringstream str(data);
    rapidxml::file<> sstr(str);

    // Parse to xml_document
    XtedsPtr doc = std::make_shared<rapidxml::xml_document<> >();
    doc->parse<0>(sstr.data());

    return doc;
}

char * XtedsParser::getXtedsName(XtedsPtr &doc)
{
    rapidxml::xml_node<> *node = doc->first_node("xTEDS");
    rapidxml::xml_attribute<> *attr = node->first_attribute("name");
    return attr->value();
}

//char * XtedsParser::getComponentName(XtedsPtr &doc)
//{

//}

}
