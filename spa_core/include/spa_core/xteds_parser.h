#ifndef XTEDS_PARSER_H
#define XTEDS_PARSER_H

#include <string>
#include <spa_core/xteds_repository.h>
#include <rapidxml/rapidxml.h>

namespace spa
{

class XtedsParser
{
public:
    XtedsParser() {}

    typedef std::shared_ptr<rapidxml::xml_document<> > XtedsDocPtr;

    /// Takes an XML version of an xTEDS and deserializes it into an object.
    /// \param data Reference of string format xTEDS data.
    /// \return Reference of shared pointer to the xTEDS object.
    XtedsDocPtr deserialize(const std::string &data);

    /// Get the name of the parsed xTEDS.
    /// \param doc Reference of shared pointer to the xTEDS object.
    /// \return Name of the xTEDS.
    char * getXtedsName(XtedsDocPtr &doc);

    /// Get the component name (described in the Application/Device node).
    /// \param doc Reference of shared pointer to the xTEDS object.
    /// \return Name of the Application/Device.
    //char * getComponentName(XtedsPtr &doc);
private:

};

}

#endif
