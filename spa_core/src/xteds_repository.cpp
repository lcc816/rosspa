#include <spa_core/xteds_repository.h>
#include <cstring>
#include <iostream>
#include <fstream>

namespace spa
{

/*----------- Implementation of XtedsIndex -------------*/

bool XtedsIndex::insert(XtedsNode * const node, const std::string &name)
{
    return table.insert(XtedsTable::value_type(node, name)).second;
}

bool XtedsIndex::erase(XtedsNode * const node)
{
    if (table.erase(node))
    {
        return true;
    }
    return false;
}

std::string XtedsIndex::getName(XtedsNode * const node)
{
    return table[node];
}

/*---------- Implementation of XtedsRepository ----------*/

void XtedsRepository::index(const XtedsPtr &doc)
{
    // Store the object of xTEDS.
    xtedsList.push_back(doc);
    // Traverse all nodes of the xtEDS
    index_node(doc->first_node());
}

inline void XtedsRepository::index_node(XtedsNode * const node)
{
    if (node == nullptr)
    {
        return;
    }

    if (XtedsAttribute *attr = node->first_attribute("name"))
    {
        std::string nodeName(attr->value());

        if (std::strcmp(node->name(), "Application") || std::strcmp(node->name(), "Device"))
        {
            componentNameIndex.insert(node, nodeName);
        }
        else if (std::strcmp(node->name(), "Interface"))
        {
            interfaceNameIndex.insert(node, nodeName);
        }
        else if (std::strcmp(node->name(), "DataMsg") || std::strcmp(node->name(), "CommandMsg") || \
            std::strcmp(node->name(), "DataReplyMsg"))
        {
            itemNameIndex.insert(node, nodeName);
        }
    }

    for (XtedsNode *child = node->first_node(); child; child = child->next_sibling())
    {
        index_node(child);
    }
}

} // namespace spa
