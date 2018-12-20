#include <spa_core/xteds_repository.h>
#include <cstring>
#include <iostream>
#include <fstream>

namespace spa
{

/*----------- Implementation of XtedsIndex -------------*/

void XtedsIndex::insert(const std::string &name, XtedsNode * const node)
{
    table.insert(XtedsTable::value_type(name, node));
}

void XtedsIndex::erase(const std::string &name, XtedsNode * const node)
{
    std::pair<iterator, iterator> range = findItems(name);
    if (range.first == table.end())
        return;

    for (iterator it = range.first; it != range.second; )
        if (it->second == node)
            it = table.erase(it);
        else
            ++it;
}

std::pair<XtedsIndex::iterator, XtedsIndex::iterator> XtedsIndex::findItems(const std::string &name)
{
    return table.equal_range(name);
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
        std::string itemName(attr->value());
        itemNameIndex.insert(itemName, node);

        if (std::strcmp(node->name(), "Application") || std::strcmp(node->name(), "Device"))
        {
            componentNameIndex.insert(itemName, node);
        }
        else if (std::strcmp(node->name(), "Interface"))
        {
            interfaceNameIndex.insert(itemName, node);
        }
        else // Variable || DataMsg || CommandMsg || DataReplyMsg || BitField ...
            ;
    }

    for (XtedsNode *child = node->first_node(); child; child = child->next_sibling())
    {
        index_node(child);
    }
}

} // namespace spa
