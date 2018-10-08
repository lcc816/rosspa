#include "spa/xteds_repository.h"
#include <cstring>
#include <iostream>
#include <fstream>

namespace spa
{

/*----------- Implementation of XtedsIndex -------------*/

bool XtedsIndex::insert(const std::string &name, const XtedsNode &node)
{
    ItemType item(name, node);
    iterator ret = table.insert(item);
    if (ret == table.end())
        return false;

    return true;
}

void XtedsIndex::erase(const std::string &name, const XtedsNode &node)
{
    std::pair<iterator, iterator> range = findAll(name);
    if (range.first == table.end())
        return;

    for (iterator it = range.first; it != range.second; )
        if (it->second == node)
            it = table.erase(it);
        else
            ++it;
}

std::pair<XtedsIndex::iterator, XtedsIndex::iterator> XtedsIndex::findAll(const std::string &name)
{

    std::pair<iterator, iterator> range = table.equal_range(name);
    return range;
}

/*--------- Implementation of ComponentNameIndex ---------*/

void ComponentNameIndex::index(const XtedsDocPtr &doc)
{
    XtedsNode temp = doc->first_node("xTEDS");
    XtedsNode node = temp->first_node("Application");

    if (node)
        ;
    else
        node = temp->first_node("Device");

    char *name = node->first_attribute("name")->value();

    insert(std::string(name), node);
}

/*--------- Implementation of InterfaceNameIndex ---------*/

void InterfaceNameIndex::index(const XtedsDocPtr &doc)
{
    XtedsNode node = doc->first_node("xTEDS")->first_node();

    for ( ; node; node = node->next_sibling())
    {
        if (std::strcmp(node->name(), "Interface"))
        {
             char *name = node->first_attribute("name")->value();
             insert(std::string(name), node);
        }
    }
}

/*---------- Implementation of ItemNameIndex ------------*/

void ItemNameIndex::index(const XtedsDocPtr &doc)
{

}

/*---------- Implementation of XtedsRepository ----------*/

void XtedsRepository::saveAsXteds(const std::string &content, const char *name)
{
    saveAsXteds(content.c_str(), name);
}

void XtedsRepository::saveAsXteds(const char *content, const char *name)
{
    std::string url;
    url = "../xTEDS/" + name + ".xml";
    std::ofstream fout(url.c_str());
    for (const char *it = content; *it != 0; it++)
        fout << *it;
    std::cout << "registered xTEDS " << url << std::endl;

    fout.close();
}

void XtedsRepository::store(const XtedsDocPtr &doc)
{
    // Store the object of xTEDS.
    xtedsList.push_back(doc);
    // Index every element and attribute.
    componentNameIndex.index(doc);
    interfaceNameIndex.index(doc);
    itemNameIndex.index(doc);
}

