#ifndef XTEDS_REPOSITORY_H
#define XTEDS_REPOSITORY_H

#include <unordered_map> // Use it to implement the hash table of the xTEDS's node.
#include <list>
#include <memory>
#include "rapidxml.hpp"

namespace spa
{

/// Use shared pointer to xml_document as its handle to store.
typedef std::shared_ptr<rapidxml::xml_document<> > XtedsDocPtr;

/// The list type used to store list of xTEDS files.
typedef std::list<XtedsDocPtr> XtedsList;

class XtedsIndex
{
public:
    XtedsIndex() {}
    ~XtedsIndex() {}

    /// Node type of xTEDS.
    typedef rapidxml::xml_node<>* XtedsNode;
    /// Container type of table to sotre the nodes.
    typedef std::unordered_multimap<std::string, XtedsNode> XtedsTable;
    /// Item type.
    typedef std::pair<std::string, XtedsNode> ItemType;
    /// Iterator to item.
    typedef XtedsTable::iterator iterator;

    /// Insert a node into the table.
    /// \param node Reference of item to be indexed.
    /// \param name Name of item.
    /// \return True if inserted successfully.
    bool insert(const std::string &name, const XtedsNode &node);
    /// Insert a node into the table.
    /// \param node Reference of item to be deleted.
    /// \param name Name of item.
    void erase(const std::string &name, const XtedsNode &node);

    /// Find all items with given name.
    /// \param name Name to look for.
    /// \return std::pair containing a pair of iterators defining the range.
    std::pair<iterator, iterator> findAll(const std::string &name);

private:
    XtedsTable table;
};

class ComponentNameIndex : public XtedsIndex
{
public:
    ComponentNameIndex() {}
    ~ComponentNameIndex() {}

    /// Index the component name into the hash table.
    /// \param doc Reference of pointer to the xTEDS object.
    void index(const XtedsDocPtr &doc);
};

class InterfaceNameIndex : public XtedsIndex
{
public:
    InterfaceNameIndex() {}
    ~InterfaceNameIndex() {}

    /// Index the Interface name into the hash table.
    /// \param doc Reference of pointer to the xTEDS object.
    void index(const XtedsDocPtr &doc);
};

class ItemNameIndex : public XtedsIndex
{
public:
    /// Index all item names into the hash table.
    /// \param doc Reference of pointer to the xTEDS object.
    void index(const XtedsDocPtr &doc);
};

class XtedsRepository
{
public:
    XtedsRepository() {}
    virtual ~XtedsRepository() {}

    /// Save the xTEDS file passed by a string using the given file name.
    /// \param content File content to save.
    /// \param url Given file name.
    void saveAsXteds(const std::string &content, const char *url);

    /// Save the xTEDS file passed by a string using the given file name.
    /// \param content File content to save.
    /// \param url Given file name.
    void saveAsXteds(const char *content, const char *url);

    /// Index a xTEDS object.
    /// \param doc Reference of pointer to xTEDS object.
    void store(const XtedsDocPtr &doc);

private:
    XtedsList xtedsList;
    ComponentNameIndex componentNameIndex;
    InterfaceNameIndex interfaceNameIndex;
    ItemNameIndex itemNameIndex;
};

}

#endif // XTEDS_REPOSITORY_H
