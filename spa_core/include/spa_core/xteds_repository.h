#ifndef XTEDS_REPOSITORY_H
#define XTEDS_REPOSITORY_H

#include <unordered_map> // Use it to implement the hash table of the xTEDS's node.
#include <list>
#include <string>
#include <memory>
#include <spa_core/xteds.h>

namespace spa
{

class XtedsIndex
{
public:
    XtedsIndex() {}
    ~XtedsIndex() {}

    /// Container type of table to sotre the nodes.
    typedef std::unordered_map<XtedsNode *, std::string> XtedsTable;
    /// Item type.
    //typedef XtedsTable::value_type ItemType;
    /// Iterator to item.
    typedef XtedsTable::iterator iterator;

    /// Insert a node into the table.
    /// \param node Reference of item to be indexed.
    /// \param name Name of item.
    /// \return Return true if the insert succeeds.
    bool insert(XtedsNode * const node, const std::string &name);

    /// Insert a node into the table.
    /// \param node Reference of item to be deleted.
    /// \param name Name of item.
    /// \return Return true if the erase succeeds
    bool erase(XtedsNode * const node);

    /// Get name of an item.
    /// \param name Pointer to item to search for.
    /// \return Name of item search for.
    std::string getName(XtedsNode * const node);

private:
    XtedsTable table;
};

/// The type of pointer to xTEDS object
typedef std::shared_ptr<Xteds> XtedsPtr;

/// The list type used to store list of xTEDS files.
typedef std::list<XtedsPtr> XtedsList;

class XtedsRepository
{
public:
    XtedsRepository() {}
    virtual ~XtedsRepository() {}

    /// Index a xTEDS object.
    /// \param doc Reference of pointer to xTEDS object.
    void index(const XtedsPtr &doc);

    /// Index a xTEDS node
    /// \param node Pointer to the node to be indexed.
    void index_node(XtedsNode * const node);

    XtedsList xtedsList;
    XtedsIndex componentNameIndex;
    XtedsIndex interfaceNameIndex;
    XtedsIndex itemNameIndex;
};

}

#endif // XTEDS_REPOSITORY_H
