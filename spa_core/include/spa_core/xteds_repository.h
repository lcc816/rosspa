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
    typedef std::unordered_multimap<std::string, XtedsNode *> XtedsTable;
    /// Item type.
    //typedef XtedsTable::value_type ItemType;
    /// Iterator to item.
    typedef XtedsTable::iterator iterator;

    /// Insert a node into the table.
    /// \param node Reference of item to be indexed.
    /// \param name Name of item.
    void insert(const std::string &name, XtedsNode * const node);

    /// Insert a node into the table.
    /// \param node Reference of item to be deleted.
    /// \param name Name of item.
    void erase(const std::string &name, XtedsNode * const node);

    /// Find all items with given name.
    /// \param name Name to look for.
    /// \return containing a pair of iterators defining the range.
    std::pair<iterator, iterator> findItems(const std::string &name);

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

private:
    XtedsList xtedsList;
    XtedsIndex componentNameIndex;
    XtedsIndex interfaceNameIndex;
    XtedsIndex itemNameIndex;
};

}

#endif // XTEDS_REPOSITORY_H
