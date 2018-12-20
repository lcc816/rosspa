#ifndef XTEDS_H_
#define XTEDS_H_

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <rapidxml/rapidxml.hpp>
#include <spa_core/spa_common.h>

namespace spa
{

typedef rapidxml::xml_node<> XtedsNode;

typedef rapidxml::xml_attribute<> XtedsAttribute;

/// This class represents roots of the xTEDS DOM hierarchy.
class Xteds : public rapidxml::xml_document<>
{
public:
    /// Constructs an xTEDS object by parsing the given zero-terminated string,
    /// and stores it to a directory.
    /// \param file xTEDS data to parse.
    /// \param xteds_repo Directory to store xTEDS.
    Xteds(const std::string &file, const char *uri)
    :   m_uri(uri)
    {
        m_data.assign(file.begin(), file.end());
        m_data.push_back('\0');

        try
        {
            parse<0>(data());
            setName();
            setXuuid(file);
        }
        catch (rapidxml::parse_error &exc)
        {
            throw std::runtime_error(std::string("cannot parse file: ") + exc.what());
        }

        // sotre
        std::ofstream fout(m_uri.c_str());
        if (!fout.is_open())
            throw std::runtime_error(std::string("cannot save file ") + m_uri);

        fout << file;
        fout.close();
    }

    /// Loads xTEDS file into the memeory
    /// and constructs an xTEDS object by parsing the file data.
    /// \param uri File to load.
    /// \exception runtime_error If cannot open file.
    Xteds(const char *uri)
    :   m_uri(uri)
    {
        using namespace std;
        // Open stream
        ifstream stream(uri, ios::binary);
        if (!stream.is_open())
            throw runtime_error(string("cannot open file ") + uri);
        stream.unsetf(ios::skipws); // do not skip white space

        // Determine stream size
        stream.seekg(0, ios::end);
        size_t size = stream.tellg();
        stream.seekg(0);

        // Load data and add terminating 0
        m_data.resize(size + 1);
        stream.read(&m_data.front(), static_cast<streamsize>(size));
        m_data[size] = '\0';
        setXuuid(data());

        try
        {
            parse<0>(data());
            setName();
        }
        catch (rapidxml::parse_error &exc)
        {
            std::cout << "can't parse file " << uri << std::endl;
            std::cout << exc.what() << std::endl;
        }
    }

    /// Gets xTEDS UUID.
    /// \return UUID of  xTEDS.
    uuid_t xuuid() const {return m_xuuid;}

    /// Gets xTEDS name.
    /// \return Name of xTEDS.
    std::string name() const {return xteds_name;}

    /// Get file data.
    /// \return Pointer to data of file.
    char *data() {return &m_data.front();}

    /// Gets file data.
    /// \return Pointer to data of file.
    const char *data() const {return &m_data.front();}

    /// Gets xTEDS URI.
    /// \return URI of xTEDS.
    std::string uri() const {return m_uri;}
private:
    // No copying
    Xteds(const Xteds &);

    /// Sets uuid of xTEDS.
    /// \param data File data to set. Must be zero-terminatied.
    void setXuuid(const std::string &data)
    {
        uuid_create_sha1_from_name(&m_xuuid, NameSpace_DNS, data.c_str(), data.size());
    }

    /// Sets name of xTEDS by parsing its root node.
    bool setName()
    {
        if (XtedsNode *cmpt = first_node()) // root node
        {
            if (XtedsAttribute *attr = cmpt->first_attribute("name"))
            {
                xteds_name = std::string(attr->value());
                return true;
            }
        }
        return false;
    }

    std::vector<char> m_data;
    std::string m_uri;
    uuid_t m_xuuid;
    std::string xteds_name; // xTEDS name
};

}

#endif // XTEDS_H_
