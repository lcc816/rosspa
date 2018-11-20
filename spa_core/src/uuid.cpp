#include <cstring>
#include <iostream>
#include <arpa/inet.h> // for sequence of octets
// #include<Winsock2.h>
#include <spa_core/uuid.h>
extern "C"{
#include <openssl/sha.h> // for SHA1 hash
}

namespace spa
{

//static int read_state(uint16_t *clockseq, uuid_time_t *timestamp, uuid_node_t *node);

static void format_uuid_v3or5(uuid_t *uuid, unsigned char hash[16], int v);

// Some pre-defined values of name space ID

/* Name string is a fully-qualified domain name */
uuid_t NameSpace_DNS = { /* 6ba7b810-9dad-11d1-80b4-00c04fd430c8 */
   0x6ba7b810,
   0x9dad,
   0x11d1,
   0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

/* Name string is a URL */
uuid_t NameSpace_URL = { /* 6ba7b811-9dad-11d1-80b4-00c04fd430c8 */
   0x6ba7b811,
   0x9dad,
   0x11d1,
   0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

/* Name string is an ISO OID */
uuid_t NameSpace_OID = { /* 6ba7b812-9dad-11d1-80b4-00c04fd430c8 */
   0x6ba7b812,
   0x9dad,
   0x11d1,
   0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

/* Name string is an X.500 DN (in DER or a text output format) */
uuid_t NameSpace_X500 = { /* 6ba7b814-9dad-11d1-80b4-00c04fd430c8 */
   0x6ba7b814,
   0x9dad,
   0x11d1,
   0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

void uuid_create_sha1_from_name(uuid_t *uuid, const uuid_t nsid, const char *name, int namelen)
{
    SHA_CTX c;
    unsigned char hash[20];
    uuid_t net_nsid;

    /* put name space ID in network byte order so it hashes the same
       no matter what endian machine we're on */
    net_nsid = nsid;
    net_nsid.time_low = htonl(net_nsid.time_low);
    net_nsid.time_mid = htons(net_nsid.time_mid);
    net_nsid.time_hi_and_version = htons(net_nsid.time_hi_and_version);

    SHA1_Init(&c);
    SHA1_Update(&c, &net_nsid, sizeof net_nsid);
    SHA1_Update(&c, name, namelen);
    SHA1_Final(hash, &c);

    /* the hash is in network byte order at this point */
    format_uuid_v3or5(uuid, hash, 5);
}

/* format_uuid_v3or5 -- make a UUID from a (pseudo)random 128-bit
   number */
void format_uuid_v3or5(uuid_t *uuid, unsigned char hash[], int v)
{
    /* convert UUID to local byte order */
    memcpy(uuid, hash, sizeof *uuid);
    uuid->time_low = ntohl(uuid->time_low);
    uuid->time_mid = ntohs(uuid->time_mid);
    uuid->time_hi_and_version = ntohs(uuid->time_hi_and_version);

    /* put in the variant and version bits */
    uuid->time_hi_and_version &= 0x0FFF;
    uuid->time_hi_and_version |= (v << 12);
    uuid->clock_seq_hi_and_reserved &= 0x3F;
    uuid->clock_seq_hi_and_reserved |= 0x80;
}

} // namespace spa

