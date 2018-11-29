#ifndef SPA_UUID_H_
#define SPA_UUID_H_

#include <stdint.h>
#include <string>
#include <vector>

namespace spa
{

struct uuid_t
{
    uint32_t time_low;
    uint16_t time_mid;
    uint16_t time_hi_and_version;
    uint8_t  clock_seq_hi_and_reserved;
    uint8_t  clock_seq_low;
    uint8_t  node[6];
    std::string toString() const;
    std::vector<uint8_t> serialize() const;
    void deserialize(const std::vector<uint8_t> &data);
};

/* uuid_create_sha1_from_name -- create a version 5 (SHA-1) UUID
   using a "name" from a "name space" */
void uuid_create_sha1_from_name(
    uuid_t *uuid,         /* resulting UUID */
    const uuid_t nsid,          /* UUID of the namespace */
    const char *name,           /* the name from which to generate a UUID */
    int namelen           /* the length of the name */
);

/* uuid_compare --  Compare two UUID's "lexically" and return
        -1   u1 is lexically before u2
         0   u1 is equal to u2
         1   u1 is lexically after u2
   Note that lexical ordering is not temporal ordering!
*/
int uuid_compare(uuid_t *u1, uuid_t *u2);

// Some pre-defined values of name space ID
extern uuid_t NameSpace_DNS;
extern uuid_t NameSpace_OID;
extern uuid_t NameSpace_URL;
extern uuid_t NameSpace_X500;

} // namespace spa

#endif // SPA_UUID_H_
