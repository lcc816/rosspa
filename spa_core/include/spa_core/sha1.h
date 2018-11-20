#ifndef SHA1_H_
#define SHA1_H_

#ifdef __cplusplus
extern "C" {
#endif

/* length of sha1 digest */
#define SHA_DIGEST_LENGTH 20

typedef struct
{
    unsigned H[5];
    unsigned nblocks;
    unsigned char buf[64];
    int count;
} SHA_CTX;

void SHA1_Init(SHA_CTX *c);

void SHA1_Update(SHA_CTX *c, const void *data, unsigned long len);

void SHA1_Final(unsigned char *md, SHA_CTX *c);

#ifdef __cplusplus
}
#endif

#endif /* SHA1_H_ */
