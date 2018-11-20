#include <spa_core/sha1.h>
#include <stdlib.h>
#include <string.h>

#undef BIG_ENDIAN_HOST
typedef unsigned int u32;

/****************
* Rotate a 32 bit integer by n bytes
*/
#if defined(__GNUC__) && defined(__i386__)
static inline u32
    rol( u32 x, int n)
{
    __asm__("roll %%cl,%0"
        :"=r" (x)
        :"0" (x),"c" (n));
    return x;
}
#else
#define rol(x,n) ( ((x) << (n)) | ((x) >> (32-(n))) )
#endif

static void transform(SHA_CTX *hd, unsigned char *data);

void SHA1_Init(SHA_CTX *c)
{
    c->H[0] = 0x67452301;
    c->H[1] = 0xefcdab89;
    c->H[2] = 0x98badcfe;
    c->H[3] = 0x10325476;
    c->H[4] = 0xc3d2e1f0;
    c->nblocks = 0;
    c->count = 0;
}

void SHA1_Update(SHA_CTX *hd, const void *data, unsigned long inlen)
{
    unsigned char *inbuf = (unsigned char *)data;
    if( hd->count == 64 ) { /* flush the buffer */
        transform( hd, hd->buf );
        hd->count = 0;
        hd->nblocks++;
    }
    if( !inbuf )
        return;
    if( hd->count ) {
        for( ; inlen && hd->count < 64; inlen-- )
            hd->buf[hd->count++] = *inbuf++;
        SHA1_Update( hd, NULL, 0 );
        if( !inlen )
            return;
    }

    while( inlen >= 64 ) {
        transform( hd, inbuf );
        hd->count = 0;
        hd->nblocks++;
        inlen -= 64;
        inbuf += 64;
    }
    for( ; inlen && hd->count < 64; inlen-- )
        hd->buf[hd->count++] = *inbuf++;
}

void SHA1_Final(unsigned char *md, SHA_CTX *hd)
{
    u32 t, msb, lsb;

    SHA1_Update(hd, NULL, 0); /* flush */;

    t = hd->nblocks;
    /* multiply by 64 to make a byte count */
    lsb = t << 6;
    msb = t >> 26;
    /* add the count */
    t = lsb;
    if( (lsb += hd->count) < t )
        msb++;
    /* multiply by 8 to make a bit count */
    t = lsb;
    lsb <<= 3;
    msb <<= 3;
    msb |= t >> 29;

    if( hd->count < 56 ) { /* enough room */
        hd->buf[hd->count++] = 0x80; /* pad */
        while( hd->count < 56 )
            hd->buf[hd->count++] = 0;  /* pad */
    }
    else { /* need one extra block */
        hd->buf[hd->count++] = 0x80; /* pad character */
        while( hd->count < 64 )
            hd->buf[hd->count++] = 0;
        SHA1_Update(hd, NULL, 0);  /* flush */;
        memset(hd->buf, 0, 56 ); /* fill next block with zeroes */
    }
    /* append the 64 bit count */
    hd->buf[56] = msb >> 24;
    hd->buf[57] = msb >> 16;
    hd->buf[58] = msb >>  8;
    hd->buf[59] = msb       ;
    hd->buf[60] = lsb >> 24;
    hd->buf[61] = lsb >> 16;
    hd->buf[62] = lsb >>  8;
    hd->buf[63] = lsb       ;
    transform( hd, hd->buf );

#ifdef BIG_ENDIAN_HOST
    memcpy( md, hd->H, 20 );
#else
    {
        int i;
        unsigned char *p2;
        for(i=0, p2=(unsigned char*)hd->H; i < 5; i++, p2 += 4)
        {
            *md++ = p2[3];
            *md++ = p2[2];
            *md++ = p2[1];
            *md++ = p2[0];
        }
    }
#endif
}

/****************
* Transform the message X which consists of 16 32-bit-words
*/
void transform(SHA_CTX *hd, unsigned char *data)
{
    u32 a,b,c,d,e,tm;
    u32 x[16];

    /* get values from the chaining vars */
    a = hd->H[0];
    b = hd->H[1];
    c = hd->H[2];
    d = hd->H[3];
    e = hd->H[4];

#ifdef BIG_ENDIAN_HOST
    memcpy( x, data, 64 );
#else
    {
        int i;
        unsigned char *p2;
        for(i=0, p2=(unsigned char*)x; i < 16; i++, p2 += 4)
        {
            p2[3] = *data++;
            p2[2] = *data++;
            p2[1] = *data++;
            p2[0] = *data++;
        }
    }
#endif


#define K1  0x5A827999L
#define K2  0x6ED9EBA1L
#define K3  0x8F1BBCDCL
#define K4  0xCA62C1D6L
#define F1(x,y,z)   ( z ^ ( x & ( y ^ z ) ) )
#define F2(x,y,z)   ( x ^ y ^ z )
#define F3(x,y,z)   ( ( x & y ) | ( z & ( x | y ) ) )
#define F4(x,y,z)   ( x ^ y ^ z )


#define M(i) ( tm =   x[i&0x0f] ^ x[(i-14)&0x0f] \
    ^ x[(i-8)&0x0f] ^ x[(i-3)&0x0f] \
    , (x[i&0x0f] = rol(tm,1)) )

#define R(a,b,c,d,e,f,k,m)  do { e += rol( a, 5 )     \
    + f( b, c, d )  \
    + k       \
    + m;          \
    b = rol( b, 30 );    \
    } while(0)
    R( a, b, c, d, e, F1, K1, x[ 0] );
    R( e, a, b, c, d, F1, K1, x[ 1] );
    R( d, e, a, b, c, F1, K1, x[ 2] );
    R( c, d, e, a, b, F1, K1, x[ 3] );
    R( b, c, d, e, a, F1, K1, x[ 4] );
    R( a, b, c, d, e, F1, K1, x[ 5] );
    R( e, a, b, c, d, F1, K1, x[ 6] );
    R( d, e, a, b, c, F1, K1, x[ 7] );
    R( c, d, e, a, b, F1, K1, x[ 8] );
    R( b, c, d, e, a, F1, K1, x[ 9] );
    R( a, b, c, d, e, F1, K1, x[10] );
    R( e, a, b, c, d, F1, K1, x[11] );
    R( d, e, a, b, c, F1, K1, x[12] );
    R( c, d, e, a, b, F1, K1, x[13] );
    R( b, c, d, e, a, F1, K1, x[14] );
    R( a, b, c, d, e, F1, K1, x[15] );
    R( e, a, b, c, d, F1, K1, M(16) );
    R( d, e, a, b, c, F1, K1, M(17) );
    R( c, d, e, a, b, F1, K1, M(18) );
    R( b, c, d, e, a, F1, K1, M(19) );
    R( a, b, c, d, e, F2, K2, M(20) );
    R( e, a, b, c, d, F2, K2, M(21) );
    R( d, e, a, b, c, F2, K2, M(22) );
    R( c, d, e, a, b, F2, K2, M(23) );
    R( b, c, d, e, a, F2, K2, M(24) );
    R( a, b, c, d, e, F2, K2, M(25) );
    R( e, a, b, c, d, F2, K2, M(26) );
    R( d, e, a, b, c, F2, K2, M(27) );
    R( c, d, e, a, b, F2, K2, M(28) );
    R( b, c, d, e, a, F2, K2, M(29) );
    R( a, b, c, d, e, F2, K2, M(30) );
    R( e, a, b, c, d, F2, K2, M(31) );
    R( d, e, a, b, c, F2, K2, M(32) );
    R( c, d, e, a, b, F2, K2, M(33) );
    R( b, c, d, e, a, F2, K2, M(34) );
    R( a, b, c, d, e, F2, K2, M(35) );
    R( e, a, b, c, d, F2, K2, M(36) );
    R( d, e, a, b, c, F2, K2, M(37) );
    R( c, d, e, a, b, F2, K2, M(38) );
    R( b, c, d, e, a, F2, K2, M(39) );
    R( a, b, c, d, e, F3, K3, M(40) );
    R( e, a, b, c, d, F3, K3, M(41) );
    R( d, e, a, b, c, F3, K3, M(42) );
    R( c, d, e, a, b, F3, K3, M(43) );
    R( b, c, d, e, a, F3, K3, M(44) );
    R( a, b, c, d, e, F3, K3, M(45) );
    R( e, a, b, c, d, F3, K3, M(46) );
    R( d, e, a, b, c, F3, K3, M(47) );
    R( c, d, e, a, b, F3, K3, M(48) );
    R( b, c, d, e, a, F3, K3, M(49) );
    R( a, b, c, d, e, F3, K3, M(50) );
    R( e, a, b, c, d, F3, K3, M(51) );
    R( d, e, a, b, c, F3, K3, M(52) );
    R( c, d, e, a, b, F3, K3, M(53) );
    R( b, c, d, e, a, F3, K3, M(54) );
    R( a, b, c, d, e, F3, K3, M(55) );
    R( e, a, b, c, d, F3, K3, M(56) );
    R( d, e, a, b, c, F3, K3, M(57) );
    R( c, d, e, a, b, F3, K3, M(58) );
    R( b, c, d, e, a, F3, K3, M(59) );
    R( a, b, c, d, e, F4, K4, M(60) );
    R( e, a, b, c, d, F4, K4, M(61) );
    R( d, e, a, b, c, F4, K4, M(62) );
    R( c, d, e, a, b, F4, K4, M(63) );
    R( b, c, d, e, a, F4, K4, M(64) );
    R( a, b, c, d, e, F4, K4, M(65) );
    R( e, a, b, c, d, F4, K4, M(66) );
    R( d, e, a, b, c, F4, K4, M(67) );
    R( c, d, e, a, b, F4, K4, M(68) );
    R( b, c, d, e, a, F4, K4, M(69) );
    R( a, b, c, d, e, F4, K4, M(70) );
    R( e, a, b, c, d, F4, K4, M(71) );
    R( d, e, a, b, c, F4, K4, M(72) );
    R( c, d, e, a, b, F4, K4, M(73) );
    R( b, c, d, e, a, F4, K4, M(74) );
    R( a, b, c, d, e, F4, K4, M(75) );
    R( e, a, b, c, d, F4, K4, M(76) );
    R( d, e, a, b, c, F4, K4, M(77) );
    R( c, d, e, a, b, F4, K4, M(78) );
    R( b, c, d, e, a, F4, K4, M(79) );

    /* Update chaining vars */
    hd->H[0] += a;
    hd->H[1] += b;
    hd->H[2] += c;
    hd->H[3] += d;
    hd->H[4] += e;
}
