/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @file:      md5.c
* @Engineer:  TenYan
* @Company:  徐工信息智能硬件部
* @Date:      2020-07-13
* @brief:     MD5 (Message-Digest Algorithm)
* @Description: 
* The MD5 algorithm takes as input a message of arbitrary length and produces
* as output a 128-bit message digest of the input. Refer to RFC 1321
*****************************************************************************/

//Dependencies
#include "md5.h"
#include <string.h>

#define MD5_SUPPORT  ENABLED

//Check crypto library configuration
#if (MD5_SUPPORT == ENABLED)
/******************************************************************************
 * Macros
 ******************************************************************************/
#define cryptoMemcpy(dest, src, length) (void)memcpy(dest, src, length)

// MD5 auxiliary functions
#define F(x, y, z) (((x) & (y)) | (~(x) & (z)))
#define G(x, y, z) (((x) & (z)) | ((y) & ~(z)))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | ~(z)))

#define FF(a, b, c, d, x, s, k) a += F(b, c, d) + (x) + (k), a = ROL32(a, s) + (b)
#define GG(a, b, c, d, x, s, k) a += G(b, c, d) + (x) + (k), a = ROL32(a, s) + (b)
#define HH(a, b, c, d, x, s, k) a += H(b, c, d) + (x) + (k), a = ROL32(a, s) + (b)
#define II(a, b, c, d, x, s, k) a += I(b, c, d) + (x) + (k), a = ROL32(a, s) + (b)

/******************************************************************************
 *   Data Types
 ******************************************************************************/
// MD5 padding
static const uint8_t padding[64] =
{
   0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// MD5 constants
static const uint32_t k[64] =
{
   0xD76AA478, 0xE8C7B756, 0x242070DB, 0xC1BDCEEE, 0xF57C0FAF, 0x4787C62A, 0xA8304613, 0xFD469501,
   0x698098D8, 0x8B44F7AF, 0xFFFF5BB1, 0x895CD7BE, 0x6B901122, 0xFD987193, 0xA679438E, 0x49B40821,
   0xF61E2562, 0xC040B340, 0x265E5A51, 0xE9B6C7AA, 0xD62F105D, 0x02441453, 0xD8A1E681, 0xE7D3FBC8,
   0x21E1CDE6, 0xC33707D6, 0xF4D50D87, 0x455A14ED, 0xA9E3E905, 0xFCEFA3F8, 0x676F02D9, 0x8D2A4C8A,
   0xFFFA3942, 0x8771F681, 0x6D9D6122, 0xFDE5380C, 0xA4BEEA44, 0x4BDECFA9, 0xF6BB4B60, 0xBEBFBC70,
   0x289B7EC6, 0xEAA127FA, 0xD4EF3085, 0x04881D05, 0xD9D4D039, 0xE6DB99E5, 0x1FA27CF8, 0xC4AC5665,
   0xF4292244, 0x432AFF97, 0xAB9423A7, 0xFC93A039, 0x655B59C3, 0x8F0CCC92, 0xFFEFF47D, 0x85845DD1,
   0x6FA87E4F, 0xFE2CE6E0, 0xA3014314, 0x4E0811A1, 0xF7537E82, 0xBD3AF235, 0x2AD7D2BB, 0xEB86D391
};

/*************************************************************************
 * @brief Process message in 16-word blocks
 * @param[in] context Pointer to the MD5 context
 *************************************************************************/
void md5_ProcessBlock(md5_context_t *pThis)
{
   uint8_t i;

   // Initialize the 4 working registers
   uint32_t a = pThis->h[0];
   uint32_t b = pThis->h[1];
   uint32_t c = pThis->h[2];
   uint32_t d = pThis->h[3];

   // Process message in 16-word blocks
   uint32_t *x = pThis->x;

   // Convert from little-endian byte order to host byte order
   for(i = 0; i < 16; i++)
   {
      x[i] = letoh32(x[i]);
   }

   // Round 1
   FF(a, b, c, d, x[0],  7,  k[0]);
   FF(d, a, b, c, x[1],  12, k[1]);
   FF(c, d, a, b, x[2],  17, k[2]);
   FF(b, c, d, a, x[3],  22, k[3]);
   FF(a, b, c, d, x[4],  7,  k[4]);
   FF(d, a, b, c, x[5],  12, k[5]);
   FF(c, d, a, b, x[6],  17, k[6]);
   FF(b, c, d, a, x[7],  22, k[7]);
   FF(a, b, c, d, x[8],  7,  k[8]);
   FF(d, a, b, c, x[9],  12, k[9]);
   FF(c, d, a, b, x[10], 17, k[10]);
   FF(b, c, d, a, x[11], 22, k[11]);
   FF(a, b, c, d, x[12], 7,  k[12]);
   FF(d, a, b, c, x[13], 12, k[13]);
   FF(c, d, a, b, x[14], 17, k[14]);
   FF(b, c, d, a, x[15], 22, k[15]);

   // Round 2
   GG(a, b, c, d, x[1],  5,  k[16]);
   GG(d, a, b, c, x[6],  9,  k[17]);
   GG(c, d, a, b, x[11], 14, k[18]);
   GG(b, c, d, a, x[0],  20, k[19]);
   GG(a, b, c, d, x[5],  5,  k[20]);
   GG(d, a, b, c, x[10], 9,  k[21]);
   GG(c, d, a, b, x[15], 14, k[22]);
   GG(b, c, d, a, x[4],  20, k[23]);
   GG(a, b, c, d, x[9],  5,  k[24]);
   GG(d, a, b, c, x[14], 9,  k[25]);
   GG(c, d, a, b, x[3],  14, k[26]);
   GG(b, c, d, a, x[8],  20, k[27]);
   GG(a, b, c, d, x[13], 5,  k[28]);
   GG(d, a, b, c, x[2],  9,  k[29]);
   GG(c, d, a, b, x[7],  14, k[30]);
   GG(b, c, d, a, x[12], 20, k[31]);

   // Round 3
   HH(a, b, c, d, x[5],  4,  k[32]);
   HH(d, a, b, c, x[8],  11, k[33]);
   HH(c, d, a, b, x[11], 16, k[34]);
   HH(b, c, d, a, x[14], 23, k[35]);
   HH(a, b, c, d, x[1],  4,  k[36]);
   HH(d, a, b, c, x[4],  11, k[37]);
   HH(c, d, a, b, x[7],  16, k[38]);
   HH(b, c, d, a, x[10], 23, k[39]);
   HH(a, b, c, d, x[13], 4,  k[40]);
   HH(d, a, b, c, x[0],  11, k[41]);
   HH(c, d, a, b, x[3],  16, k[42]);
   HH(b, c, d, a, x[6],  23, k[43]);
   HH(a, b, c, d, x[9],  4,  k[44]);
   HH(d, a, b, c, x[12], 11, k[45]);
   HH(c, d, a, b, x[15], 16, k[46]);
   HH(b, c, d, a, x[2],  23, k[47]);

   // Round 4
   II(a, b, c, d, x[0],  6,  k[48]);
   II(d, a, b, c, x[7],  10, k[49]);
   II(c, d, a, b, x[14], 15, k[50]);
   II(b, c, d, a, x[5],  21, k[51]);
   II(a, b, c, d, x[12], 6,  k[52]);
   II(d, a, b, c, x[3],  10, k[53]);
   II(c, d, a, b, x[10], 15, k[54]);
   II(b, c, d, a, x[1],  21, k[55]);
   II(a, b, c, d, x[8],  6,  k[56]);
   II(d, a, b, c, x[15], 10, k[57]);
   II(c, d, a, b, x[6],  15, k[58]);
   II(b, c, d, a, x[13], 21, k[59]);
   II(a, b, c, d, x[4],  6,  k[60]);
   II(d, a, b, c, x[11], 10, k[61]);
   II(c, d, a, b, x[2],  15, k[62]);
   II(b, c, d, a, x[9],  21, k[63]);

   // Update the hash value
   pThis->h[0] += a;
   pThis->h[1] += b;
   pThis->h[2] += c;
   pThis->h[3] += d;
}

/*************************************************************************
 * @brief Initialize MD5 message digest context
 * @param[in] context Pointer to the MD5 context to initialize
 *************************************************************************/
void md5_Init(md5_context_t *pThis)
{
   // Set initial hash value
   pThis->h[0] = 0x67452301;
   pThis->h[1] = 0xEFCDAB89;
   pThis->h[2] = 0x98BADCFE;
   pThis->h[3] = 0x10325476;

   // Number of bytes in the buffer
   pThis->size = 0;
   // Total length of the message
   pThis->totalSize = 0;
}

/*************************************************************************
 * @brief Update the MD5 context with a portion of the message being hashed
 * @param[in] context Pointer to the MD5 context
 * @param[in] data Pointer to the buffer being hashed
 * @param[in] length Length of the buffer
 *************************************************************************/
void md5_Update(md5_context_t *pThis, const void *data, size_t length)
{
   size_t n;

   // Process the incoming data
   while(length > 0)
   {
      // The buffer can hold at most 64 bytes
      n = MIN(length, 64 - pThis->size);

      // Copy the data to the buffer
      cryptoMemcpy(pThis->buffer + pThis->size, data, n);

      // Update the MD5 context
      pThis->size += n;
      pThis->totalSize += n;
      // Advance the data pointer
      data = (uint8_t *) data + n;
      // Remaining bytes to process
      length -= n;

      //Process message in 16-word blocks
      if(pThis->size == 64)
      {
         md5_ProcessBlock(pThis); // Transform the 16-word block
         pThis->size = 0;  // Empty the buffer
      }
   }
}

/*************************************************************************
 * @brief Finish the MD5 message digest
 * @param[in] context Pointer to the MD5 context
 * @param[out] digest Calculated digest (optional parameter)
 *************************************************************************/
void md5_Final(md5_context_t *pThis, uint8_t *digest)
{
   uint8_t i;
   size_t paddingSize;
   uint64_t totalSize;

   // Length of the original message (before padding)
   totalSize = pThis->totalSize * 8;

   // Pad the message so that its length is congruent to 56 modulo 64
   if(pThis->size < 56)
      paddingSize = 56 - pThis->size;
   else
      paddingSize = 64 + 56 - pThis->size;

   // Append padding
   md5_Update(pThis, padding, paddingSize);

   // Append the length of the original message
   pThis->x[14] = htole32((uint32_t) totalSize);
   pThis->x[15] = htole32((uint32_t) (totalSize >> 32));

   // Calculate the message digest
   md5_ProcessBlock(pThis);

   // Convert from host byte order to little-endian byte order
   for(i = 0; i < 4; i++)
   {
      pThis->h[i] = htole32(pThis->h[i]);
   }

   // Copy the resulting digest
   if(digest != NULL)
      cryptoMemcpy(digest, pThis->digest, MD5_DIGEST_SIZE);
}

/*************************************************************************
 * @brief Finish the MD5 message digest (no padding is added)
 * @param[in] context Pointer to the MD5 context
 * @param[out] digest Calculated digest
 *************************************************************************/
void md5_FinalRaw(md5_context_t *pThis, uint8_t *digest)
{
   uint8_t i;

   // Convert from host byte order to little-endian byte order
   for(i = 0; i < 4; i++)
   {
      pThis->h[i] = htole32(pThis->h[i]);
   }

   // Copy the resulting digest
   cryptoMemcpy(digest, pThis->digest, MD5_DIGEST_SIZE);

   // Convert from little-endian byte order to host byte order
   for(i = 0; i < 4; i++)
   {
      pThis->h[i] = letoh32(pThis->h[i]);
   }
}

/*************************************************************************
 * @brief Digest a message using MD5
 * @param[in] data Pointer to the message being hashed
 * @param[in] length Length of the message
 * @param[out] digest Pointer to the calculated digest
 * @return Error code
 *************************************************************************/
void md5_Compute(const void *data, size_t length, uint8_t *digest)
{
	md5_context_t context;
	
  md5_Init(&context); // Initialize the MD5 context
  md5_Update(&context, data, length); // Digest the message
  md5_Final(&context, digest); // Finalize the MD5 message digest
}

#if 0
void md5_demo(void)
{
	uint8_t result[16];
	const char *test_str = "gchinaran@gmail.com";
	
	memset(result,0x00,sizeof(result));
	md5_Compute(test_str,strlen(test_str),result);
	// md5 value: 84701974fb98315895e3ed9053a0b389
	return;
}
#endif
#endif
