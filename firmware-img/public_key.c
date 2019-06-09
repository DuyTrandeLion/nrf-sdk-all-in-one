
/* This file was automatically generated by nrfutil on 2019-01-31 (YY-MM-DD) at 22:49:31 */

#include "sdk_config.h"
#include "stdint.h"
#include "compiler_abstraction.h"

#if NRF_CRYPTO_BACKEND_OBERON_ENABLED
/* Oberon backend is changing endianness thus public key must be kept in RAM. */
#define _PK_CONST
#else
#define _PK_CONST const
#endif


/** @brief Public key used to verify DFU images */
__ALIGN(4) _PK_CONST uint8_t pk[64] =
{
    0xe9, 0xc9, 0x58, 0xd3, 0x15, 0xf9, 0x27, 0xb3, 0xb4, 0x9c, 0x6f, 0xa8, 0x10, 0xbc, 0xdf, 0x91, 0x76, 0x94, 0xf7, 0x2a, 0x84, 0x9e, 0xd3, 0x9e, 0x8c, 0x62, 0xaa, 0xb8, 0xbf, 0x4b, 0xe9, 0x19, 
    0x02, 0x32, 0xcf, 0x9a, 0xc9, 0xa6, 0x14, 0xf8, 0x33, 0x0e, 0x94, 0xc6, 0xe2, 0x16, 0x62, 0x93, 0x8d, 0x03, 0xe4, 0x05, 0xec, 0x3e, 0xd0, 0x7b, 0x33, 0x69, 0x6e, 0x49, 0x7a, 0xcd, 0x66, 0x87
};