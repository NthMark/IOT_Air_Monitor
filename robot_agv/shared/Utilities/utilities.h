/*******************************************************************************
 *
 * Copyright (c) 2025
 * Lumi, JSC.
 * All Rights Reserved
 *
 *
 * Description:
 *
 * Author: HoangNH
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 2.0.0 $
 * Last Changed:     $Date: 10/10/2025 $
 *
 ******************************************************************************/
#ifndef _UTILITIES_H_
#define _UTILITIES_H_
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define NOTFOUND                0xFFFF

#ifndef NULL
#define NULL                    0
#endif // NULL

#define st(x)                   do { x } while(__LINE__ == -1)

#ifndef BV
#define BV(n)                   (1 << (n))
#endif /* BV */

#define absSub(a, b)  			((a) > (b) ? ((a)-(b)) : ((b)-(a)))

#define cat2(i,j)       		i##j
#define cat3(i,j,k)     		i##j##k

#ifndef min
#define min(a,b)				((a) < (b) ? (a) : (b))
#endif
#ifndef min2
#define min2(a,b)				((a) < (b) ? (a) : (b))
#endif
#ifndef min3
#define min3(a,b,c)				min2(min2(a, b), c)
#endif
#ifndef max2
#define max2(a,b)				((a) > (b) ? (a): (b))
#endif
#ifndef max3
#define max3(a,b,c)				max2(max2(a, b), c)
#endif

#define LENGTHOF(s, m)          (sizeof(((s*)0)->m))
#define OFFSETOF(s, m)          ((unsigned int) &((s *)0)->m)
#define ROUND_INT(x, r)         (((x) + (r) - 1) / (r) * (r))
#define ROUND_TO_POW2(x, r)     (((x) + (r) - 1) & ~((r) - 1))

//  direct memory access
#define U8_GET(addr)			(*(volatile unsigned char  *)(addr))
#define U16_GET(addr)			(*(volatile unsigned short *)(addr))
#define U32_GET(addr)			(*(volatile unsigned int  *)(addr))

#define U8_SET(addr, v)			(*(volatile unsigned char  *)(addr) = (unsigned char)(v))
#define U16_SET(addr, v)		(*(volatile unsigned short *)(addr) = (unsigned short)(v))
#define U32_SET(addr, v)		(*(volatile unsigned int  *)(addr) = (v))

#define U8_INC(addr)			U8_GET(addr) += 1
#define U16_INC(addr)			U16_GET(addr) += 1
#define U32_INC(addr)			U32_GET(addr) += 1

#define U8_DEC(addr)			U8_GET(addr) -= 1
#define U16_DEC(addr)			U16_GET(addr) -= 1
#define U32_DEC(addr)			U32_GET(addr) -= 1

#define U8_CPY(addr1,addr2)		U8_SET(addr1, U8_GET(addr2))
#define U16_CPY(addr1,addr2)	U16_SET(addr1, U16_GET(addr2))
#define U32_CPY(addr1,addr2)	U32_SET(addr1, U32_GET(addr2))

#define BOUND(x, l, m)			((x) < (l) ? (l) : ((x) > (m) ? (m) : (x)))
#define SET_BOUND(x, l, m)		((x) = BOUND(x, l, m))
#define BOUND_INC(x, m)			do{++(x); (x) = (x) < (m) ? (x) : 0;} while(0)
#define BOUND_INC_POW2(x, m)	do{                             \
									STATIC_ASSERT_POW2(m);      \
									(x) = ((x)+1) & (m-1);      \
								}while(0)

#define IS_POWER_OF_2(x)		(!(x & (x-1)))
#define IS_LITTLE_ENDIAN 		(*(unsigned short*)"\0\xff" > 0x100)

#define IMPLIES(x, y) 			(!(x) || (y))

// x > y ? 1 : (x ==y : 0 ? -1)
#define COMPARE(x, y) 			(((x) > (y)) - ((x) < (y)))
#define SIGN(x) 				COMPARE(x, 0)

// better than xor swap: http://stackoverflow.com/questions/3912699/why-swap-with-xor-works-fine-in-c-but-in-java-doesn't-some-puzzle
#define SWAP(x, y, T) 			do { T tmp = (x); (x) = (y); (y) = tmp; } while(0)
#define SORT2(a, b, T) 			do { if ((a) > (b)) SWAP((a), (b), T); } while (0)

#define foreach(i, n) 			for(int i = 0; i < (n); ++i)
#define foreach_range(i, s, e) 	for(int i = (s); i < (e); ++i)
#define foreach_arr(i, arr) 	for(int i = 0; i < ARRAY_SIZE(arr); ++i)
//  round robbin foreach
#define foreach_hint(i, n, h) 	for(int i = 0, ++h, h=h<n?h:0; i < n; ++h, h=h<n?h:0)

#define ARRAY_SIZE(a) 			(sizeof(a) / sizeof(*a))

#define everyN(i, n) 			++(i); (i)=((i) < N ? (i) : 0); if(0 == (i))


#define HI_UINT16(a) 			(((a) >> 8) & 0xFF)
#define LO_UINT16(a) 			((a) & 0xFF)

#define U24_BYTE0(a) 			((a) & 0xFF)
#define U24_BYTE1(a) 			(((a) >> 8) & 0xFF)
#define U24_BYTE2(a) 			(((a) >> 16) & 0xFF)

#define U32_BYTE0(a) 			((a) & 0xFF)
#define U32_BYTE1(a) 			(((a) >> 8) & 0xFF)
#define U32_BYTE2(a) 			(((a) >> 16) & 0xFF)
#define U32_BYTE3(a) 			(((a) >> 24) & 0xFF)

#define BUILD_U16(lo, hi)			( (unsigned short)((((hi) & 0x00FF) << 8) + ((lo) & 0x00FF)) )
#define BUILD_U24(b0, b1, b2)		( (unsigned int)((((b2) & 0x000000FF) << 16) + (((b1) & 0x000000FF) << 8) + ((b0) & 0x000000FF)) )
#define BUILD_U32(b0, b1, b2, b3)	( (unsigned int)((((b3) & 0x000000FF) << 24) + (((b2) & 0x000000FF) << 16) + (((b1) & 0x000000FF) << 8) + ((b0) & 0x000000FF)) )

#define BUILD_S16(lo, hi)			( (signed short)((((hi) & 0x00FF) << 8) + ((lo) & 0x00FF)) )
#define BUILD_S24(b0, b1, b2)		( (signed int)((((b2) & 0x000000FF) << 16) + (((b1) & 0x000000FF) << 8) + ((b0) & 0x000000FF)) )
#define BUILD_S32(b0, b1, b2, b3)	( (signed int)((((b3) & 0x000000FF) << 24) + (((b2) & 0x000000FF) << 16) + (((b1) & 0x000000FF) << 8) + ((b0) & 0x000000FF)) )

#define HASH_MAGIC_VAL 			5381u
#define INT_MASK      			0x7fffffff

/* TODO: check is this hash optimal for ints */
#define HASH_FUNC_STEP(hash_var, v) 		((((hash_var) << 5) + (hash_var)) + (v))
#define FOUR_INT_HASH_FUNC(v1, v2, v3, v4) 	(HASH_FUNC_STEP(HASH_FUNC_STEP( HASH_FUNC_STEP( HASH_FUNC_STEP(HASH_MAGIC_VAL, (v1)), (v2)), (v3)), (v4)) & INT_MASK)
#define TWO_INT_HASH_FUNC(v1, v2)      		(HASH_FUNC_STEP(HASH_FUNC_STEP(HASH_MAGIC_VAL, (v1)), (v2)) & INT_MASK)

#define CONV_PCT_TO_Q0_16(x)    ((uint16_t)(((float)(x) * (float)UINT16_MAX) / 100.0f))
#define CONV_Q0_16_TO_PCT(x)    (((float)(x) * 100.0f) / (float)UINT16_MAX)

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
uint16_t crc16_ccitt(const uint8_t* data, uint32_t len);
bool util_is_timeout_ms(uint32_t *pTimeStamp, uint32_t timeout);
float clampf(float v, float lo, float hi);
		
#endif /* END FILE */
