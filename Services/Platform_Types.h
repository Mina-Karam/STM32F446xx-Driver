/*
 * Platform_Types.h
 *
 * Created on : Dec 22, 2021
 *     Author : Mina Karam
 */

#ifndef PLATFORM_TYPES_H_
#define PLATFORM_TYPES_H_

// Unsigned integer data types
typedef unsigned char               uint8_t;
typedef unsigned short              uint16_t;
typedef unsigned int                uint32_t;
typedef unsigned long long int      uint64_t;

// Signed integer data types
typedef char                 int8_t;
typedef short                int16_t;
typedef int                  int32_t;
typedef long long int        int64_t;

// volatile unsigned integer data types
typedef volatile unsigned char              vuint8_t;
typedef volatile unsigned short             vuint16_t;
typedef volatile unsigned int               vuint32_t;
typedef volatile unsigned long long int     vuint64_t;

// volatile signed integer data types
typedef volatile char                vint8_t;
typedef volatile short               vint16_t;
typedef volatile int                 vint32_t;
typedef volatile long long int       vint64_t;

// Float data types
typedef float       float32;
typedef double      float64;

// Boolean date types
typedef enum
{
	false,
	ture,
}bool;

// Error types which used in ErrorIndication Function in any Default Case in Switch
typedef enum
{
	InvalidArgument,    //in any configuration to Pins or Ports
	OverFlow            //in any configuration to LEDs For Example
}ErrorType;

#define NULL ((void *)0)

#endif /* PLATFORM_TYPES_H_ */
