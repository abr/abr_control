/**
 * 	\file 		KeosType.h
 *
 *	\date		2600 B.C.
 *	\author		Zeus
 * 	\brief		This files regroups KEOS standard types defines and include
 *
 *  \ingroup 	KeosTypesGroup
 */


#ifndef __KEOSTYPE_H__
#define __KEOSTYPE_H__

/**
 * \defgroup KeosTypesGroup	KEOS STANDARD TYPES
 * \ingroup KeosGroup
 *
 * @{
 */

#ifndef NULL
	#define NULL 0
#endif



typedef unsigned char  Uint8;
typedef unsigned short Uint16;
typedef char	Int8;
typedef short	Int16;
typedef unsigned long Uint32;
typedef  long Int32;
typedef unsigned char  byte;			//!< 8 bits unsigned integer
typedef unsigned short word;			//!< 16 bits unsigned integer
typedef unsigned long  ulong;			//!< 32 bits unsigned integer
typedef unsigned long long Uint64;		//!< 64 bits unsigned integer
typedef long long Int64;				//!< 64 bits signed integer

typedef int ErrorCode;					//!< Standard type for error codes return


enum StringSize
{
	JacoStringSize = 20
};

typedef char JacoString[JacoStringSize];

/**
 *  \brief Byte union allowing to access data a signed on unsigned byte
 */
union BYTE
{
	unsigned char uByteData;	//!<	Access data as unsigned bytes
	char ByteData;				//!<	Access data	as signed bytes
};

/**
 * \brief 16 bit Union
 * \ingroup KeosTypesGroup
 */
union WORD
{
	Uint8 uByteData[2];		//!<	Access data as unsigned bytes
	char ByteData[2];		//!<	Access data	as signed bytes
	Uint16 uShortData;		//!<	Access data as unsigned short (16 bits unsigned integer)
	short ShortData;		//!<	Access data as signed short (16 bits integer)

};

/**
 * 32 bits Union
 * \ingroup KeosTypesGroup
 */
union LONG
{
	Uint8 uByteData[4];			//!<	Access data as unsigned bytes
	Int8 ByteData[4];			//!<	Access data	as signed bytes
	Uint16 uShortData[2];		//!<	Access data as unsigned shorts (16 bits unsigned integer)
	Int16 ShortData[2];			//!<	Access data as signed shorts (16 bits integer)
	Uint32 uLongData;			//!<	Access data as unsigned long (32 bits unsigned integer)
	Int32 LongData;				//!<	Access data as signed long (32 bits integer)
	float FloatData;			//!<	Access data as float
};

typedef union BYTE u_byte;		//!< 8 bits union
typedef union WORD u_word;		//!< 16 bits union
typedef union LONG longUnion;	//!< 32 bits union

/**
 * @}
 */
#endif
