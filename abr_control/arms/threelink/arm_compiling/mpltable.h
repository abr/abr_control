/* Maple RTable Definitions Header File 
   Copyright (c) 1998 by Waterloo Maple, Inc.
   All rights reserved. Unauthorized duplication prohibited.
   Written June 1998 by Paul DeMarco.

   *** THIS FILE IS INTENDED FOR DISTRIBUTION WITH THE MAPLE PRODUCT ***

   This file provides rtable definitions required to implement shared modules 
   that can be linked into a running Maple kernel. Typically, such modules are
   generated automatically by Maple's define_external function, and are then
   used to call other non-Maple shared modules. However, a user could, in
   principle, write a Maple-specific shared module that doesn't require this
   layering. */

#ifndef __MPLTABLE_H__
#define __MPLTABLE_H__

/* Values for the RTABLE_DATATYPE sub-field. */
#define RTABLE_DAG	 	0
#define RTABLE_CXDAG	 	1
#define RTABLE_INTEGER8	 	2
#define RTABLE_INTEGER16 	3
#define RTABLE_INTEGER32 	4
#define RTABLE_INTEGER64 	5
#define RTABLE_FLOAT     	6
#define RTABLE_FLOAT64     	6
#define RTABLE_COMPLEX   	7
#define RTABLE_FLOAT32     	8
#define RTABLE_NUM_DATATYPES	9

#define RTABLE_UINTEGER8	9
#define RTABLE_UINTEGER16 	10
#define RTABLE_UINTEGER32 	11
#define RTABLE_UINTEGER64 	12

/* Datatypes that match above sizes. */
#define INTEGER8	signed char
#define INTEGER16	short
#define UINTEGER16	unsigned INTEGER16
#define INTEGER32	int
#define UINTEGER32 	unsigned INTEGER32

#if defined _MSC_VER
#  define INTEGER64	__int64
#elif defined __alpha \
    || defined __ia64__ \
    || defined __x86_64__ \
    || defined __ppc64__ /* 64-bit Architectures (non-Windows) */
#  define INTEGER64	long
#else
#  define INTEGER64	long long
#endif
#define FLOAT32		float
#define FLOAT64		double
#define COMPLEXF32      ComplexFloat32
#define COMPLEXF64      ComplexFloat64
#define CXDAG		ComplexFloatDAG

/* Sizes for integer types. */
#define MAX_INTEGER8	127
#define MIN_INTEGER8	(-MAX_INTEGER8-1)
#define MAX_INTEGER16	32767
#define MIN_INTEGER16	(-MAX_INTEGER16-1)
#define MAX_INTEGER32	2147483647
#define MIN_INTEGER32	(-MAX_INTEGER32-1)
#if defined NO_INTEGER64
#  define MAX_INTEGER64 MAX_INTEGER32
#  define MIN_INTEGER64 MIN_INTEGER32
#elif defined _MSC_VER
#  define MAX_INTEGER64	9223372036854775807
#  define MIN_INTEGER64	(-MAX_INTEGER64-1)
#else
#  define MAX_INTEGER64	9223372036854775807LL
#  define MIN_INTEGER64	(-MAX_INTEGER64-1)
#endif

/* Values for the RTABLE_SUBTYPE sub-field. */
#define RTABLE_ARRAY  0
#define RTABLE_MATRIX 1
#define RTABLE_COLUMN 2
#define RTABLE_ROW    3

#define RTABLE_VECTOR(rt) (RTABLE_SUBTYPE(rt) > RTABLE_MATRIX)

/* Values for the RTABLE_STORAGE sub-field. */
#define RTABLE_SPARSE 		0
#define RTABLE_EMPTY         	1
#define RTABLE_DIAG		2
#define RTABLE_BAND		3
#define RTABLE_RECT		4
#define RTABLE_UPPTRI		5
#define RTABLE_UPPTRIMINUS	6
#define RTABLE_UPPTRIPLUS	7
#define RTABLE_LOWTRI		8
#define RTABLE_LOWTRIMINUS	9
#define RTABLE_LOWTRIPLUS	10
#define RTABLE_SCALAR		11

/* Sparse-Storage Qualifiers */
#define RTABLE_SPARSE_DEFAULT		0
#define RTABLE_SPARSE_UPPER		1
#define RTABLE_SPARSE_LOWER		2

#define RTABLE_IS_SPARSE(rt) (RTABLE_STORAGE(rt)==RTABLE_SPARSE)

/* Values for the RTABLE_ORDER sub-field. */
#define RTABLE_FORTRAN 0
#define RTABLE_C 1

/* The RTABLE_READONLY and RTABLE_FOREIGN sub-fields are TRUE or FALSE. */

/* Kernel-defined indexing functions */
#define NON_KERNEL_INDFN                -1
#define RTABLE_INDEX_ZERO               0
#define RTABLE_INDEX_BAND               1
#define RTABLE_INDEX_CONSTANT           2
#define RTABLE_INDEX_DIAGONAL           3
#define RTABLE_INDEX_SCALAR             4
#define RTABLE_INDEX_ANTIHERMITIAN      5
#define RTABLE_INDEX_HERMITIAN          6
#define RTABLE_INDEX_UPPHESSEN          7
#define RTABLE_INDEX_LOWHESSEN          8
#define RTABLE_INDEX_IDENTITY           9
#define RTABLE_INDEX_ANTISYMMETRIC      10
#define RTABLE_INDEX_SYMMETRIC          11
#define RTABLE_INDEX_LOWTRI             12
#define RTABLE_INDEX_UPPTRI             13
#define RTABLE_INDEX_UNIUPPTRI          14
#define RTABLE_INDEX_UNILOWTRI          15
#define RTABLE_INDEX_UNIT               16
#define RTABLE_INDEX_NONE               17

/* Restrictions. */
#define RTABLE_MAX_DIMS 63
#define RTABLE_MAX_BOUNDS (2*RTABLE_MAX_DIMS)

/* Offset Macros */
#define INDEX_NOT_IN_STORAGE -1

/* Vector Offset Macros */
/* for a vector, 'V' of length m, give the offset for 'V'[i] */
/* Note, these all assume m>=i>=1 */
#define VECTOR_OFFSET_RECT(i) ((i)-1)
#define VECTOR_OFFSET_EMPTY(i) (INDEX_NOT_IN_STORAGE)
#define VECTOR_OFFSET_SCALAR(i) (0)
#define VECTOR_OFFSET_DIAG(i) ((i)-1)

/* Matrix Offset Macros */
/* for an mxn matrix, 'A', give the offset for 'A'[i,j] */
/* Note, these all assume m>=i>=1 and n>=j>=1 */
#define INDEX_NOT_IN_STORAGE -1

#define MATRIX_OFFSET_FORTRAN_UPPTRI(i,j,m,n) \
   (((i)>(j)) ? INDEX_NOT_IN_STORAGE \
	  : (((j)>(m)) ? (2*(j)-1-(m))*(m)/2+(i)-1 \
		   : (j)*((j)-1)/2+(i)-1))
#define MATRIX_OFFSET_C_UPPTRI(i,j,m,n) \
   (((i)>(j)) ? INDEX_NOT_IN_STORAGE \
	  : (((i)-1)*(2*(n)+2-(i))/2+(j)-(i)))

#define MATRIX_OFFSET_FORTRAN_LOWTRI(i,j,m,n) \
   (((i)<(j)) ? INDEX_NOT_IN_STORAGE  \
	  : ((j)-1)*(2*(m)+2-(j))/2+(i)-(j))
#define MATRIX_OFFSET_C_LOWTRI(i,j,m,n) \
   (((i)<(j)) ? INDEX_NOT_IN_STORAGE  \
	  : ((i)>(n)) ? (2*(i)-2+1-(n))*(n)/2+(j)-1  \
		  : ((i)-1)*(i)/2+(j)-1)

#define MATRIX_OFFSET_FORTRAN_UPPTRIMINUS(i,j,m,n) \
   (((i)>=(j)) ? INDEX_NOT_IN_STORAGE  \
	   : ((j)>(m)) ? (2*(j)-3-(m))*(m)/2+(i)-1 \
		   : ((j)-1)*((j)-2)/2+(i)-1)
#define MATRIX_OFFSET_C_UPPTRIMINUS(i,j,m,n) \
   (((i)>=(j)) ? INDEX_NOT_IN_STORAGE  \
	   : ((i)-1)*(2*(n)-(i))/2+(j)-(i)-1)

#define MATRIX_OFFSET_FORTRAN_LOWTRIMINUS(i,j,m,n) \
   (((i)<=(j)) ? INDEX_NOT_IN_STORAGE  \
	   : ((j)-1)*(2*(m)-(j))/2+(i)-(j)-1)
#define MATRIX_OFFSET_C_LOWTRIMINUS(i,j,m,n) \
   (((i)<=(j)) ? INDEX_NOT_IN_STORAGE  \
	   : ((i)>(n)) ? (2*(i)-3-(n))*(n)/2+(j)-1 \
		   : ((i)-1)*((i)-2)/2+(j)-1)
 
#define MATRIX_OFFSET_FORTRAN_UPPTRIPLUS(i,j,m,n) \
   (((i)>(j)+1) ? INDEX_NOT_IN_STORAGE  \
	    : ((j)>(m)) ? (2*(j)+1-(m))*(m)/2+(i)-2 \
		    : (j)*((j)+1)/2+(i)-2)
#define MATRIX_OFFSET_C_UPPTRIPLUS(i,j,m,n) \
   (((i)>(j)+1) ? INDEX_NOT_IN_STORAGE  \
	    : ((j)>=(i)) ? ((i)-1)*(2*(n)+4-(i))/2+(j)-(i) \
		     : ((i)-1)*(2*(n)+2-(i))/2 +2*(j)-(i))

#define MATRIX_OFFSET_FORTRAN_LOWTRIPLUS(i,j,m,n) \
   (((j)>(i)+1) ? INDEX_NOT_IN_STORAGE  \
	    : ((i)>=(j)) ? ((j)-1)*(2*(m)+4-(j))/2+(i)-(j) \
		     : ((j)-1)*(2*(m)+2-(j))/2 +2*(i)-(j))
#define MATRIX_OFFSET_C_LOWTRIPLUS(i,j,m,n) \
   (((j)>(i)+1) ? INDEX_NOT_IN_STORAGE  \
	    : ((i)>(n)) ? (2*(i)+1-(n))*(n)/2+(j)-2 \
		    : (i)*((i)+1)/2+(j)-2)

#define MATRIX_OFFSET_FORTRAN_RECT(i,j,m,n) \
   (((j)-1)*(m)+((i)-1))
#define MATRIX_OFFSET_C_RECT(i,j,m,n) \
   (((i)-1)*(n)+((j)-1))

#define MATRIX_OFFSET_FORTRAN_EMPTY(i,j,m,n) \
   (INDEX_NOT_IN_STORAGE)
#define MATRIX_OFFSET_C_EMPTY(i,j,m,n) \
   (INDEX_NOT_IN_STORAGE)

#define MATRIX_OFFSET_FORTRAN_SCALAR(i,j,m,n) \
   (0)
#define MATRIX_OFFSET_C_SCALAR(i,j,m,n) \
   (0)

#define MATRIX_OFFSET_FORTRAN_DIAG(i,j,m,n) \
   (((i)!=(j)) ? INDEX_NOT_IN_STORAGE  \
	   : ((i)-1))
#define MATRIX_OFFSET_C_DIAG(i,j,m,n) \
   (((i)!=(j)) ? INDEX_NOT_IN_STORAGE  \
	   : ((i)-1))

#define MATRIX_OFFSET_FORTRAN_BAND(i,j,m,n,b1,b2) \
   (((i)-(j)<-(b2) || (i)-(j)>(b1)) ? INDEX_NOT_IN_STORAGE  \
			: (((j)-1)*((b2)+(b1)+1)+(i)-(j)+(b2)))
#define MATRIX_OFFSET_C_BAND(i,j,m,n,b1,b2) \
   (((i)-(j)<-(b2) || (i)-(j)>(b1)) ? INDEX_NOT_IN_STORAGE  \
			: (((i)-(j)+(b2))*(n)+(j)-1))

/* Specific select/assign functions, tailored to a given rtable */
typedef void (*RTableIndexChainCallback)( ALGEB rt, void *data );
typedef ALGEB (*RTableSelectFunction)( ALGEB rt, ALGEB ind );
typedef ALGEB (*RTable1DSelectFunction)( ALGEB rt, M_INT i );
typedef ALGEB (*RTable2DSelectFunction)( ALGEB rt, M_INT i, M_INT j );

typedef ALGEB (*RTableAssignFunction)( ALGEB rt, ALGEB ind, ALGEB val );
typedef ALGEB (*RTable1DAssignFunction)( ALGEB rt, M_INT i, ALGEB val );
typedef ALGEB (*RTable2DAssignFunction)( ALGEB rt, M_INT i, M_INT j, ALGEB val );

typedef M_INT (*RTableOffsetFunction)( ALGEB rt, ALGEB ind );
typedef M_INT (*RTable1DOffsetFunction)( ALGEB rt, M_INT i );
typedef M_INT (*RTable2DOffsetFunction)( ALGEB rt, M_INT i, M_INT j );

/* Types */
typedef struct _ComplexFloat32 {
    FLOAT32 re, im;
} ComplexFloat32;

typedef struct _ComplexFloat64 {
    FLOAT64 re, im;
} ComplexFloat64;

typedef struct _ComplexFloatDAG {
    ALGEB re, im;
} ComplexFloatDAG;

typedef union {
    INTEGER8      	int8;
    INTEGER16     	int16;
    INTEGER32     	int32;
    INTEGER64     	int64;
    FLOAT32       	float32;
    FLOAT64       	float64;
    ComplexFloat64    	complexf64;
    CXDAG	    	cxdag;
    ALGEB         	dag;
} RTableData;

typedef struct {
    int data_type;         /* type of data in the rtable */
    ALGEB maple_type;      /* set this if data_type is RTABLE_DAG */
    int subtype;           /* type of rtable -- array, matrix, vector */
    int storage;           /* storage layout */
    M_INT p1, p2;          /* optional parameters for some storage layouts */
    int order;             /* C or Fortran order indexing */
    M_BOOL read_only;      /* disallow data writes when true */
    M_BOOL foreign;        /* true if external program manages data memory */
    int num_dimensions;    /* a 2x3 rtable has 2 dimensions */
    ALGEB index_functions; /* list of array indexing functions */
    ALGEB attributes;      /* attributes given to this rtable */
    M_BOOL transpose;      /* when true, transpose the matrix at creation */
    ALGEB fill;            /* the value for unspecified elements */
} RTableSettings;

#endif /* MPLTABLE_H */

