/* Maple Shared Library Header File
   Copyright (c) 1996 by Waterloo Maple, Inc.
   All rights reserved. Unauthorized duplication prohibited.
   Written October 1996 by Stefan Vorkoetter.

   *** THIS FILE IS INTENDED FOR DISTRIBUTION WITH THE MAPLE PRODUCT ***

   This file provides definitions required to implement shared modules that
   can be linked into a running Maple kernel. Typically, such modules are
   generated automatically by Maple's define_external function, and are then
   used to call other non-Maple shared modules. However, a user could, in
   principle, write a Maple-specific shared module that doesn't require this
   layering. */

#ifndef __MPLSHLIB_H__
#define __MPLSHLIB_H__

#define MAPLE_RELEASE   14
#define MAPLE_REVISION  0

#ifndef M_DECL
#  ifdef WMI_WINNT
#    define M_DECL __stdcall
#    ifndef USE_PROTO
#      define USE_PROTO              /* stdcall requires prototypes */
#    endif
#  else
#    define M_DECL
#  endif
#endif

#if !defined GMP_DECL
#  ifdef WMI_WINNT
#    define GMP_DECL __cdecl
#  else
#    define GMP_DECL
#  endif
#endif

/* Decide whether or not to use ANSI C function prototypes. */
#if defined(__STDC__) || defined(__cplusplus) || defined(USE_PROTO)
# define MPL_P(x) x
#else
# define MPL_P(x) ()
#endif

/* gmp.h must be included before this file to use GMP conversion functions */
#ifndef __GMP_H__
# define mpz_ptr void*
#endif

#ifndef M_INT   /* Exclude most of this file when compiling Maple itself. */

/* Determine machine word size dependent parameters. */
#if defined __alpha \
    || defined __sparcv9 \
    || defined __ia64__ \
    || defined __x86_64__ \
    || defined __ppc64__ /* 64-bit Architectures (non-Windows)*/
# define M_INT long
# define RTABLE_LONG RTABLE_INTEGER64
# define WORDSIZE 64
#elif defined _M_X64 /* 64-bit Architecture (Windows) */
# define M_INT long long
# define RTABLE_LONG RTABLE_INTEGER32
# define WORDSIZE 64
#else /* 32-bit Architectures */
# define M_INT int
# define RTABLE_LONG RTABLE_INTEGER32
# define WORDSIZE 32
#endif

/* Maple Internal Types */
typedef M_INT ***ALGEB;
typedef M_INT M_BOOL;
typedef unsigned M_INT M_UINT;

#ifdef X86_64_WINDOWS
#    define NAG_INT long long
#else
#    define NAG_INT long
#endif
#define DAG(x) ((ALGEB)(x))
#define TRUE 1
#define FALSE 0
#define FAIL -1

#endif /* M_INT */

#ifndef HF_MAX_PARAMS
#define HF_MAX_PARAMS 50
#endif

#include <stdarg.h>
#include <stdlib.h>
#include "mpltable.h"

#ifdef COMPLEX_STRUCT_REDEF
#undef COMPLEXF64
#define COMPLEXF64 COMPLEX_STRUCT_REDEF
#endif

#define MAPLE_TASK_NO_CALLBACK 0x1

#define MAPLE_ENABLE_GMP 0x1
#define MAPLE_SUCCEEDED 0
#define MAPLE_ERROR_INVALID_OPTIONS 1
#define MAPLE_ERROR_ALREADY_REGISTERED 2
#define MAPLE_ERROR_NOT_REGISTERED 3

#ifdef __cplusplus
     extern "C" {
#endif

#if !defined __MAPLE_EVALHF_STRUCTS_H__
typedef struct _hfdata {
    double re, im;
    char id;
} hfdata;
#endif

/* Maple object ids */
typedef enum { 
    MAPLE_INTNEG = 1,
    MAPLE_INTPOS,
    MAPLE_RATIONAL,
    MAPLE_FLOAT,
    MAPLE_HFLOAT,
    MAPLE_COMPLEX,
    MAPLE_STRING,
    MAPLE_NAME,
    MAPLE_MEMBER,
    MAPLE_TABLEREF,
    MAPLE_DCOLON,
    MAPLE_CATENATE,
    MAPLE_POWER,
    MAPLE_PROD,
    MAPLE_SERIES,
    MAPLE_SUM,
    MAPLE_POLY,
    MAPLE_ZPPOLY,
    MAPLE_SDPOLY,
    MAPLE_FUNCTION,
    MAPLE_UNEVAL,
    MAPLE_EQUATION,
    MAPLE_INEQUAT,
    MAPLE_LESSEQ,
    MAPLE_LESSTHAN,
    MAPLE_AND,
    MAPLE_NOT,
    MAPLE_OR,
    MAPLE_XOR,
    MAPLE_IMPLIES,
    MAPLE_EXPSEQ,
    MAPLE_LIST,
    MAPLE_LOCAL,
    MAPLE_PARAM,
    MAPLE_LEXICAL,
    MAPLE_PROC,
    MAPLE_RANGE,
    MAPLE_SET,
    MAPLE_TABLE,
    MAPLE_RTABLE,
    MAPLE_MODDEF,
    MAPLE_MODULE,
    MAPLE_ASSIGN,
    MAPLE_FOR,
    MAPLE_IF,
    MAPLE_READ,
    MAPLE_SAVE,
    MAPLE_STATSEQ,
    MAPLE_STOP,
    MAPLE_ERROR,
    MAPLE_TRY,
    MAPLE_RETURN,
    MAPLE_BREAK,
    MAPLE_NEXT,
    MAPLE_USE
} MapleID;

/* DAG Conversion. These are implemented within Maple itself. */
/* The functions here have to be alphabetically ordered as in extsupp.c */
typedef struct _MKernelVectorDesc {
    /* Allocate Using Maple's Allocator */
    void   *(M_DECL *alloc) MPL_P(( M_INT nbytes ));
    ALGEB   (M_DECL *allocALGEB) MPL_P(( M_INT length, MapleID id ));

    /* Assign to Maple Variables */
    ALGEB   (M_DECL *assign) MPL_P(( ALGEB lhs, ALGEB rhs ));
    ALGEB   (M_DECL *assignIndexed) MPL_P(( ALGEB lhs, M_INT dim, 
					M_INT *ind, ALGEB rhs ));

    /* Free Memory Allocated By Maple's Allocator */
    void    (M_DECL *dispose) MPL_P(( ALGEB s ));

    /* Raising Errors */
    void    (M_DECL *error) MPL_P(( char *msg, /* ALGEB opt1, opt2, */ ... ));

    /* Evaluate an assigned name */
    ALGEB   (M_DECL *eval) MPL_P(( ALGEB s ));
    /* Evaluate a Maple Procedure or DAG using hardware floats */
    double  (M_DECL *evalhf) MPL_P(( ALGEB s ));
    /* first argument here is args[1] */
    double  (M_DECL *evalhfMapleProc) MPL_P(( ALGEB fn, int nargs, double *args ));

    /* Evaluate a Maple Procedure or statement */
    ALGEB   (M_DECL *evalMapleProc) MPL_P(( ALGEB fn, int nargs, /* ALGEB arg1, ALGEB arg2, */ ... ));
    ALGEB   (M_DECL *evalMapleProcedure) MPL_P(( ALGEB fn, ALGEB args ));
    ALGEB   (M_DECL *evalMapleStatement) MPL_P(( char *statement ));

    /* Data Queries */
    M_BOOL  (M_DECL *isAssignedName) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isComplexNumeric) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isInteger) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isInteger8) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isInteger16) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isInteger32) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isInteger64) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isMapleNULL) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isName) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isNumeric) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isPointer) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isPointerNULL) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isProcedure) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isRTable) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isString) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isTable) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isUnassignedName) MPL_P(( ALGEB s ));
    M_BOOL  (M_DECL *isUnnamedZero) MPL_P(( ALGEB s ));
    /* List Manipulation */
    ALGEB   (M_DECL *listAlloc) MPL_P(( M_INT num_members ));
    void    (M_DECL *listAssign) MPL_P(( ALGEB list, M_INT i, ALGEB val ));
    ALGEB   (M_DECL *listSelect) MPL_P(( ALGEB list, M_INT i ));

    /* Garbage Collection */
    void    (M_DECL *gcAllow) MPL_P(( ALGEB a ));
    void    (M_DECL *gcProtect) MPL_P(( ALGEB a ));

    /* Conversion From Maple Objects */
    COMPLEXF32     (M_DECL *mapleToComplexFloat32) MPL_P(( ALGEB s ));
    COMPLEXF64     (M_DECL *mapleToComplexFloat64) MPL_P(( ALGEB s ));
    CXDAG          (M_DECL *mapleToComplexFloatDAG) MPL_P(( ALGEB s ));
    FLOAT32        (M_DECL *mapleToFloat32) MPL_P(( ALGEB s ));
    FLOAT64        (M_DECL *mapleToFloat64) MPL_P(( ALGEB s ));
    INTEGER8       (M_DECL *mapleToInteger8) MPL_P(( ALGEB s ));
    INTEGER16      (M_DECL *mapleToInteger16) MPL_P(( ALGEB s ));
    INTEGER32      (M_DECL *mapleToInteger32) MPL_P(( ALGEB s ));
    INTEGER64      (M_DECL *mapleToInteger64) MPL_P(( ALGEB s ));
    M_BOOL	   (M_DECL *mapleToM_BOOL) MPL_P(( ALGEB s ));
    M_INT          (M_DECL *mapleToM_INT) MPL_P(( ALGEB s ));
    void*          (M_DECL *mapleToPointer) MPL_P(( ALGEB s ));
    char*          (M_DECL *mapleToString) MPL_P(( ALGEB s ));

    /* Determine the Number of Arguments in a Maple Object */
    M_INT   (M_DECL *numArgs) MPL_P(( ALGEB expr ));


    /* Rectangular Table (vector,matrix,array) Manipulation */
    void    (M_DECL *rtableAppendAttribute) MPL_P(( RTableSettings *s, char *name));
    void    (M_DECL *rtableAppendIndFn) MPL_P(( RTableSettings *s, ALGEB indfn ));
    RTableData (M_DECL *rtableAssign) MPL_P(( ALGEB rt, M_INT *index, RTableData val )); 
    ALGEB   (M_DECL *rtableCopy) MPL_P(( RTableSettings *s, ALGEB rt ));
    ALGEB   (M_DECL *rtableCopyImPart) MPL_P(( RTableSettings *s, ALGEB rt ));
    ALGEB   (M_DECL *rtableCopyRealPart) MPL_P(( RTableSettings *s, ALGEB rt ));
    ALGEB   (M_DECL *rtableCreate) MPL_P(( RTableSettings *s, void *pdata, M_INT *bounds ));
    void*   (M_DECL *rtableData) MPL_P(( ALGEB rt ));
    void    (M_DECL *rtableGetDefaults) MPL_P(( RTableSettings *s ));
    void    (M_DECL *rtableGetSettings) MPL_P(( RTableSettings *s, ALGEB rt ));
    M_INT   (M_DECL *rtableIndFn) MPL_P(( ALGEB rt, int num ));
    ALGEB   (M_DECL *rtableIndFnArgs) MPL_P(( ALGEB rt, int num ));
    M_BOOL  (M_DECL *rtableIsReal) MPL_P(( ALGEB rt ));
    M_INT   (M_DECL *rtableLowerBound) MPL_P(( ALGEB rt, int dim ));
    M_INT   (M_DECL *rtableNumElems) MPL_P(( ALGEB rt ));
    M_INT   (M_DECL *rtableNumDimensions) MPL_P(( ALGEB rt ));
    RTableData (M_DECL *rtableSelect) MPL_P(( ALGEB rt, M_INT *index )); 
    void    (M_DECL *rtableSetAttribute) MPL_P(( RTableSettings *s, char* name ));
    void    (M_DECL *rtableSetIndFn) MPL_P(( RTableSettings *s, ALGEB indfn ));
    void    (M_DECL *rtableSetType) MPL_P(( RTableSettings *s, int id, char* name));
    void    (M_DECL *rtableSparseCompact) MPL_P(( ALGEB rt ));
    NAG_INT* (M_DECL *rtableSparseIndexRow) MPL_P(( ALGEB rt, int dim ));
    ALGEB   (M_DECL *rtableSparseIndexSort) MPL_P(( ALGEB rt, int by_dim ));
    void    (M_DECL *rtableSparseSetNumElems) MPL_P(( ALGEB rt, M_INT num ));
    M_INT   (M_DECL *rtableSparseSize) MPL_P(( ALGEB rt ));
    M_INT   (M_DECL *rtableUpperBound) MPL_P(( ALGEB rt, int dim ));
    ALGEB   (M_DECL *rtableZipReIm) MPL_P(( RTableSettings *s, ALGEB rt_re, ALGEB rt_im ));

    /* Data Selection */
    ALGEB   (M_DECL *selectImaginaryPart) MPL_P(( ALGEB s ));
    ALGEB   (M_DECL *selectIndexed) MPL_P(( ALGEB s, M_INT dim, M_INT *ind ));
    ALGEB   (M_DECL *selectRealPart) MPL_P(( ALGEB s ));

    /* Data Uniquification */
    ALGEB   (M_DECL *simplify) MPL_P(( ALGEB s ));

    /* Table Manipulation */
    ALGEB   (M_DECL *tableAlloc) MPL_P(( void ));
    void    (M_DECL *tableAssign) MPL_P(( ALGEB table, ALGEB ind, ALGEB val ));
    void    (M_DECL *tableDelete) MPL_P(( ALGEB table, ALGEB ind ));
    M_BOOL  (M_DECL *tableHasEntry) MPL_P(( ALGEB table, ALGEB ind ));
    ALGEB   (M_DECL *tableSelect) MPL_P(( ALGEB table, ALGEB ind ));

    /* Conversion To Maple Objects */
    ALGEB   (M_DECL *toMapleBoolean) MPL_P(( M_INT b ));
    ALGEB   (M_DECL *toMapleChar) MPL_P(( M_INT c ));
    ALGEB   (M_DECL *toMapleComplex) MPL_P(( double re, double im ));
    ALGEB   (M_DECL *toMapleComplexFloat) MPL_P(( ALGEB re, ALGEB im ));
    ALGEB   (M_DECL *newMapleExpressionSequence) MPL_P(( int nargs ));
    ALGEB   (M_DECL *toMapleExpressionSequence) MPL_P(( int nargs, /* ALGEB arg1, ALGEB arg2, */ ... ));
    ALGEB   (M_DECL *toMapleInteger) MPL_P(( M_INT i ));
    ALGEB   (M_DECL *toMapleInteger64) MPL_P(( INTEGER64 i ));
    ALGEB   (M_DECL *toMapleFloat) MPL_P(( double f ));
    ALGEB   (M_DECL *toMapleName) MPL_P(( char *n, M_BOOL is_global ));
    ALGEB   (M_DECL *toMapleNULL) MPL_P(( void ));
    ALGEB   (M_DECL *toMapleNULLPointer) MPL_P(( void ));
    ALGEB   (M_DECL *toMaplePointer) MPL_P(( void *v, M_INT type ));
    ALGEB   (M_DECL *toMapleRelation) MPL_P(( const char *const rel, ALGEB lhs, ALGEB rhs ));
    ALGEB   (M_DECL *toMapleString) MPL_P(( char *s ));
    ALGEB   (M_DECL *toMapleUneval) MPL_P(( ALGEB s ));

    /* Evaluate a C Function, Trapping Any Raised Errors */
    void*   (M_DECL *traperror) MPL_P(( void *(M_DECL *proc) MPL_P(( void *data )),
                                 void *data, M_BOOL *errorflag ));

    /* Provide Run-time Information */
    void    (M_DECL *userinfo) MPL_P(( int level, char *name, char *msg ));

#ifndef ENHANCED_EF_BACKOUT

    /* Enhanced External Functions 
       (experimental and subject to change without notice) */
    ALGEB   (M_DECL *xxExtObHandleAlloc) MPL_P((void *val, void *typ,
                                            void (*marker)(void *v, void *t)
));
    M_BOOL  (M_DECL *xxExtObHandleTest)  MPL_P(( ALGEB ob ));
    void *  (M_DECL *xxExtObHandleType)  MPL_P(( ALGEB ob ));
    void *  (M_DECL *xxExtObHandleValue) MPL_P(( ALGEB ob ));

    void    (M_DECL *xxGcRequest) MPL_P(( void ));
    M_BOOL  (M_DECL *xxGcAddEndListener)   MPL_P(( void (*fun) MPL_P(( void )) ));
    M_BOOL  (M_DECL *xxGcAddMarkListener)  MPL_P(( void (*fun) MPL_P(( void )) ));
    M_BOOL  (M_DECL *xxGcAddStartListener) MPL_P(( void (*fun) MPL_P(( void )) ));
    M_INT   (M_DECL *xxGcHeapObjectWalk) MPL_P(( void **splace, void **oplace,
					     M_INT obc, void **obv ));
    M_INT   (M_DECL *xxGcHeapSectionWalk)  MPL_P(( void **splace ));
    M_BOOL  (M_DECL *xxGcMarkOne) MPL_P(( ALGEB ));
    void ** (M_DECL *xxGcMemInfo) MPL_P(( void ));

    ALGEB   (M_DECL *xxNewUnsimplified) MPL_P(( M_INT id, M_INT length ));
    M_INT   (M_DECL *xxObjectId)  MPL_P(( ALGEB ));
    M_INT   (M_DECL *xxObjectNArgs)  MPL_P(( ALGEB ));
#endif /*--ENHANCED_EF--*/

    /* print C structures to Maple's gui */
    int     (M_DECL *printf) MPL_P(( const char *fmt, ... ));

    /* Record constructor. First argument n is the number of slots. It is
       followed by n pairs of type ( char *, ALGEB ) that determine the
       slot names and initial values, respectively. */
    ALGEB   (M_DECL *record) MPL_P(( const int n, ... ));

    /* Function call constructor. The first argument `funcname' need not be a
       name; it can be any expression that may appear in the name part of a
       FUNCTION dag. The number of arguments passed as arguments after the
       first two is given in the second argument nargs. */
    ALGEB   (M_DECL *function) MPL_P(( const ALGEB funcname, const M_INT nargs, ... ));

    void    (M_DECL *disposeALGEB) MPL_P(( ALGEB s ));

    /* lprint Maple structures to Maple's gui */
    void    (M_DECL *lprint) MPL_P(( ALGEB s ));

    /* print Maple structures using Maple's gui (... are all type ALGEB) */
    int     (M_DECL *maplePrintf) MPL_P(( const char *fmt, ... ));

    /* print Maple structures to a string (... are all type ALGEB) */
    int     (M_DECL *mapleSPrintf) MPL_P(( char *s, const char *fmt, ... ));

    /* lookup help */
    char * (M_DECL *helpLookUpText) MPL_P(( char *topic, char *section,
	    M_BOOL (M_DECL *writechar) ( void *data, int c ),
	    M_BOOL (M_DECL *writeattrib) ( void *data, int a ),
	    int width, void *data ));

    /* set/query path to libraries and help */
    ALGEB (M_DECL *libname) MPL_P(( ALGEB expseq )); 

    /* end of session check */ 
    M_BOOL  (M_DECL *isStop) MPL_P(( ALGEB s ));

    /* session management */
    M_BOOL  (M_DECL *restart) MPL_P(( char *error ));
    void  (M_DECL *quit) MPL_P(( void ));

    /* settings */
    ALGEB (M_DECL *kernelopts) MPL_P(( char *option, ALGEB value ));

    /* Numeric Arithmetics */
    ALGEB  (M_DECL *numericAdd) MPL_P(( ALGEB a, ALGEB b ));
    ALGEB  (M_DECL *numericSubtract) MPL_P(( ALGEB a, ALGEB b ));
    ALGEB  (M_DECL *numericMultiply) MPL_P(( ALGEB a, ALGEB b ));
    ALGEB  (M_DECL *numericPower) MPL_P(( ALGEB a, M_INT n ));
    ALGEB  (M_DECL *integerDivide) MPL_P(( ALGEB a, ALGEB b, ALGEB *r ));
    int  (M_DECL *integerAbsCompare) MPL_P(( ALGEB a, ALGEB b ));
    ALGEB  (M_DECL *floatDivide) MPL_P(( ALGEB a, ALGEB b ));
 
    /* misc */
    void   (M_DECL *rtableSparseResize) MPL_P(( ALGEB rt, M_INT size ));
    M_BOOL (M_DECL *isMapleList) MPL_P(( ALGEB s ));
    M_BOOL (M_DECL *isMapleSet) MPL_P(( ALGEB s ));
    M_BOOL (M_DECL *gcIsProtected) MPL_P(( ALGEB p )); 
    ALGEB    (M_DECL *mapleSafeSPrintf) MPL_P(( const char *fmt, ... ));
    int (M_DECL *va_printf) ( const char *format, va_list ap );
    int (M_DECL *va_ALGEBprintf) ( const char *format, va_list ap );
    ALGEB (M_DECL *va_ALGEBsprintf) ( const char *format, va_list ap );

    /* pointer management */
    M_INT  (M_DECL *pointerType) MPL_P(( ALGEB p ));
    void   (M_DECL *pointerSetType) MPL_P(( ALGEB p, M_INT type ));
    void   (M_DECL *pointerSetMarkFunction) MPL_P(( ALGEB p, 
		void (M_DECL *markfn) ( ALGEB p) ));
    void   (M_DECL *pointerSetPrintFunction) MPL_P(( ALGEB p, 
		ALGEB (M_DECL *printfn) ( ALGEB p) ));
    void   (M_DECL *pointerSetDisposeFunction) MPL_P(( ALGEB p, 
		void (M_DECL *disposefn) ( ALGEB p) ));
    M_BOOL (M_DECL *isMapleExpressionSequence) MPL_P(( ALGEB s ));
    void (M_DECL *registerRestartCallBack) MPL_P(( void (M_DECL *callback) ( void *data ) ) );
    void (M_DECL *MapleExpressionSequenceAssign) MPL_P(( ALGEB e, M_INT i, ALGEB v ));
    ALGEB (M_DECL *MapleExpressionSequenceSelect) MPL_P(( ALGEB e, M_INT i ));
    void  (M_DECL *mapleToComplexSingle) MPL_P(( ALGEB s, COMPLEXF32 *c ));
    void  (M_DECL *mapleToComplexDouble) MPL_P(( ALGEB s, COMPLEXF64 *c ));
    ALGEB (M_DECL *rtableCreateFromALGEB) MPL_P(( RTableSettings *s,
						ALGEB obj, M_INT bounds[] ));
    mpz_ptr (M_DECL *mapleToGMPInteger) MPL_P(( ALGEB dag ));
    ALGEB (M_DECL *GMPIntegerToMaple) MPL_P(( mpz_ptr g ));
    char* (M_DECL *appendFeature) MPL_P(( char*, char*, char*, char* ));
    void (M_DECL *removeFeature) MPL_P(( char*, char*, char*, char* ));
    void (M_DECL *getRTableSelectFunction) MPL_P(( ALGEB rt, 
		RTableSelectFunction *sf ));
    void (M_DECL *getRTableAssignFunction) MPL_P(( ALGEB rt, 
		RTableAssignFunction *af ));
    M_BOOL (M_DECL *getRTableOffsetFunction) MPL_P(( ALGEB rt, 
		RTableOffsetFunction *of ));
    void (M_DECL *getRTable1DSelectFunction) MPL_P(( ALGEB rt, 
		RTable1DSelectFunction *sf ));
    void (M_DECL *getRTable1DAssignFunction) MPL_P(( ALGEB rt, 
		RTable1DAssignFunction *af ));
    void (M_DECL *getRTable2DSelectFunction) MPL_P(( ALGEB rt, 
		RTable2DSelectFunction *sf ));
    void (M_DECL *getRTable2DAssignFunction) MPL_P(( ALGEB rt, 
		RTable2DAssignFunction *af ));

    /* Random functions */

    INTEGER32 (M_DECL *randomInt32) MPL_P(( void ));
    INTEGER64 (M_DECL *randomInt64) MPL_P(( void ));
    M_INT (M_DECL *randomM_INT) MPL_P(( void ));
    double (M_DECL *randomDouble01) MPL_P(( void ));
    ALGEB (M_DECL *randomSoftwareFloat01) MPL_P(( void ));
    ALGEB (M_DECL *randomSoftwareInteger) MPL_P(( M_INT bits ));
    M_INT (M_DECL *randomCalcBits) MPL_P(( ALGEB range ));
    INTEGER32 (M_DECL *randomRangeInt32) MPL_P(( INTEGER32 range, M_INT bits ));
    ALGEB (M_DECL *randomRangeSoftwareInt) MPL_P(( ALGEB range, M_INT bits ));

    ALGEB (M_DECL *nameValue) MPL_P(( ALGEB s ));

    void (M_DECL *mapleTohfData) MPL_P(( ALGEB al, hfdata *data ));
    ALGEB (M_DECL *toMaplehfData) MPL_P(( hfdata *data ));
    hfdata (M_DECL *evalhfDataProc) MPL_P(( ALGEB fn, M_INT nargs, hfdata *args ));
    void (M_DECL *doubleTohfData) MPL_P(( double r, double i, hfdata *args ));
    ALGEB (M_DECL *ssystem) MPL_P(( char *cmd, M_INT timeout ));
    /* Lookup symbol SYM in external library LIBNAME. */
    void (M_DECL *rtableSparseSetNumSorted) MPL_P(( ALGEB rt, M_INT num ));
    void (M_DECL *registerPlotCallBack) MPL_P(( void (M_DECL *callback) ( void *data, void* ) ) );
    void (M_DECL *registerStreamCallBack) MPL_P(( char *name, ALGEB (M_DECL *callback) ( void *data, ALGEB ) ) );
    void (M_DECL *registerGUIStreamCallBack) MPL_P(( char *name, ALGEB (M_DECL *callback) ( void *data, ALGEB ) ) );
    void (M_DECL *registerPostRestartCallBack) MPL_P(( void (M_DECL *callback) ( void *data ) ) );
    M_UINT (M_DECL *dlsym) MPL_P(( const char *libname, const char *sym ));
    M_INT (M_DECL *checkInterrupt) MPL_P(( void ));
    ALGEB   (M_DECL *toMapleAllanFloat) MPL_P(( double f ));
    void *client;
    void *cb;
    M_BOOL (M_DECL *isFloat64) MPL_P(( ALGEB dag ));
    M_BOOL (M_DECL *isComplex64) MPL_P(( ALGEB dag ));
    M_BOOL (M_DECL *isTableBasedArray) MPL_P(( ALGEB dag ));
    ALGEB* (M_DECL *getElems) MPL_P(( ALGEB container, M_INT *num_elems ));
    ALGEB (M_DECL *createALGEB) MPL_P(( MapleID id, M_INT num_elems, ALGEB *elems ));
    M_INT* (M_DECL *rtableBounds) MPL_P(( ALGEB rt ));
    ALGEB (M_DECL *rtableCreateFromDataBlock) MPL_P(( RTableSettings *rts,
           M_INT *bounds, void *data_block, M_INT block_datatype ));
    void (M_DECL *pushErrorProc) MPL_P(( void (M_DECL *errorproc)( char *, void * ), void *data ));
    void (M_DECL *popErrorProc) MPL_P(( void ));
    M_BOOL (M_DECL *getInterruptValue) MPL_P(( void ));
    void (M_DECL *pushGMPAllocators)(
        void *(GMP_DECL *malloc)( size_t ),
        void *(GMP_DECL *realloc)( void *, size_t, size_t ),
        void (GMP_DECL *free)( void *, size_t ) );
    void (M_DECL *popGMPAllocators) MPL_P(( void ));

    /* mutex functions */
    ALGEB (M_DECL *mutexCreate) MPL_P(( ALGEB options ));
    void (M_DECL *mutexDestroy) MPL_P(( ALGEB id ));
    void (M_DECL *mutexLock) MPL_P(( ALGEB id ));
    M_BOOL (M_DECL *mutexTryLock) MPL_P(( ALGEB id ));
    void (M_DECL *mutexUnlock) MPL_P(( ALGEB id ));

    /* task functions */
    int (M_DECL *taskStart)( void *root, 
            int (*TaskFunction)( void *parent, int arg_numbers, void *self ), 
            void *args, void (*MarkTaskFunction)( void *self ), void **ret, 
            M_INT options );
    void (M_DECL *taskContinue)( 
            int (*TaskFunction)( void *parent, int arg_numbers, void *self ), 
            void *args, void (*MarkTaskFunction)( void *self ) );
    void (M_DECL *taskChild)( int arg_number, 
            int (*TaskFunction)( void *parent, int arg_numbers, void *self ), 
            void *args, void (*MarkTaskFunction)( void *self ) );
    int (M_DECL *taskReturn)( void *value, void (*MarkFunction)( void *self ) );

    ALGEB   (M_DECL *toMapleHFloat) MPL_P(( double f ));
    int     (M_DECL *sessionNumber) MPL_P(( void ));

    /* register external threads */
    M_BOOL (M_DECL *registerExternalThread) MPL_P(( M_INT options ));
    M_BOOL (M_DECL *unregisterExternalThread) MPL_P(( void ));

    ALGEB (M_DECL *rtableResize) MPL_P(( ALGEB rt, M_INT *bounds, M_INT num_dims ));

} MKernelVectorDesc, *MKernelVector;

#ifdef __cplusplus
     }
#endif

#endif /* MPLSHLIB_H */
