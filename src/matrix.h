/*
* PUBLISHed header for the libmx library.
*
* Copyright 1984-2013 The MathWorks, Inc.
* All Rights Reserved.
*/

/*
 * NOTE: The contents of this header are ultimately PUBLISHED in
 * extern/include/matrix.h.
 */

#if defined(_MSC_VER)
# pragma once
#endif
#if defined(__GNUC__) && (__GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ > 3))
# pragma once
#endif

#ifndef MATRIX_DEVELOPER_API_HPP
#ifndef matrix_h
#define matrix_h
#include <stdlib.h>
#include <stddef.h>
#include "tmwtypes.h"

/* we can't see these definitions, which are stored in package.h, so we
   duplicate them here. */
#ifdef _MSC_VER
  #define MATRIX_DLL_EXPORT_SYM __declspec(dllexport)
  #define MATRIX_DLL_IMPORT_SYM __declspec(dllimport)
#elif __GNUC__ >= 4
  #define MATRIX_DLL_EXPORT_SYM __attribute__ ((visibility("default")))
  #define MATRIX_DLL_IMPORT_SYM __attribute__ ((visibility("default")))
#else
  #define MATRIX_DLL_EXPORT_SYM
  #define MATRIX_DLL_IMPORT_SYM
#endif

/**
 * Define symbol access for symbols exported from the libmwmatrix dll.
 */
#if defined(BUILDING_PUBLISHED_API_CPP)
#define LIBMMWMATRIX_PUBLISHED_API MATRIX_DLL_EXPORT_SYM
#else
#if defined(export_matrix_h)
/* we are a C file coming through /src/include/matrix.h */
#define LIBMMWMATRIX_PUBLISHED_API MATRIX_DLL_IMPORT_SYM
#else
/* We are a C mex file coming through /extern/include/matrix.h
 * LIBMMWMATRIX_PUBLISHED_API is empty to match definitions in mex.h.
 */
#ifdef LIBMMWMATRIX_PUBLISHED_API
#undef LIBMMWMATRIX_PUBLISHED_API
#endif
#define LIBMMWMATRIX_PUBLISHED_API
#endif /* export_matrix_h */
#endif /* BUILDING_PUPLISHED_API_CPP */

#ifdef __cplusplus
#define LIBMMWMATRIX_PUBLISHED_API_EXTERN_C extern "C"  LIBMMWMATRIX_PUBLISHED_API
#else
#define LIBMMWMATRIX_PUBLISHED_API_EXTERN_C extern  LIBMMWMATRIX_PUBLISHED_API
#endif


/* Version 7.4.0 */
#define MX_API_VER 0x07040000


/* On linux function symbol names are chosen at link time. Thus on linux
 * we need to strip _730 from function declarations. On windows and mac
 * we chose proper function name at compilation time.
 */

#if !defined(MX_COMPAT_32) && !defined(BUILDING_PUBLISHED_API_CPP) && defined(__linux__)

#ifdef __cplusplus
extern "C" {
#endif

#ifndef mxSetProperty_730
# define mxSetProperty_730 mxSetProperty
#endif

#ifndef mxGetProperty_730
# define mxGetProperty_730 mxGetProperty
#endif

#ifndef mxSetField_730
# define mxSetField_730 mxSetField
#endif

#ifndef mxSetFieldByNumber_730
# define mxSetFieldByNumber_730 mxSetFieldByNumber
#endif

#ifndef mxGetFieldByNumber_730
# define mxGetFieldByNumber_730 mxGetFieldByNumber
#endif

#ifndef mxGetField_730
# define mxGetField_730 mxGetField
#endif

#ifndef mxCreateStructMatrix_730
# define mxCreateStructMatrix_730 mxCreateStructMatrix
#endif

#ifndef mxCreateCellMatrix_730
# define mxCreateCellMatrix_730 mxCreateCellMatrix
#endif

#ifndef mxCreateCharMatrixFromStrings_730
# define mxCreateCharMatrixFromStrings_730 mxCreateCharMatrixFromStrings
#endif

#ifndef mxGetString_730
# define mxGetString_730 mxGetString
#endif

#ifndef mxGetNumberOfDimensions_730
# define mxGetNumberOfDimensions_730 mxGetNumberOfDimensions
#endif

#ifndef mxGetDimensions_730
# define mxGetDimensions_730 mxGetDimensions
#endif

#ifndef mxSetDimensions_730
# define mxSetDimensions_730 mxSetDimensions
#endif

#ifndef mxSetIr_730
# define mxSetIr_730 mxSetIr
#endif

#ifndef mxGetIr_730
# define mxGetIr_730 mxGetIr
#endif

#ifndef mxSetJc_730
# define mxSetJc_730 mxSetJc
#endif

#ifndef mxGetJc_730
# define mxGetJc_730 mxGetJc
#endif

#ifndef mxCreateStructArray_730
# define mxCreateStructArray_730 mxCreateStructArray
#endif

#ifndef mxCreateCharArray_730
# define mxCreateCharArray_730 mxCreateCharArray
#endif

#ifndef mxCreateNumericArray_730
# define mxCreateNumericArray_730 mxCreateNumericArray
#endif

#ifndef mxCreateCellArray_730
# define mxCreateCellArray_730 mxCreateCellArray
#endif

#ifndef mxCreateLogicalArray_730
# define mxCreateLogicalArray_730 mxCreateLogicalArray
#endif

#ifndef mxGetCell_730
# define mxGetCell_730 mxGetCell
#endif

#ifndef mxSetCell_730
# define mxSetCell_730 mxSetCell
#endif

#ifndef mxSetNzmax_730
# define mxSetNzmax_730 mxSetNzmax
#endif

#ifndef mxSetN_730
# define mxSetN_730 mxSetN
#endif

#ifndef mxSetM_730
# define mxSetM_730 mxSetM
#endif

#ifndef mxGetNzmax_730
# define mxGetNzmax_730 mxGetNzmax
#endif

#ifndef mxCreateDoubleMatrix_730
# define mxCreateDoubleMatrix_730 mxCreateDoubleMatrix
#endif

#ifndef mxCreateNumericMatrix_730
# define mxCreateNumericMatrix_730 mxCreateNumericMatrix
#endif

#ifndef mxCreateLogicalMatrix_730
# define mxCreateLogicalMatrix_730 mxCreateLogicalMatrix
#endif

#ifndef mxCreateSparse_730
# define mxCreateSparse_730 mxCreateSparse
#endif

#ifndef mxCreateSparseLogicalMatrix_730
# define mxCreateSparseLogicalMatrix_730 mxCreateSparseLogicalMatrix
#endif

#ifndef mxGetNChars_730
# define mxGetNChars_730 mxGetNChars
#endif

#ifndef mxCreateStringFromNChars_730
# define mxCreateStringFromNChars_730 mxCreateStringFromNChars
#endif

#ifndef mxCalcSingleSubscript_730
# define mxCalcSingleSubscript_730 mxCalcSingleSubscript
#endif

#ifndef mxGetDimensions_fcn_730
# define mxGetDimensions_fcn_730 mxGetDimensions
#endif

#ifdef __cplusplus
}
#endif

#endif /* !defined(MX_COMPAT_32) && !defined(BUILDING_PUBLISHED_API_CPP) && defined(__linux__) */


#ifndef MATHWORKS_MATRIX_DETAIL_PUBLISHED_FWD_DECLS_HPP
#define MATHWORKS_MATRIX_DETAIL_PUBLISHED_FWD_DECLS_HPP



/*
 * Forward declaration for mxArray
 */
typedef struct mxArray_tag mxArray;

/*
 * Type representing the signature for MEX functions.
 */
typedef void (*mxFunctionPtr) (int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/*
 * Maximum mxArray name length
 */
#define mxMAXNAM TMW_NAME_LENGTH_MAX

/*
 * Logical type
 */
typedef bool mxLogical;

/*
 * Typedef required for Unicode support in MATLAB
 */
typedef CHAR16_T mxChar;

/*
 * Enumeration corresponding to all the valid mxArray types.
 */
typedef enum
{
    mxUNKNOWN_CLASS = 0,
    mxCELL_CLASS,
    mxSTRUCT_CLASS,
    mxLOGICAL_CLASS,
    mxCHAR_CLASS,
    mxVOID_CLASS,
    mxDOUBLE_CLASS,
    mxSINGLE_CLASS,
    mxINT8_CLASS,
    mxUINT8_CLASS,
    mxINT16_CLASS,
    mxUINT16_CLASS,
    mxINT32_CLASS,
    mxUINT32_CLASS,
    mxINT64_CLASS,
    mxUINT64_CLASS,
    mxFUNCTION_CLASS,
    mxOPAQUE_CLASS,
    mxOBJECT_CLASS, /* keep the last real item in the list */
#if defined(_LP64) || defined(_WIN64)
    mxINDEX_CLASS = mxUINT64_CLASS,
#else
    mxINDEX_CLASS = mxUINT32_CLASS,
#endif
    /* TEMPORARY AND NASTY HACK UNTIL mxSPARSE_CLASS IS COMPLETELY ELIMINATED */
    mxSPARSE_CLASS = mxVOID_CLASS /* OBSOLETE! DO NOT USE */
}
mxClassID;

/*
 * Indicates whether floating-point mxArrays are real or complex.
 */
typedef enum
{
    mxREAL,
    mxCOMPLEX
}
mxComplexity;

#endif /* MATHWORKS_MATRIX_DETAIL_PUBLISHED_FWD_DECLS_HPP */

/*
 * allocate memory, notifying registered listener
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void *mxMalloc(
    size_t	n		/* number of bytes */
    );


/*
 * allocate cleared memory, notifying registered listener.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void *mxCalloc(
    size_t	n,	/* number of objects */
    size_t	size	/* size of objects */
    );


/*
 * free memory, notifying registered listener.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxFree(void *ptr)	/* pointer to memory to be freed */;


/*
 * reallocate memory, notifying registered listener.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void *mxRealloc(void *ptr, size_t size);

/*
 * Get number of dimensions in array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C size_t mxGetNumberOfDimensions_730(const mxArray *pa);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxGetNumberOfDimensions_700(const mxArray *pa);


/*
 * Get pointer to dimension array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C const size_t *mxGetDimensions_730(const mxArray *pa);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C const int *mxGetDimensions_700(const mxArray *pa);
/* 
 * Get row dimension
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C size_t mxGetM(const mxArray *pa);

/*
 * Get row data pointer for sparse numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C size_t *mxGetIr_730(const mxArray *pa);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int *mxGetIr_700(const mxArray *pa);


/*
 * Get column data pointer for sparse numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C size_t *mxGetJc_730(const mxArray *pa);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int *mxGetJc_700(const mxArray *pa);

/*
 * Get maximum nonzero elements for sparse numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C size_t mxGetNzmax_730(const mxArray *pa);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxGetNzmax_700(const mxArray *pa);


/*
 * Set maximum nonzero elements for numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetNzmax_730(mxArray *pa, size_t nzmax);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetNzmax_700(mxArray *pa, int nzmax);

/*
 * Return pointer to the nth field name
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C const char *mxGetFieldNameByNumber(const mxArray *pa, int n);

/*
 * Return a pointer to the contents of the named field for 
 * the ith element (zero based).
 */ 
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxGetFieldByNumber_730(const mxArray *pa, size_t i, int fieldnum);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxGetFieldByNumber_700(const mxArray *pa, int i, int fieldnum);

/*
 * Get a pointer to the specified cell element. 
 */ 
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxGetCell_730(const mxArray *pa, size_t i);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxGetCell_700(const mxArray *pa, int i);

/*
 * Return the class (catergory) of data that the array holds.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxClassID mxGetClassID(const mxArray *pa);

/*
 * Get pointer to data
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void *mxGetData(
    const mxArray *pa		/* pointer to array */
    );

/*
 * Set pointer to data
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetData(
    mxArray *pa,		/* pointer to array */
    void  *newdata		/* pointer to data */
    );

/* 
 * Determine whether the specified array contains numeric (as opposed 
 * to cell or struct) data.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsNumeric(const mxArray *pa);

/* 
 * Determine whether the given array is a cell array.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsCell(const mxArray *pa);

/*
 * Determine whether the given array's logical flag is on.
 */ 
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C  bool mxIsLogical(const mxArray *pa);

/*
 * Determine whether the given array's scalar flag is on.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C  bool mxIsScalar(const mxArray *pa);

/*  
 * Determine whether the given array contains character data. 
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsChar(const mxArray *pa);

/*
 * Determine whether the given array is a structure array.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsStruct(const mxArray *pa);


/*
 * Determine whether the given array is an opaque array.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsOpaque(const mxArray *pa);


/*
 * Returns true if specified array is a function object.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsFunctionHandle(const mxArray *pa);


/*
 * This function is deprecated and is preserved only for backward compatibility.
 * DO NOT USE if possible.
 * Is array user defined MATLAB v5 object
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsObject(
    const mxArray *pa		/* pointer to array */
    );


/*
 * Get imaginary data pointer for numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void *mxGetImagData(
    const mxArray *pa		/* pointer to array */
    );

/*
 * Set imaginary data pointer for numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetImagData(
    mxArray *pa,		/* pointer to array */
    void    *newdata		/* imaginary data array pointer */
    );

/*
 * Determine whether the given array contains complex data.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsComplex(const mxArray *pa);


/*
 * Determine whether the given array is a sparse (as opposed to full). 
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsSparse(const mxArray *pa);

/*
 * Determine whether the specified array represents its data as 
 * double-precision floating-point numbers.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsDouble(const mxArray *pa);

/*
 * Determine whether the specified array represents its data as 
 * single-precision floating-point numbers.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsSingle(const mxArray *pa);


/*
 * Determine whether the specified array represents its data as 
 * signed 8-bit integers.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsInt8(const mxArray *pa);


/*
 * Determine whether the specified array represents its data as 
 * unsigned 8-bit integers.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsUint8(const mxArray *pa);


/*
 * Determine whether the specified array represents its data as 
 * signed 16-bit integers.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsInt16(const mxArray *pa);


/*
 * Determine whether the specified array represents its data as 
 * unsigned 16-bit integers.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsUint16(const mxArray *pa);


/*
 * Determine whether the specified array represents its data as 
 * signed 32-bit integers.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsInt32(const mxArray *pa);


/*
 * Determine whether the specified array represents its data as 
 * unsigned 32-bit integers.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsUint32(const mxArray *pa);


/*
 * Determine whether the specified array represents its data as 
 * signed 64-bit integers.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsInt64(const mxArray *pa);


/*
 * Determine whether the specified array represents its data as 
 * unsigned 64-bit integers.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsUint64(const mxArray *pa);

/* 
 * Get number of elements in array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C size_t mxGetNumberOfElements(	
    const mxArray *pa		/* pointer to array */
    );

/*
 * Get real data pointer for numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C double *mxGetPr(
    const mxArray *pa		/* pointer to array */
    );

/*
 * Set real data pointer for numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetPr(
    mxArray *pa,		/* pointer to array */
    double  *pr			/* real data array pointer */
    );

/*
 * Get imaginary data pointer for numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C double *mxGetPi(
    const mxArray *pa		/* pointer to array */
    );

/*
 * Set imaginary data pointer for numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetPi(
    mxArray *pa,		/* pointer to array */
    double  *pi			/* imaginary data array pointer */
    );

/*
 * Get string array data
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxChar *mxGetChars(
    const mxArray *pa		/* pointer to array */
    );


/*
 * Get 8 bits of user data stored in the mxArray header.  NOTE: This state
 * of these bits are not guaranteed to be preserved after API function
 * calls.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxGetUserBits(
    const mxArray	*pa		/* pointer to array */
    );


/*
 * Set 8 bits of user data stored in the mxArray header. NOTE: This state
 * of these bits are not guaranteed to be preserved after API function
 * calls.
 */ 
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetUserBits(
	mxArray	*pa,		/* pointer to array */
	int	value
    );


/*
 * Get the real component of the specified array's first data element.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C double mxGetScalar(const mxArray *pa);

/*
 * Inform Watcom compilers that scalar double return values
 * will be in the FPU register.
 */
#ifdef __WATCOMC__
#pragma aux mxGetScalar value [8087];
#endif


/*
 * Is the isFromGlobalWorkspace bit set?
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsFromGlobalWS(const mxArray *pa);


/*
 * Set the isFromGlobalWorkspace bit.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetFromGlobalWS(mxArray *pa, bool global);


/* 
 * Set row dimension
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetM_730(mxArray *pa, size_t m);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetM_700(mxArray *pa, int m);


/* 
 * Get column dimension
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C size_t mxGetN(const mxArray *pa);



/*
 * Is array empty
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsEmpty(
    const mxArray *pa		/* pointer to array */
    );
/*
 * Get the index to the named field.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxGetFieldNumber(const mxArray *pa, const char *name);

/*
 * Set row data pointer for numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetIr_730(mxArray *pa, size_t *newir);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetIr_700(mxArray *pa, int *newir);

/*
 * Set column data pointer for numeric array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetJc_730(mxArray *pa, size_t *newjc);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetJc_700(mxArray *pa, int *newjc);

/*
 * Get array data element size
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C size_t mxGetElementSize(const mxArray *pa);

/* 
 * Return the offset (in number of elements) from the beginning of 
 * the array to a given subscript.  
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C size_t mxCalcSingleSubscript_730(const mxArray *pa, size_t nsubs, const size_t *subs);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxCalcSingleSubscript_700(const mxArray *pa, int nsubs, const int *subs);

/*
 * Get number of structure fields in array
 * Returns 0 if mxArray is non-struct.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxGetNumberOfFields(
    const mxArray *pa		/* pointer to array */
    );

/*
 * Set an element in a cell array to the specified value.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetCell_730(mxArray *pa, size_t i, mxArray *value);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetCell_700(mxArray *pa, int i, mxArray *value);

/*
 * Set pa[i][fieldnum] = value 
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetFieldByNumber_730(mxArray *pa, size_t i, int fieldnum, mxArray *value);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetFieldByNumber_700(mxArray *pa, int i, int fieldnum, mxArray *value);

/*
 * Return a pointer to the contents of the named field for the ith 
 * element (zero based).  Returns NULL on no such field or if the
 * field itself is NULL
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxGetField_730(const mxArray *pa, size_t i, const char *fieldname);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxGetField_700(const mxArray *pa, int i, const char *fieldname);


/*
 * Sets the contents of the named field for the ith element (zero based).  
 * The input 'value' is stored in the input array 'pa' - no copy is made.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetField_730(mxArray *pa, size_t i, const char *fieldname, mxArray *value);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetField_700(mxArray *pa, int i, const char *fieldname, mxArray *value);

 
/*
 * mxGetProperty returns the value of a property for a given object and index.
 * The property must be public.
 *
 * If the given property name doesn't exist, isn't public, or the object isn't
 * the right type, then mxGetProperty returns NULL.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxGetProperty_730(const mxArray *pa, const size_t i, const char *propname);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxGetProperty_700(const mxArray *pa, const int i, const char *propname);

 
/*
 * mxSetProperty sets the value of a property for a given object and index.
 * The property must be public.
 *
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetProperty_730(mxArray *pa, size_t i, const char *propname, const mxArray *value);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetProperty_700(mxArray *pa, int i, const char *propname, const mxArray *value);

 
/* 
 * Return the name of an array's class.  
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C const char *mxGetClassName(const mxArray *pa);


/*
 * Determine whether an array is a member of the specified class. 
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsClass(const mxArray *pa, const char *name);

/*
 * Create a numeric matrix and initialize all its data elements to 0.
 * In standalone mode, out-of-memory will mean a NULL pointer is returned.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateNumericMatrix_730(size_t m, size_t n, mxClassID classid, mxComplexity flag);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateNumericMatrix_700(int m, int n, mxClassID classid, mxComplexity flag);

/*
 * Create an uninitialized numeric matrix.
 * The resulting array must be freed with mxDestroyArray.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateUninitNumericMatrix(size_t m, size_t n, mxClassID classid, mxComplexity flag);

/*
 * Create an uninitialized numeric array.
 * The resulting array must be freed with mxDestroyArray.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateUninitNumericArray (size_t ndim, size_t *dims, mxClassID classid, mxComplexity flag);

/* 
 * Set column dimension
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetN_730(mxArray *pa, size_t n);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxSetN_700(mxArray *pa, int n);


/*
 * Set dimension array and number of dimensions.  Returns 0 on success and 1
 * if there was not enough memory available to reallocate the dimensions array.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxSetDimensions_730(mxArray *pa, const size_t *pdims, size_t ndims);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxSetDimensions_700(mxArray *pa, const int *pdims, int ndims);

/*
 * mxArray destructor
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxDestroyArray(mxArray *pa);

/*
 * Create a numeric array and initialize all its data elements to 0.
 *
 * Similar to mxCreateNumericMatrix, in a standalone application, 
 * out-of-memory will mean a NULL pointer is returned.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateNumericArray_730(size_t ndim, const size_t *dims, mxClassID classid, mxComplexity flag);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateNumericArray_700(int ndim, const int *dims, mxClassID classid, mxComplexity flag);

/*
 * Create an N-Dimensional array to hold string data;
 * initialize all elements to 0.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateCharArray_730(size_t ndim, const size_t *dims);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateCharArray_700(int ndim, const int *dims);

/*
 * Create a two-dimensional array to hold double-precision
 * floating-point data; initialize each data element to 0.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateDoubleMatrix_730(size_t m, size_t n, mxComplexity flag);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateDoubleMatrix_700(int m, int n, mxComplexity flag);


/*
 * Get a properly typed pointer to the elements of a logical array.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxLogical *mxGetLogicals(const mxArray *pa);


/*
 * Create a logical array and initialize its data elements to false.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateLogicalArray_730(size_t ndim, const size_t *dims);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateLogicalArray_700(int ndim, const int *dims);

/*
 * Create a two-dimensional array to hold logical data and
 * initializes each data element to false.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateLogicalMatrix_730(size_t m, size_t n);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateLogicalMatrix_700(int m, int n);

/*
 * Create a logical scalar mxArray having the specified value.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateLogicalScalar(bool value);


/*
 * Returns true if we have a valid logical scalar mxArray.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsLogicalScalar(const mxArray *pa);


/*
 * Returns true if the logical scalar value is true.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsLogicalScalarTrue(const mxArray *pa);


/*
 * Create a double-precision scalar mxArray initialized to the
 * value specified
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateDoubleScalar(double value);


/*
 * Create a 2-Dimensional sparse array.
 *
 * Z = mxCreateSparse(m,n,nzmax,cmplx_flag);
 * An m-by-n, real or complex, sparse matrix with room for nzmax nonzeros.
 * Use this to allocate storage for a sparse matrix.
 * It allocates the structure, allocates the pr, pi, ir and jc arrays,
 * and sets all the fields, which may be changed later.
 * Avoids the question of malloc(0) by replacing nzmax = 0 with nzmax = 1.
 * Also sets Z->pr[0] = 0.0 so that the scalar sparse(0.0) acts like 0.0.
 *
 * Notice that the value of m is almost irrelevant.  It is only stored in
 * the mxSetM field of the matrix structure.  It does not affect the amount
 * of storage required by sparse matrices, or the amount of time required
 * by sparse algorithms.  Consequently, m can be "infinite".  The result
 * is a semi-infinite matrix with a finite number of columns and a finite,
 * but unspecified, number of nonzero rows.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateSparse_730(size_t m, size_t n, size_t nzmax, mxComplexity flag);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateSparse_700(int m, int n, int nzmax, mxComplexity flag);


/*
 * Create a 2-D sparse logical array
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateSparseLogicalMatrix_730(size_t m, size_t n, size_t nzmax);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateSparseLogicalMatrix_700(int m, int n, int nzmax);


/*
 * Copies characters from a MATLAB array to a char array
 * This function will attempt to perform null termination if it is possible.
 * nChars is the number of bytes in the output buffer
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxGetNChars_730(const mxArray *pa, char *buf, size_t nChars);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxGetNChars_700(const mxArray *pa, char *buf, int nChars);


/*
 * Converts a string array to a C-style string. The C-style string is in the
 * local codepage encoding. If the conversion for the entire Unicode string
 * cannot fit into the supplied character buffer, then the conversion includes
 * the last whole codepoint that will fit into the buffer. The string is thus
 * truncated at the greatest possible whole codepoint and does not split code-
 * points.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxGetString_730(const mxArray *pa, char *buf, size_t buflen);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxGetString_700(const mxArray *pa, char *buf, int buflen);


/*
 * Create a NULL terminated C string from an mxArray of type mxCHAR_CLASS
 * Supports multibyte character sets.  The resulting string must be freed
 * with mxFree.  Returns NULL on out of memory or non-character arrays.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C char *mxArrayToString(const mxArray *pa);

/*
 * Create a NULL terminated C string from an mxArray of type mxCHAR_CLASS
 * The C style string is in UTF-8 encoding. The resulting 
 * string must be freed with mxFree. Returns NULL on out of memory or
 * non-character arrays.
 */

LIBMMWMATRIX_PUBLISHED_API_EXTERN_C char *mxArrayToUTF8String(mxArray const *pa);


/**
 * Create a 1-by-n string array initialized to str. The supplied string is
 * presumed to be in the local codepage encoding. The character data format
 * in the mxArray will be UTF-16.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateStringFromNChars_730(const char *str, size_t n);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateStringFromNChars_700(const char *str, int n);


/*
 * Create a 1-by-n string array initialized to null terminated string
 * where n is the length of the string.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateString(const char *str);

/*
 * Create a string array initialized to the strings in str.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateCharMatrixFromStrings_730(size_t m, const char **str);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateCharMatrixFromStrings_700(int m, const char **str);

/*
 * Create a 2-Dimensional cell array, with each cell initialized
 * to NULL.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateCellMatrix_730(size_t m, size_t n);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateCellMatrix_700(int m, int n);


/*
 * Create an N-Dimensional cell array, with each cell initialized
 * to NULL.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateCellArray_730(size_t ndim, const size_t *dims);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateCellArray_700(int ndim, const int *dims);

/*
 * Create a 2-Dimensional structure array having the specified fields;
 * initialize all values to NULL.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateStructMatrix_730(size_t m, size_t n, int nfields, const char **fieldnames);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateStructMatrix_700(int m, int n, int nfields, const char **fieldnames);


/*
 * Create an N-Dimensional structure array having the specified fields;
 * initialize all values to NULL.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateStructArray_730(size_t ndim, const size_t *dims, int nfields,
			     const char **fieldnames);
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxCreateStructArray_700(int ndim, const int *dims, int nfields,
                 const char **fieldnames);

/*
 * Make a deep copy of an array, return a pointer to the copy.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C mxArray *mxDuplicateArray(const mxArray *in);


/*
 * Set classname of an unvalidated object array.  It is illegal to
 * call this function on a previously validated object array.
 * Return 0 for success, 1 for failure.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxSetClassName(mxArray *pa, const char *classname);


/* 
 * Add a field to a structure array. Returns field number on success or -1
 * if inputs are invalid or an out of memory condition occurs.
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C int mxAddField(mxArray *pa, const char *fieldname);


/*
 * Remove a field from a structure array.  Does nothing if no such field exists.
 * Does not destroy the field itself.
*/
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mxRemoveField(mxArray *pa, int field);


/*
 * Function for obtaining MATLAB's concept of EPS
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C double mxGetEps(void);


/*
 * Function for obtaining MATLAB's concept of INF (Used in MEX-File callback).
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C double mxGetInf(void);


/*
 * Function for obtaining MATLAB's concept of NaN (Used in MEX-File callback).
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C double mxGetNaN(void);


/*
 * Inform Watcom compilers that scalar double return values
 * will be in the FPU register.
 */
#ifdef __WATCOMC__
#pragma aux mxGetEps value [8087];
#pragma aux mxGetInf value [8087];
#pragma aux mxGetNaN value [8087];
#endif


/*
 * test for finiteness in a machine-independent manner
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsFinite(
    double x                  /* value to test */
    );


/*
 * test for infinity in a machine-independent manner
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsInf(
    double x                  /* value to test */
    );


/*
 * test for NaN in a machine-independent manner
 */
LIBMMWMATRIX_PUBLISHED_API_EXTERN_C bool mxIsNaN(
    double x                  /* value to test */
    );


#ifndef mxassert_h
#define mxassert_h
/*
mxAssert(int expression, char *error_message)
---------------------------------------------

  Similar to ANSI C's assert() macro, the mxAssert macro checks the
  value of an assertion, continuing execution only if the assertion
  holds.  If 'expression' evaluates to be true, then the mxAssert does
  nothing.  If, however, 'expression' is false, then mxAssert prints an
  error message to the MATLAB Command Window, consisting of the failed
  assertion's expression, the file name and line number where the failed
  assertion occurred, and the string 'error_message'.  'error_message'
  allows the user to specify a more understandable description of why
  the assertion failed.  (Use an empty string if no extra description
  should follow the failed assertion message.)  After a failed
  assertion, control returns to the MATLAB command line. 

  mxAssertS, (the S for Simple), takes the same inputs as mxAssert.  It 
  does not print the text of the failed assertion, only the file and 
  line where the assertion failed, and the explanatory error_message.

  Note that script MEX will turn off these assertions when building
  optimized MEX-functions, so they should be used for debugging 
  purposes only.
*/

#ifdef MATLAB_MEX_FILE
#  ifndef NDEBUG

LIBMMWMATRIX_PUBLISHED_API_EXTERN_C void mexPrintAssertion(const char *test, 
		       const char *fname, 
		       int linenum, 
		       const char *message);


#    define mxAssert(test, message) ( (test) ? (void) 0 : mexPrintAssertion(#test, __FILE__, __LINE__, message))
#    define mxAssertS(test, message) ( (test) ? (void) 0 : mexPrintAssertion("", __FILE__, __LINE__, message))
#  else
#    define mxAssert(test, message) ((void) 0)
#    define mxAssertS(test, message) ((void) 0)
#  endif
#else
#  include <assert.h>
#  define mxAssert(test, message) assert(test)
#  define mxAssertS(test, message) assert(test)
#endif

#endif /* mxassert_h */

#if !defined(BUILDING_PUBLISHED_API_CPP) && !defined(BUILDING_LIBMX)

/*
 * PUBLISHED APIs with changes in MATLAB 7.3
 */

#if !defined(MX_COMPAT_32) && !defined(__linux__)

#ifdef __cplusplus
extern "C" {
#endif

#ifndef mxSetProperty
# define mxSetProperty mxSetProperty_730
#endif

#ifndef mxGetProperty
# define mxGetProperty mxGetProperty_730
#endif

#ifndef mxSetField
# define mxSetField mxSetField_730
#endif

#ifndef mxSetFieldByNumber
# define mxSetFieldByNumber mxSetFieldByNumber_730
#endif

#ifndef mxGetFieldByNumber
# define mxGetFieldByNumber mxGetFieldByNumber_730
#endif

#ifndef mxGetField
# define mxGetField mxGetField_730
#endif

#ifndef mxCreateStructMatrix
# define mxCreateStructMatrix mxCreateStructMatrix_730
#endif

#ifndef mxCreateCellMatrix
# define mxCreateCellMatrix mxCreateCellMatrix_730
#endif

#ifndef mxCreateCharMatrixFromStrings
# define mxCreateCharMatrixFromStrings mxCreateCharMatrixFromStrings_730
#endif

#ifndef mxGetString
# define mxGetString mxGetString_730
#endif

#ifndef mxGetNumberOfDimensions
# define mxGetNumberOfDimensions mxGetNumberOfDimensions_730
#endif

#ifndef mxGetDimensions
# define mxGetDimensions mxGetDimensions_730
#endif

#ifndef mxSetDimensions
# define mxSetDimensions mxSetDimensions_730
#endif

#ifndef mxSetIr
# define mxSetIr mxSetIr_730
#endif

#ifndef mxGetIr
# define mxGetIr mxGetIr_730
#endif

#ifndef mxSetJc
# define mxSetJc mxSetJc_730
#endif

#ifndef mxGetJc
# define mxGetJc mxGetJc_730
#endif

#ifndef mxCreateStructArray
# define mxCreateStructArray mxCreateStructArray_730
#endif

#ifndef mxCreateCharArray
# define mxCreateCharArray mxCreateCharArray_730
#endif

#ifndef mxCreateNumericArray
# define mxCreateNumericArray mxCreateNumericArray_730
#endif

#ifndef mxCreateCellArray
# define mxCreateCellArray mxCreateCellArray_730
#endif

#ifndef mxCreateLogicalArray
# define mxCreateLogicalArray mxCreateLogicalArray_730
#endif

#ifndef mxGetCell
# define mxGetCell mxGetCell_730
#endif

#ifndef mxSetCell
# define mxSetCell mxSetCell_730
#endif

#ifndef mxSetNzmax
# define mxSetNzmax mxSetNzmax_730
#endif

#ifndef mxSetN
# define mxSetN mxSetN_730
#endif

#ifndef mxSetM
# define mxSetM mxSetM_730
#endif

#ifndef mxGetNzmax
# define mxGetNzmax mxGetNzmax_730
#endif

#ifndef mxCreateDoubleMatrix
# define mxCreateDoubleMatrix mxCreateDoubleMatrix_730
#endif

#ifndef mxCreateNumericMatrix
# define mxCreateNumericMatrix mxCreateNumericMatrix_730
#endif

#ifndef mxCreateLogicalMatrix
# define mxCreateLogicalMatrix mxCreateLogicalMatrix_730
#endif

#ifndef mxCreateSparse
# define mxCreateSparse mxCreateSparse_730
#endif

#ifndef mxCreateSparseLogicalMatrix
# define mxCreateSparseLogicalMatrix mxCreateSparseLogicalMatrix_730
#endif

#ifndef mxGetNChars
# define mxGetNChars mxGetNChars_730
#endif

#ifndef mxCreateStringFromNChars
# define mxCreateStringFromNChars mxCreateStringFromNChars_730
#endif

#ifndef mxCalcSingleSubscript
# define mxCalcSingleSubscript mxCalcSingleSubscript_730
#endif

#ifndef mxGetDimensions_fcn
# define mxGetDimensions_fcn mxGetDimensions_730
#endif

#ifdef __cplusplus
}
#endif

#endif /* !MX_COMPAT_32 */


#ifdef MX_COMPAT_32

/*
 * 32-bit compatibility APIs
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef mxGetNumberOfDimensions
#define mxGetNumberOfDimensions mxGetNumberOfDimensions_700
#endif

#ifndef mxGetDimensions
#define mxGetDimensions mxGetDimensions_700
#endif

#ifndef mxGetDimensions_fcn
# define mxGetDimensions_fcn mxGetDimensions_700
#endif

#ifndef mxGetIr
#define mxGetIr mxGetIr_700
#endif

#ifndef mxGetJc
#define mxGetJc mxGetJc_700
#endif

#ifndef mxGetCell
#define mxGetCell mxGetCell_700
#endif

#ifndef mxGetNzmax
#define mxGetNzmax mxGetNzmax_700
#endif

#ifndef mxSetNzmax
#define mxSetNzmax mxSetNzmax_700
#endif

#ifndef mxGetFieldByNumber
#define mxGetFieldByNumber mxGetFieldByNumber_700
#endif

#ifndef mxSetProperty
#define mxSetProperty mxSetProperty_700
#endif

#ifndef mxGetProperty
#define mxGetProperty mxGetProperty_700
#endif

#ifndef mxSetField
#define mxSetField mxSetField_700
#endif

#ifndef mxSetFieldByNumber
#define mxSetFieldByNumber mxSetFieldByNumber_700
#endif

#ifndef mxGetField
#define mxGetField mxGetField_700
#endif

#ifndef mxCreateStructMatrix
#define mxCreateStructMatrix mxCreateStructMatrix_700
#endif

#ifndef mxCreateCellMatrix
#define mxCreateCellMatrix mxCreateCellMatrix_700
#endif

#ifndef mxCreateCharMatrixFromStrings
#define mxCreateCharMatrixFromStrings mxCreateCharMatrixFromStrings_700
#endif

#ifndef mxGetString
#define mxGetString mxGetString_700
#endif

#ifndef mxSetDimensions
#define mxSetDimensions mxSetDimensions_700
#endif

#ifndef mxSetIr
#define mxSetIr mxSetIr_700
#endif

#ifndef mxSetJc
#define mxSetJc mxSetJc_700
#endif

#ifndef mxCreateStructArray
#define mxCreateStructArray mxCreateStructArray_700
#endif

#ifndef mxCreateCharArray
#define mxCreateCharArray mxCreateCharArray_700
#endif

#ifndef mxCreateNumericArray
#define mxCreateNumericArray mxCreateNumericArray_700
#endif

#ifndef mxCreateCellArray
#define mxCreateCellArray mxCreateCellArray_700
#endif

#ifndef mxCreateLogicalArray
#define mxCreateLogicalArray mxCreateLogicalArray_700
#endif

#ifndef mxSetCell
#define mxSetCell mxSetCell_700
#endif

#ifndef mxSetN
#define mxSetN mxSetN_700
#endif

#ifndef mxSetM
#define mxSetM mxSetM_700
#endif

#ifndef mxCreateDoubleMatrix
#define mxCreateDoubleMatrix mxCreateDoubleMatrix_700
#endif

#ifndef mxCreateNumericMatrix
#define mxCreateNumericMatrix mxCreateNumericMatrix_700
#endif

#ifndef mxCreateLogicalMatrix
#define mxCreateLogicalMatrix mxCreateLogicalMatrix_700
#endif

#ifndef mxCreateSparse
#define mxCreateSparse mxCreateSparse_700
#endif

#ifndef mxCreateSparseLogicalMatrix
#define mxCreateSparseLogicalMatrix mxCreateSparseLogicalMatrix_700
#endif

#ifndef mxGetNChars
#define mxGetNChars mxGetNChars_700
#endif

#ifndef mxCreateStringFromNChars
#define mxCreateStringFromNChars mxCreateStringFromNChars_700
#endif

#ifndef mxCalcSingleSubscript
#define mxCalcSingleSubscript mxCalcSingleSubscript_700
#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifdef MX_COMPAT_32 */
#endif /* !defined(BUILDING_LIBMX) && !defined(PUBLISHED_API_CPP) */
#endif /* matrix_h */
#endif /* MATRIX_DEVELOPER_API_HPP */
