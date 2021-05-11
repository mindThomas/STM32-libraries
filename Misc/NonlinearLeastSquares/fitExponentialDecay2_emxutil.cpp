//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fitExponentialDecay2_emxutil.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 08-May-2020 23:23:34
//

// Include Files
#include "fitExponentialDecay2_emxutil.h"
#include "fitExponentialDecay2.h"
#include <MATLABCoderInit/rt_nonfinite.h>
#include <stdlib.h>
#include <string.h>

// Function Declarations
static void emxCopy_real32_T(emxArray_real32_T** dst, emxArray_real32_T* const* src);
static void emxFreeStruct_cell_wrap_1(cell_wrap_1* pStruct);
static void emxInitStruct_cell_wrap_1(cell_wrap_1* pStruct);

// Function Definitions

//
// Arguments    : emxArray_real32_T **dst
//                emxArray_real32_T * const *src
// Return Type  : void
//
static void emxCopy_real32_T(emxArray_real32_T** dst, emxArray_real32_T* const* src)
{
    int numElDst;
    int numElSrc;
    int i;
    numElDst = 1;
    numElSrc = 1;
    for (i = 0; i < (*dst)->numDimensions; i++) {
        numElDst *= (*dst)->size[i];
        numElSrc *= (*src)->size[i];
    }

    for (i = 0; i < (*dst)->numDimensions; i++) {
        (*dst)->size[i] = (*src)->size[i];
    }

    emxEnsureCapacity_real32_T(*dst, numElDst);
    for (i = 0; i < numElSrc; i++) {
        (*dst)->data[i] = (*src)->data[i];
    }
}

//
// Arguments    : cell_wrap_1 *pStruct
// Return Type  : void
//
static void emxFreeStruct_cell_wrap_1(cell_wrap_1* pStruct)
{
    emxFree_real32_T(&pStruct->f1);
}

//
// Arguments    : cell_wrap_1 *pStruct
// Return Type  : void
//
static void emxInitStruct_cell_wrap_1(cell_wrap_1* pStruct)
{
    emxInit_real32_T(&pStruct->f1, 1);
}

//
// Arguments    : cell_wrap_1 *dst
//                const cell_wrap_1 *src
// Return Type  : void
//
void emxCopyStruct_cell_wrap_1(cell_wrap_1* dst, const cell_wrap_1* src)
{
    emxCopy_real32_T(&dst->f1, &src->f1);
}

//
// Arguments    : emxArray_real32_T *emxArray
//                int oldNumel
// Return Type  : void
//
void emxEnsureCapacity_real32_T(emxArray_real32_T* emxArray, int oldNumel)
{
    int   newNumel;
    int   i;
    void* newData;
    if (oldNumel < 0) {
        oldNumel = 0;
    }

    newNumel = 1;
    for (i = 0; i < emxArray->numDimensions; i++) {
        newNumel *= emxArray->size[i];
    }

    if (newNumel > emxArray->allocatedSize) {
        i = emxArray->allocatedSize;
        if (i < 16) {
            i = 16;
        }

        while (i < newNumel) {
            if (i > 1073741823) {
                i = MAX_int32_T;
            } else {
                i <<= 1;
            }
        }

        newData = calloc((unsigned int)i, sizeof(float));
        if (emxArray->data != NULL) {
            memcpy(newData, (void*)emxArray->data, sizeof(float) * oldNumel);
            if (emxArray->canFreeData) {
                free((void*)emxArray->data);
            }
        }

        emxArray->data          = (float*)newData;
        emxArray->allocatedSize = i;
        emxArray->canFreeData   = true;
    }
}

//
// Arguments    : emxArray_real32_T *emxArray
//                int oldNumel
// Return Type  : void
//
void emxEnsureCapacity_real32_T1(emxArray_real32_T* emxArray, int oldNumel)
{
    int   newNumel;
    int   i;
    void* newData;
    if (oldNumel < 0) {
        oldNumel = 0;
    }

    newNumel = 1;
    for (i = 0; i < emxArray->numDimensions; i++) {
        newNumel *= emxArray->size[i];
    }

    if (newNumel > emxArray->allocatedSize) {
        i = emxArray->allocatedSize;
        if (i < 16) {
            i = 16;
        }

        while (i < newNumel) {
            if (i > 1073741823) {
                i = MAX_int32_T;
            } else {
                i <<= 1;
            }
        }

        newData = calloc((unsigned int)i, sizeof(float));
        if (emxArray->data != NULL) {
            memcpy(newData, (void*)emxArray->data, sizeof(float) * oldNumel);
            if (emxArray->canFreeData) {
                free((void*)emxArray->data);
            }
        }

        emxArray->data          = (float*)newData;
        emxArray->allocatedSize = i;
        emxArray->canFreeData   = true;
    }
}

//
// Arguments    : cell_wrap_1 pMatrix[2]
// Return Type  : void
//
void emxFreeMatrix_cell_wrap_1(cell_wrap_1 pMatrix[2])
{
    int i;
    for (i = 0; i < 2; i++) {
        emxFreeStruct_cell_wrap_1(&pMatrix[i]);
    }
}

//
// Arguments    : emxArray_real32_T **pEmxArray
// Return Type  : void
//
void emxFree_real32_T(emxArray_real32_T** pEmxArray)
{
    if (*pEmxArray != (emxArray_real32_T*)NULL) {
        if (((*pEmxArray)->data != (float*)NULL) && (*pEmxArray)->canFreeData) {
            free((void*)(*pEmxArray)->data);
        }

        free((void*)(*pEmxArray)->size);
        free((void*)*pEmxArray);
        *pEmxArray = (emxArray_real32_T*)NULL;
    }
}

//
// Arguments    : cell_wrap_1 pMatrix[2]
// Return Type  : void
//
void emxInitMatrix_cell_wrap_1(cell_wrap_1 pMatrix[2])
{
    int i;
    for (i = 0; i < 2; i++) {
        emxInitStruct_cell_wrap_1(&pMatrix[i]);
    }
}

//
// Arguments    : emxArray_real32_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
void emxInit_real32_T(emxArray_real32_T** pEmxArray, int numDimensions)
{
    emxArray_real32_T* emxArray;
    int                i;
    *pEmxArray              = (emxArray_real32_T*)malloc(sizeof(emxArray_real32_T));
    emxArray                = *pEmxArray;
    emxArray->data          = (float*)NULL;
    emxArray->numDimensions = numDimensions;
    emxArray->size          = (int*)malloc(sizeof(int) * numDimensions);
    emxArray->allocatedSize = 0;
    emxArray->canFreeData   = true;
    for (i = 0; i < numDimensions; i++) {
        emxArray->size[i] = 0;
    }
}

//
// Arguments    : emxArray_real32_T **pEmxArray
//                int numDimensions
// Return Type  : void
//
void emxInit_real32_T1(emxArray_real32_T** pEmxArray, int numDimensions)
{
    emxArray_real32_T* emxArray;
    int                i;
    *pEmxArray              = (emxArray_real32_T*)malloc(sizeof(emxArray_real32_T));
    emxArray                = *pEmxArray;
    emxArray->data          = (float*)NULL;
    emxArray->numDimensions = numDimensions;
    emxArray->size          = (int*)malloc(sizeof(int) * numDimensions);
    emxArray->allocatedSize = 0;
    emxArray->canFreeData   = true;
    for (i = 0; i < numDimensions; i++) {
        emxArray->size[i] = 0;
    }
}

//
// File trailer for fitExponentialDecay2_emxutil.cpp
//
// [EOF]
//
