/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rt_nonfinite.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 19-Nov-2018 11:57:59
 */

#ifndef RT_NONFINITE_H
#define RT_NONFINITE_H
#if defined(_MSC_VER) && (_MSC_VER <= 1200)
#include <float.h>
#endif

#include <stddef.h>
#include <MATLABCoderInit/rtwtypes.h>

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
extern void rt_InitInfAndNaN(size_t realSize);
extern boolean_T rtIsInf(real_T value);
extern boolean_T rtIsInfF(real32_T value);
extern boolean_T rtIsNaN(real_T value);
extern boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

#endif

/*
 * File trailer for rt_nonfinite.h
 *
 * [EOF]
 */
