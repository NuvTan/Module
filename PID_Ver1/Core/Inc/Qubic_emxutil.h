/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Qubic_emxutil.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 12-May-2023 16:01:02
 */

#ifndef QUBIC_EMXUTIL_H
#define QUBIC_EMXUTIL_H

/* Include Files */
#include "Qubic_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);

extern void emxFree_real_T(emxArray_real_T **pEmxArray);

extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Qubic_emxutil.h
 *
 * [EOF]
 */
