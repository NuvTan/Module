/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Qubic.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 12-May-2023 16:01:02
 */

#ifndef QUBIC_H
#define QUBIC_H

/* Include Files */
#include "Qubic_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void Qubic(double q_k1, double q_k2, double qdot_k1, double qdot_k2,
                  double tf, emxArray_real_T *q_position,
                  emxArray_real_T *q_velocity, emxArray_real_T *q_acc);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Qubic.h
 *
 * [EOF]
 */
