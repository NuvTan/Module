/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Qubic.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 12-May-2023 16:01:02
 */

/* Include Files */
#include "Qubic.h"
#include "Qubic_emxutil.h"
#include "Qubic_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }
  return y;
}

/*
 * Arguments    : double q_k1
 *                double q_k2
 *                double qdot_k1
 *                double qdot_k2
 *                double tf
 *                emxArray_real_T *q_position
 *                emxArray_real_T *q_velocity
 *                emxArray_real_T *q_acc
 * Return Type  : void
 */
void Qubic(double q_k1, double q_k2, double qdot_k1, double qdot_k2, double tf,
           emxArray_real_T *q_position, emxArray_real_T *q_velocity,
           emxArray_real_T *q_acc)
{
  double C2;
  double C2_tmp;
  double d;
  double delta1;
  double *q_acc_data;
  double *q_position_data;
  double *q_velocity_data;
  int i;
  int k;
  int q_acc_tmp_tmp;
  q_acc_data = q_acc->data;
  delta1 = tf * 2000.0;
  if (!(delta1 >= 0.0)) {
    q_acc->size[0] = 1;
    q_acc->size[1] = 0;
  } else {
    d = floor(delta1);
    i = q_acc->size[0] * q_acc->size[1];
    q_acc->size[0] = 1;
    q_acc->size[1] = (int)d;
    emxEnsureCapacity_real_T(q_acc, i);
    q_acc_data = q_acc->data;
    if ((int)d >= 1) {
      q_acc_tmp_tmp = (int)d - 1;
      q_acc_data[(int)floor(delta1) - 1] = tf;
      if (q_acc->size[1] >= 2) {
        q_acc_data[0] = 0.0;
        if (q_acc->size[1] >= 3) {
          if (-tf == 0.0) {
            delta1 = tf / ((double)q_acc->size[1] - 1.0);
            for (k = 2; k <= q_acc_tmp_tmp; k++) {
              q_acc_data[k - 1] =
                  (double)(((k << 1) - q_acc->size[1]) - 1) * delta1;
            }
            if ((q_acc->size[1] & 1) == 1) {
              q_acc_data[q_acc->size[1] >> 1] = 0.0;
            }
          } else if ((tf < 0.0) && (fabs(tf) > 8.9884656743115785E+307)) {
            delta1 = tf / ((double)q_acc->size[1] - 1.0);
            i = q_acc->size[1];
            for (k = 0; k <= i - 3; k++) {
              q_acc_data[k + 1] = delta1 * ((double)k + 1.0);
            }
          } else {
            delta1 = tf / ((double)q_acc->size[1] - 1.0);
            i = q_acc->size[1];
            for (k = 0; k <= i - 3; k++) {
              q_acc_data[k + 1] = ((double)k + 1.0) * delta1;
            }
          }
        }
      }
    }
  }
  delta1 = q_k2 - q_k1;
  C2_tmp = tf * tf;
  C2 = 3.0 * (delta1 / C2_tmp) + (-qdot_k2 - 2.0 * qdot_k1) / tf;
  delta1 =
      -2.0 * (delta1 / rt_powd_snf(tf, 3.0)) + (qdot_k2 + qdot_k1) / C2_tmp;
  i = q_velocity->size[0] * q_velocity->size[1];
  q_velocity->size[0] = 1;
  q_velocity->size[1] = q_acc->size[1];
  emxEnsureCapacity_real_T(q_velocity, i);
  q_velocity_data = q_velocity->data;
  q_acc_tmp_tmp = q_acc->size[1];
  i = q_position->size[0] * q_position->size[1];
  q_position->size[0] = 1;
  q_position->size[1] = q_acc->size[1];
  emxEnsureCapacity_real_T(q_position, i);
  q_position_data = q_position->data;
  for (i = 0; i < q_acc_tmp_tmp; i++) {
    d = q_acc_data[i];
    C2_tmp = d * d;
    q_velocity_data[i] = C2_tmp;
    q_position_data[i] =
        ((q_k1 + qdot_k1 * d) + C2 * C2_tmp) + delta1 * rt_powd_snf(d, 3.0);
  }
  i = q_velocity->size[0] * q_velocity->size[1];
  q_velocity->size[0] = 1;
  q_velocity->size[1] = q_acc->size[1];
  emxEnsureCapacity_real_T(q_velocity, i);
  q_velocity_data = q_velocity->data;
  d = 2.0 * C2;
  C2_tmp = 3.0 * delta1;
  q_acc_tmp_tmp = q_acc->size[1] - 1;
  for (i = 0; i <= q_acc_tmp_tmp; i++) {
    q_velocity_data[i] =
        (qdot_k1 + d * q_acc_data[i]) + C2_tmp * q_velocity_data[i];
  }
  i = q_acc->size[0] * q_acc->size[1];
  q_acc->size[0] = 1;
  emxEnsureCapacity_real_T(q_acc, i);
  q_acc_data = q_acc->data;
  C2_tmp = 6.0 * delta1;
  for (i = 0; i <= q_acc_tmp_tmp; i++) {
    q_acc_data[i] = d + C2_tmp * q_acc_data[i];
  }
}

/*
 * File trailer for Qubic.c
 *
 * [EOF]
 */
