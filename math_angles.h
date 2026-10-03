/*
 * math_angles.h
 *
 *  Created on: 1 θών. 2022 γ.
 *      Author: 1
 */

#ifndef MATH_ANGLES_H_
#define MATH_ANGLES_H_

struct Vec vctr_summ(struct Vec a, struct Vec b);
struct Vec vctr_diff(struct Vec a, struct Vec b);
struct Vec vctr_mltp(struct Vec a,  struct Vec b);
struct Vec vctr_mltp_n(float a, struct Vec b);
struct MATRIX_3_3 mtrx_1(struct MATRIX_3_3 m);
struct MATRIX_3_3 mtrx_mltp(struct MATRIX_3_3 m, struct MATRIX_3_3 m1 );
struct MATRIX_3_3 mtrx_dv(struct MATRIX_3_3 m, struct MATRIX_3_3 m1) ;
float modul(struct Vec a);
float vctr_cos(struct Vec a, struct Vec b);
struct Vec mtrx_vctr_mltp(struct MATRIX_3_3 m, struct Vec v) ;
float predict(float *abc, float *data, float S_x[5], int N);
void S_X(float S_x[5], int N);
struct Cal add_cal(struct Cal first, struct Cal second);
struct Vec callibrate(struct Vec Data , struct Cal calib);

#endif /* MATH_ANGLES_H_ */
