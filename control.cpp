#include "control.h"

/* Function Definitions */

/*
 * Arguments    : double dt
 *                double a
 *                double ob_p
 *                float *position
 *                float *velocity
 * Return Type  : void
 */
void Kalman::init(float flp,float flv){
  lp=flp;
  lv=flv;
}
void Kalman::fiter(double dt, double a, double ob_p,float *position, float *velocity)
{
  double F[4];
  double forcast_x[2];
  double k[2];
  int i0;
  double d0;
  double b_F[4];
  double b;
  F[0] = 1.0;
  F[2] = dt;
  F[1] = 0.0;
  F[3] = 1.0;
  forcast_x[0] = lp + dt * lv;
  forcast_x[1] = 0.0 * lp + lv;
  k[0] = 0.5 * (dt * dt) * a;
  k[1] = dt * a;
  for (i0 = 0; i0 < 2; i0++) {
    forcast_x[i0] += k[i0];
    d0 = F[i0 + 2];
    b_F[i0] = F[i0] * P[0] + d0 * P[1];
    b_F[i0 + 2] = F[i0] * P[2] + d0 * P[3];
  }
  for (i0 = 0; i0 < 2; i0++) {
    P[i0] = (b_F[i0] + b_F[i0 + 2] * dt) + Q[i0];
    P[i0 + 2] = (b_F[i0] * 0.0 + b_F[i0 + 2]) + Q[i0 + 2];
  }

  d0 = H[1] * P[3];
  b = ((H[0] * P[0] + H[1] * P[1]) * H[0] + (H[0] * P[2] + d0) * H[1]) + R;
  k[0] = (P[0] * H[0] + P[2] * H[1]) * b;
  k[1] = (P[1] * H[0] + d0) * b;
  b = ob_p - (H[0] * forcast_x[0] + H[1] * forcast_x[1]);
  for (i0 = 0; i0 < 2; i0++) {
    forcast_x[i0] += k[i0] * b;
    F[i0] = k[i0] * H[0];
    F[i0 + 2] = k[i0] * H[1];
    d0 = F[i0 + 2];
    b_F[i0] = P[i0] - (F[i0] * P[0] + d0 * P[1]);
    b_F[i0 + 2] = P[i0 + 2] - (F[i0] * P[2] + d0 * P[3]);
  }

  P[0] = b_F[0];
  P[1] = b_F[1];
  P[2] = b_F[2];
  P[3] = b_F[3];
  *position = forcast_x[0];
  *velocity = forcast_x[1];
  lv=*velocity;
  lp=*position;
}