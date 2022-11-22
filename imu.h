#ifnedf __IMU_H
#define __IMU_H
void imu_init();
void getdata();
void getquater();
void rotates(double rowx, double rowy, double rowz, double q0, double q1, double q2, double q3, double *x, double *y, double *z);
void get_imu_h(float *h);
#endif 
