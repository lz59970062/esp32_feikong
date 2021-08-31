#include <Wire.h>
#include "JY901.h"
#include "Arduino.h"
#define Debug 1
extern float Yaw_angle, Roll_angle, Pitch_angle;
extern float gx, gy, gz;
extern float ax, ay, az;
extern float g_vx, g_vy, g_vz;
extern float posx, posy, posz;
extern float grand_ax, grand_ay, grand_az;
extern float altitude;
extern float ex_roll, ex_pitch, ex_yaw;
void(* resetFunc) (void) = 0;
bool inited = 0;
extern float q0, q1, q2, q3;
float xoff=-0.4888, yoff=-0.2087, zoff;
#define g 9.783
//本地加速度
#define cyclenumber 20
#define avgnumber 200

#define IPOS 1
//获取四元数
void getquater()
{
    static float pq0, pq1, pq2, pq3;
    float rate = 0.5;
    if ((!pq0 && !pq1 && !pq2 && !pq3))
    {
        pq0 = q0;
        pq1 = q1;
        pq2 = q2;
        pq3 = q3;
    }
    JY901.GetQ();
    q0 = rate * (float)JY901.stcSQ.q0 / 32678 + (1 - rate) * pq0;
    q1 = rate * (float)JY901.stcSQ.q1 / 32678 + (1 - rate) * pq1;
    q2 = rate * (float)JY901.stcSQ.q2 / 32678 + (1 - rate) * pq2;
    q3 = rate * (float)JY901.stcSQ.q3 / 32678 + (1 - rate) * pq3;
    pq0 = q0;
    pq1 = q1;
    pq2 = q2;
    pq3 = q3;
}
#define NUM 6
float fiter1(float input)
{
    static int cnt = 0;
    static float f[NUM] = {0};
    float  sum = 0;
    f[cnt++] = input;
    if (cnt == 5)
        cnt = 0;
    for (int i = 0; i < NUM; i++)
    {
        f[i] == 0 ? f[i] = input : 1;
    }
    for (int i = 0; i < NUM; i++)
    {
        sum+=f[i];

    }
    return sum/NUM;
}
//通过四元数进行坐标系转换,从机体系转换到地面系

void rotates(double rowx, double rowy, double rowz, double q0, double q1, double q2, double q3, float *x, float *y, float *z)
{
    double q0_tmp;
    double b_q0_tmp;
    double c_q0_tmp;
    double d_q0_tmp;
    double b_q0[9];
    double e_q0_tmp;
    double f_q0_tmp;
    double g_q0_tmp;
    int i0;
    double temp2[3];
    q0_tmp = q0 * q0;
    b_q0_tmp = q1 * q1;
    c_q0_tmp = q2 * q2;
    d_q0_tmp = q3 * q3;
    b_q0[0] = ((q0_tmp + b_q0_tmp) - c_q0_tmp) - d_q0_tmp;
    e_q0_tmp = q1 * q2;
    f_q0_tmp = q0 * q3;
    g_q0_tmp = 2.0 * (e_q0_tmp + f_q0_tmp);
    b_q0[1] = g_q0_tmp;
    b_q0[2] = 2.0 * (q1 * q3 - q0 * q2);
    b_q0[3] = 2.0 * (e_q0_tmp - f_q0_tmp);
    q0_tmp -= b_q0_tmp;
    b_q0[4] = (q0_tmp + c_q0_tmp) - d_q0_tmp;
    b_q0_tmp = q2 * q3;
    e_q0_tmp = q0 * q1;
    b_q0[5] = 2.0 * (e_q0_tmp + b_q0_tmp);
    b_q0[6] = g_q0_tmp;
    b_q0[7] = 2.0 * (b_q0_tmp - e_q0_tmp);
    b_q0[8] = (q0_tmp - c_q0_tmp) + d_q0_tmp;
    for (i0 = 0; i0 < 3; i0++)
    {
        temp2[i0] = (b_q0[i0] * rowx + b_q0[i0 + 3] * rowy) + b_q0[i0 + 6] * rowz;
    }

    *x = temp2[0];
    *y = temp2[1];
    *z = temp2[2];
}

//获得九轴数据并滤波
void getdata()
{
    double dt;
    static long pt = 0;
    static float pax, pay, paz, pxangle, pyangle, pzangle, pgx, pgy, pgz;
    float rate = 0.6;

    pt > 0 ? pt : 0;
    dt = (micros() - pt) * 0.000001;
    pt = micros();
    JY901.GetAcc();
    JY901.GetAngle();
    JY901.GetGyro();

    if (!pax && !pay && !paz && !pxangle && !pyangle && !pzangle && !pgx && !pgy && !pgz)
    {
        pax = ax;
        pay = ay;
        paz = az;
        pxangle = Roll_angle;
        pyangle = Pitch_angle;
        pzangle = Yaw_angle;
        pgx = gx;
        pgy = gy;
        pgz = gz;
    }
    ax = rate * (float)JY901.stcAcc.a[0] / 32768 * 16 * g + (1 - rate) * pax; //m/s*s
    ay = rate * (float)JY901.stcAcc.a[1] / 32768 * 16 * g + (1 - rate) * pay;
    az = rate * (float)JY901.stcAcc.a[2] / 32768 * 16 * g + (1 - rate) * paz;
    Roll_angle = rate * (float)JY901.stcAngle.Angle[0] / 32768 * 180 + (1 - rate) * pxangle-xoff;
    Pitch_angle = rate * (float)JY901.stcAngle.Angle[1] / 32768 * 180 + (1 - rate) * pyangle-yoff;
    Yaw_angle = rate * (float)JY901.stcAngle.Angle[2] / 32768 * 180 + (1 - rate) * pzangle;
    gx = rate * (float)JY901.stcGyro.w[0] / 32768 * 2000 + (1 - rate) * pgx;
    gy = rate * (float)JY901.stcGyro.w[1] / 32768 * 2000 + (1 - rate) * pgy;
    gz = rate * (float)JY901.stcGyro.w[2] / 32768 * 2000 + (1 - rate) * pgz;
    getquater();
    float rawgrand_az=0;
    rotates(ax, ay, az, q0, q1, q2, q3, &grand_ax, &grand_ay, &rawgrand_az);
    grand_az=fiter1(rawgrand_az);
      
#ifdef IPOS
    posx += g_vx * dt;
    posy += g_vy * dt;
    posz += g_vz * dt;
#endif
    pax = ax;
    pay = ay;
    paz = az;
    pxangle = Roll_angle;
    pyangle = Pitch_angle;
    pzangle = Yaw_angle;
    pgx = gx;
    pgy = gy;
    pgz = gz;
#ifdef Debug
//Serial.printf("Getdata Dt: %lf",dt);
//Serial.printf("q0: %f q1:%f q2:%f q3:%f\n",q0, q1, q2, q3);
//Serial.printf("ax: %f ay:%f az:%f\n",ax, ay, az);
//Serial.printf("ax: %f ay:%f az:%f\n",grand_ax,grand_ay,grand_az);
//Serial.printf("vx:%f vy:%f vz:%f\n",g_vx,g_vy,g_vz);
//Serial.printf("posx:%f posy :%f posz:%f  \n",posx,posy,posz);
#endif
}

//初始化imu
void imu_init()
{
    JY901.StartIIC();
    double sum1[3] = {0, 0, 0};
    double sum[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < cyclenumber; j++)
        {
            getdata();
            if (i == 0)
                sum[i] += Roll_angle;
            if (i == 1)
                sum[i] += Pitch_angle;
            if (i == 2)
                sum[i] += Yaw_angle;
        }
    }

    ex_roll = (float)(sum[0] / cyclenumber);
    ex_pitch = (float)(sum[1] / cyclenumber);
    ex_yaw = (float)(sum[2] / cyclenumber);

    for (int count = 0; count < avgnumber; count++)
    {
        getdata();
        sum1[0] += ax;
        sum1[1] += ay;
        sum1[2] += az;
    }
    if(grand_az>9||grand_az<8)
    {
        resetFunc();
    }
    
    xoff = sum1[0] / avgnumber;
    yoff = sum1[1] / avgnumber;
    zoff = sum1[2] / avgnumber;

    inited = 1;
}
