#include <Arduino.h>
#include "control.h"
#define roll 0
#define pitch 1
#define yaw 2

#define Debug 1
#define i_limit 100
#define ratepid_1 1
#define ratepid_2 0
extern float Yaw_angle, Roll_angle, Pitch_angle;
extern float gx, gy, gz;
extern float altitude;
extern float v_hight;
extern float grand_az;
extern float g_vx, g_vy, g_vz;
extern float ex_roll, ex_pitch, ex_yaw;
extern float expect_h;
extern float zoff;
extern bool inited;
bool stable = 0;
float xout, yout;
int hover_thr = 1350;
#ifndef M_PI_F
#define M_PI_F 3.141592653589793f
#endif
#ifndef PI
#define PI M_PI_F
#endif
#ifndef M_PI_2
#define M_PI_2 1.570796326794897f
#endif

//////////////////////////////////////////////////////
//flag
bool Init_flag = 0;
bool Flying_flag = 0;
bool Landing_flag = 0;
bool Lowpower_flag = 0;
bool Landed_flag = 1;
bool Start_flag = 0;
/////////////////////////////////////////////////////
//33  PID类
typedef struct
{
    long Last_Time;
    long Now_Time;
    long Time_Delta;
} Testime;

typedef struct
{
    float Input_Butter[3];
    float Output_Butter[3];
} Butter_BufferData;

typedef struct
{
    float a[3];
    float b[3];
} Butter_Parameter;

//
typedef struct
{
    float ex[3];
} expect;

expect expect1, expect2;

typedef struct
{
    float error1[3], error2[3];
} errordef;
errordef poserror;

typedef struct
{
    float pre[3];
} perdef;
perdef prerror1;
perdef prerror2;

typedef struct
{
    double op, oi, od;
} pidout;
pidout posout1[3], posout2[3], highout;

float posoutangle[3];
float posoutrate[3];
float p[2], i[2], d[2];
int highpre = 0, higherror;
int thr = 1300;
//要传输，读取的pid常数
typedef struct
{
    float p1[3], i1[3], d1[3], p2[3], i2[3], d2[3];
    float p[3], i[3], d[3];
    float expect[3];
    float ex_h;
    float evx, xp, xi, xd;
    float evy, yp, yi, yd;
    bool state[4];
    float startflag;
} k;

k posepid;
void copy(k pid)
{
    memcpy(&posepid, &pid, sizeof(k));
}
void init_expect()
{
    if (stable)
    {
        expect1.ex[roll] = ex_roll;
        expect1.ex[pitch] = ex_pitch;
        expect1.ex[yaw] = ex_yaw;
    }
    else
    {
        expect1.ex[pitch] = 0;
        expect1.ex[roll] = 0;
        expect1.ex[yaw] = ex_yaw;
    }
}

Butter_BufferData rate_datax, rate_datay, rate_dataz;
Butter_Parameter rate_parameter;
float Control_Device_LPF(float curr_inputer, Butter_BufferData *Buffer, Butter_Parameter *Parameter)
{
    /*加速度计Butterworth滤波 */
    /* 获取最新x(n) */
    Buffer->Input_Butter[2] = curr_inputer;
    /* Butterworth滤波 */
    Buffer->Output_Butter[2] =
        Parameter->b[0] * Buffer->Input_Butter[2] + Parameter->b[1] * Buffer->Input_Butter[1] + Parameter->b[2] * Buffer->Input_Butter[0] - Parameter->a[1] * Buffer->Output_Butter[1] - Parameter->a[2] * Buffer->Output_Butter[0];
    /* x(n) 序列保存*/
    Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
    Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
    /* y(n) 序列保存 */
    Buffer->Output_Butter[0] = Buffer->Output_Butter[1];
    Buffer->Output_Butter[1] = Buffer->Output_Butter[2];
    return (Buffer->Output_Butter[2]);
}
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent, Butter_Parameter *LPF)
{
    float fr = sample_frequent / cutoff_frequent;
    float ohm = tan(M_PI_F / fr);
    float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
    if (cutoff_frequent <= 0.0f)
    {
        // no filtering
        return;
    }
    LPF->b[0] = ohm * ohm / c;
    LPF->b[1] = 2.0f * LPF->b[0];
    LPF->b[2] = LPF->b[0];
    LPF->a[0] = 1.0f;
    LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
    LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}

float ki_process(float k, float error, uint8_t limit)
{
    float rate;
    if (fabs(error) > limit)
        rate = 0;
    else
        rate = map(fabs(error), 0, limit, 2, 0);
    return rate * k;
}
float ki_process(float k, float error, uint8_t limit, float i_item) //积分常数区间处理
{
    float rate;
    /**
    if (fabs(i_item) > i_limit)
    {
        if (i_item > 0)
        {
            if (error < 0)
                return k;
            else
                return 0;
        }
        if (i_item < 0)
        {

            if (error > 0)
                return k;
            else
                return 0;
        }
    }
    **/

    if (fabs(error) > limit)
        return 0;
    else
        rate = map(fabs(error), 0, limit, 2, 0);
    return rate * k;
}

void posturepid()
{
    static bool state = 0;
    double dt;
    static long pt = 0;
    static bool traget_initflag = 0;
    if (!traget_initflag)
    {

        init_expect();
        traget_initflag = 1;
    }
#ifdef Debug
#endif

    if (inited)
    {
        pt > 0 ? pt : 0;
        dt = (micros() - pt) * 0.000001;
        pt = micros();
        if (Roll_angle > 80 || Pitch_angle > 80)
        {
            Landed_flag = 1;
            Flying_flag = 0;
            Start_flag = 0;
        }
        posepid.expect[yaw]=zoff;
        posepid.expect[yaw]=constrain(posepid.expect[yaw],-150,150);
       
        poserror.error1[roll] = posepid.expect[roll] - Roll_angle;
        poserror.error1[pitch] = posepid.expect[pitch] - Pitch_angle;
        poserror.error1[yaw] = posepid.expect[yaw] - Yaw_angle; //;
        //fabs(poserror.error1[roll]) < 0.5 ? poserror.error1[roll] = 0 : 1;
        //fabs(poserror.error1[pitch]) < 0.5 ? poserror.error1[pitch] = 0 : 1;
        //fabs(poserror.error1[yaw]) < 0.5 ? poserror.error1[yaw] = 0 : 1;
        //Serial.printf("%f,%f\n", posepid.expect[pitch], Pitch_angle);
        if(posepid.state[3]){
            Serial.printf("%f,%f\n",posepid.expect[roll],Roll_angle);
        }
        posout1[roll].op = posepid.p1[roll] * poserror.error1[roll];

        posout1[roll].oi += ki_process(posepid.i1[roll], poserror.error1[roll], 10, posout1[roll].oi) * poserror.error1[roll] * dt;
        posout1[roll].oi = constrain(posout1[roll].oi, -80, 80);
        posout1[roll].od = posepid.d1[roll] * (poserror.error1[roll] - prerror1.pre[roll]) / dt;
        prerror1.pre[roll] = poserror.error1[roll];
        if (Landed_flag = 1)
            posout1[roll].oi = 0;
        posoutangle[roll] = constrain(posout1[roll].op + posout1[roll].oi + posout1[roll].od, -800, 800);

        posout1[pitch].op = posepid.p1[pitch] * poserror.error1[pitch];

        posout1[pitch].oi += ki_process(posepid.i1[pitch], poserror.error1[pitch], 10, posout1[pitch].oi) * poserror.error1[pitch] * dt;
        posout1[pitch].oi = constrain(posout1[pitch].oi, -80, 80);
        posout1[pitch].od = posepid.d1[pitch] * (poserror.error1[pitch] - prerror1.pre[pitch]) / dt;
        prerror1.pre[pitch] = poserror.error1[pitch];
        if (Landed_flag = 1)
            posout1[pitch].oi = 0;
        posoutangle[pitch] = constrain(posout1[pitch].op + posout1[pitch].oi + posout1[pitch].od, -800, 800);
        posout1[yaw].op = posepid.p1[yaw] * poserror.error1[yaw];
        posout1[yaw].oi += ki_process(posepid.i1[yaw], poserror.error1[yaw], 10) * (poserror.error1[yaw]) * dt;
        posout1[yaw].oi = constrain(posout1[yaw].oi, -150, 150);
        posout1[yaw].od = posepid.d1[yaw] * (poserror.error1[yaw] - prerror1.pre[yaw]) / dt;
        prerror1.pre[yaw] = poserror.error1[yaw];
        posoutangle[yaw] = constrain(posout1[yaw].op + posout1[yaw].oi + posout1[yaw].od, -800, 800);
    }
}
#ifdef ratepid_1
void rate_pid()
{

    static bool initflag = 0;
    double dt;
    static long pt = 0;
    pt > 0 ? pt : 0;
    dt = (micros() - pt) * 0.000001;
    pt = micros();
    if (!initflag)
    {
        Set_Cutoff_Frequency(500, 20, &rate_parameter);
        initflag = 1;
    }
    ///////////////////////////// 二级pid/////////////////////////////////////////////////
    expect2.ex[roll] = posoutangle[roll];
    expect2.ex[pitch] = posoutangle[pitch];
    expect2.ex[yaw] = posoutangle[yaw];

    poserror.error2[roll] = expect2.ex[roll] - gx;
    poserror.error2[pitch] = expect2.ex[pitch] - gy;
    poserror.error2[yaw] = expect2.ex[yaw] - gz;
    //poserror.error2[yaw] = posepid.expect[yaw]-gz;
    poserror.error2[roll] = Control_Device_LPF(poserror.error2[roll], &rate_datax, &rate_parameter);
    poserror.error2[pitch] = Control_Device_LPF(poserror.error2[pitch], &rate_datay, &rate_parameter);
    poserror.error2[yaw] = Control_Device_LPF(poserror.error2[yaw], &rate_dataz, &rate_parameter);

    posout2[roll].op = posepid.p2[roll] * poserror.error2[roll];
    posout2[roll].oi += ki_process(posepid.i2[roll], poserror.error2[roll], 600, posout2[roll].oi) * (poserror.error2[roll]) * dt;
    posout2[roll].oi = constrain(posout2[roll].oi, -300, 300);
    posout2[roll].od = posepid.d2[roll] * (poserror.error2[roll] - prerror2.pre[roll]) / dt;
    prerror2.pre[roll] = poserror.error2[roll];
    posoutrate[roll] = constrain(posout2[roll].op + posout2[roll].oi + posout2[roll].od, -500, 500);

    posout2[pitch].op = posepid.p2[pitch] * poserror.error2[pitch];
    posout2[pitch].oi += ki_process(posepid.i2[pitch], poserror.error2[pitch], 600, posout2[pitch].oi) * (poserror.error2[pitch]) * dt;
    posout2[pitch].oi = constrain(posout2[pitch].oi, -300, 300);
    posout2[pitch].od = posepid.d2[pitch] * (poserror.error2[pitch] - prerror2.pre[pitch]) / dt;
    prerror2.pre[pitch] = poserror.error2[pitch];
    posoutrate[pitch] = constrain(posout2[pitch].op + posout2[pitch].oi + posout2[pitch].od, -500, 500);

    posout2[yaw].op = posepid.p2[yaw] * poserror.error2[yaw];
    posout2[yaw].oi += ki_process(posepid.i2[yaw], poserror.error2[yaw], 300) * poserror.error2[yaw] * dt;
    posout2[yaw].oi = constrain(posout2[yaw].oi, -100, 100);
    posout2[yaw].od = posepid.d2[yaw] * (poserror.error2[yaw] - prerror2.pre[yaw]) / dt;
    prerror2.pre[yaw] = poserror.error2[yaw];
    posoutrate[yaw] = constrain(posout2[yaw].op + posout2[yaw].oi + posout2[yaw].od, -300, 300);
}
void Reset_pose_i()
{
    if (posepid.state[1])
    {
        posout2[yaw].oi = posout2[roll].oi = posout2[pitch].oi = posout1[yaw].oi = posout1[roll].oi = posout1[pitch].oi = 0;
        thr = 1350;
        hover_thr = 1350;
    }
}
#ifdef Debug
//Serial.printf("posout %f   %f    %f   \n", poserror.error1[roll], poserror.error1[pitch], poserror.error1[yaw]);
//Serial.printf("dt1 %lf \t",dt);
#endif

#endif

float estimateMinThru(void)
{
    float minThru = 1;
    float BatteryVal = analogRead(15) * 0.00362005622; //实际电压值计算0.00374277为实验得出的系数
    //Serial.printf("%f,\n",BatteryVal);
    BatteryVal/=3;
    if (BatteryVal > 3.2 && BatteryVal < 4.2)
    {
        if (BatteryVal > 4.05)
        {
            minThru = 1.0;
        }
        else if (BatteryVal > 3.80)
        {
            minThru = 1.02;
        }
        else if (BatteryVal > 3.7)
        {
            minThru = 1.03;
        }
        else
        {
            minThru = 1.04;
        }

        return minThru;
    }
    else
        return 1;
}

//33

//控制设备LPF

class PID
{
public:
    float Kp, Ki, Kd;               //控制参数 PID
    float Expect;                   //期望
    float FeedBack;                 //反馈值
    float Err;                      //偏差
    float Last_Err;                 //上次偏差
    uint8_t Err_Max;                //偏差限幅值
    float Integrate_Separation_Err; //积分分离偏差值
    float Integrate;                //积分值
    float Integrate_Max;            //积分限幅值
    float Control_OutPut;           //控制器总输出voi
    float Last_Control_OutPut;      //上次控制器总输出
    float Control_OutPut_Limit;     //输出限幅
    /***************************************/
    float Pre_Last_Err;         //上上次偏差
    float Adaptable_Kd;         //自适应微分参数
    float Last_FeedBack;        //上次反馈值
    float Dis_Err;              //微分量
    float Dis_Error_History[5]; //历史微分量
    float Err_LPF;
    float Last_Err_LPF;
    float Dis_Err_LPF;
    float Last_Dis_Err_LPF;
    float Pre_Last_Dis_Err_LPF;
    Butter_Parameter Control_Device_Err_LPF_Parameter;
    Butter_Parameter Control_Device_Div_LPF_Parameter;
    Butter_BufferData Control_Device_LPF_Buffer; //控制器低通输入输出缓冲
    Testime PID_Controller_Dt;                   //前面有Testime结构体的定义

    bool Err_Limit_Flag = 0;
    bool Integrate_Separation_Flag = 0;
    bool Integrate_Limit_Flag = 0;
    float PID_Control_Div_LPF();
    float PID_Control_Err_LPF();
    void Set_pid(float p, float i, float d);
    void Set_ex_feed(float ex, float fe);
    void Set_errormax(uint8_t emax);
    void Set_i_error(float imax);
    void Set_i_mode(bool elimitflag, bool isepara_flag, bool ilimitflag);
    void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent, Butter_Parameter *LPF);

    // float ntegrate_Separation_Err;
    // float Integrate;
    //  Integrate_Max, Control_OutPut, Last_Control_OutPut, Control_OutPut_Limit, Pre_Last_Err，;
    // uint16_t limit;
    // float preerror;
    // float pre_ivalue = 0;
    // bool seted1, seted2; //确保参数都要设定后才能做pip运算
    // long pt = 0;
    // int process(float traget, float measure);
    // int process(float traget, float measure, float rate, float d_value);
    // void setpid(float kp, float ki, float kd);
    // void setlimit(uint16_t li);
};
void PID::Set_pid(float p, float i, float d)
{
    Kp = p;
    Ki = i;
    Kd = d;
}
void PID::Set_ex_feed(float ex, float fe)
{
    Expect = ex;
    FeedBack = fe;
}
void PID::Set_errormax(uint8_t emax)
{
    Err_Max = emax;
}
void PID::Set_i_error(float imax)
{
    Integrate_Max = imax;
}
void PID::Set_i_mode(bool elimitflag, bool isepara_flag, bool ilimitflag)
{
    Err_Limit_Flag = elimitflag;
    Integrate_Separation_Flag = isepara_flag;
    Integrate_Limit_Flag = ilimitflag;
}
void PID::Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent, Butter_Parameter *LPF)
{
    float fr = sample_frequent / cutoff_frequent;
    float ohm = tan(M_PI_F / fr);
    float c = 1.0f + 2.0f * cos(M_PI_F / 4.0f) * ohm + ohm * ohm;
    if (cutoff_frequent <= 0.0f)
    {
        // no filtering
        return;
    }
    LPF->b[0] = ohm * ohm / c;
    LPF->b[1] = 2.0f * LPF->b[0];
    LPF->b[2] = LPF->b[0];
    LPF->a[0] = 1.0f;
    LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
    LPF->a[2] = (1.0f - 2.0f * cos(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}

float PID::PID_Control_Div_LPF()
{

    float tempa, tempb, tempc, max, min; //用于防跳变滤波
    double controller_dt = 0;

    PID_Controller_Dt.Now_Time = micros();
    PID_Controller_Dt.Time_Delta = PID_Controller_Dt.Now_Time - PID_Controller_Dt.Last_Time;
    PID_Controller_Dt.Last_Time = PID_Controller_Dt.Now_Time;
    controller_dt = PID_Controller_Dt.Time_Delta * 0.000001f;

    /*******偏差计算*********************/
    Last_Err = Err;           //保存上次偏差
    Err = Expect - FeedBack;  //期望减去反馈得到偏差
    Dis_Err = Err - Last_Err; //原始微分

    /******************************************/
    //均值滤波，保证得到数据不跳变，避免期望阶跃时,微分输出异常
    tempa = Pre_Last_Dis_Err_LPF;
    tempb = Last_Dis_Err_LPF;
    tempc = Dis_Err;
    max = tempa > tempb ? tempa : tempb;
    max = max > tempc ? max : tempc;
    min = tempa < tempb ? tempa : tempb;
    min = min < tempc ? min : tempc;
    if (tempa > min && tempa < max)
        Dis_Err = tempa;
    if (tempb > min && tempb < max)
        Dis_Err = tempb;
    if (tempc > min && tempc < max)
        Dis_Err = tempc;
    Pre_Last_Dis_Err_LPF = Last_Dis_Err_LPF;
    Last_Dis_Err_LPF = Dis_Err;
    /*****************************************/

    for (int16_t i = 4; i > 0; i--) //数字低通后微分项保存
    {
        Dis_Error_History[i] = Dis_Error_History[i - 1];
    }
    Dis_Error_History[0] = Control_Device_LPF(Dis_Err,
                                              &Control_Device_LPF_Buffer,
                                              &Control_Device_Div_LPF_Parameter); //巴特沃斯低通后得到的微分项，20hz

    if (Err_Limit_Flag == 1) //偏差限幅度标志位
    {
        if (Err >= Err_Max)
            Err = Err_Max;
        if (Err <= -Err_Max)
            Err = -Err_Max;
    }
    /*******积分计算*********************/
    if (Integrate_Separation_Flag == 1) //»ý·Ö·ÖÀë±êÖ¾Î»
    {
        if (fabs(Err) <= Integrate_Separation_Err)
            Integrate += Ki * Err * controller_dt;
    }
    else
    {
        Integrate += Ki * Err * controller_dt;
    }
    /*******»ý·ÖÏÞ·ù*********************/
    if (Integrate_Limit_Flag == 1) //»ý·ÖÏÞÖÆ·ù¶È±êÖ¾
    {
        if (Integrate >= Integrate_Max)
            Integrate = Integrate_Max;
        if (Integrate <= -Integrate_Max)
            Integrate = -Integrate_Max;
    }
    /*******×ÜÊä³ö¼ÆËã*********************/
    Last_Control_OutPut = Control_OutPut;                         //Êä³öÖµµÝÍÆ
    Control_OutPut = Kp * Err                                     //±ÈÀý
                     + Integrate                                  //»ý·Ö
                                                                  //+Controler->Kd*Controler->Dis_Err;//Î¢·Ö
                     + Kd * Dis_Error_History[0] / controller_dt; //Î¢·ÖÏîÀ´Ô´ÓÚ°ÍÌØÎÖË¹µÍÍ¨ÂË²¨Æ÷
    /*******×ÜÊä³öÏÞ·ù*********************/

    Control_OutPut = constrain(Control_OutPut, -Control_OutPut_Limit, Control_OutPut_Limit);
    /*******·µ»Ø×ÜÊä³ö*********************/
    return Control_OutPut;
}

float PID::PID_Control_Err_LPF()
{
    float controller_dt = 0;

    PID_Controller_Dt.Now_Time = micros();
    PID_Controller_Dt.Time_Delta = PID_Controller_Dt.Now_Time - PID_Controller_Dt.Last_Time;
    PID_Controller_Dt.Last_Time = PID_Controller_Dt.Now_Time;
    controller_dt = PID_Controller_Dt.Time_Delta * 0.000001f;

    /*******偏差计算*********************/
    Last_Err = Err;           //保存上次偏差
    Err = Expect - FeedBack;  //期望值减去反馈得到偏差
    Dis_Err = Err - Last_Err; //原始微分
    //Serial.println(Err);
    Last_Err_LPF = Err_LPF;

    Err_LPF = Control_Device_LPF(Err,
                                 &Control_Device_LPF_Buffer,
                                 &Control_Device_Err_LPF_Parameter); ///°ÍÌØÎÖË¹µÍÍ¨ºóµÃµ½µÄÎ¢·ÖÏî,20hz

    Dis_Err_LPF = Err_LPF - Last_Err_LPF; //偏差经过低通后的微分量
    //Serial.println(Err_LPF);
    if (Err_Limit_Flag == 1) //偏差限幅度标志位
    {
        if (Err_LPF >= Err_Max)
            Err_LPF = Err_Max;
        if (Err_LPF <= -Err_Max)
            Err_LPF = -Err_Max;
    }
    /*******积分计算*********************/
    if (Integrate_Separation_Flag == 1) //积分分离标志位
    {
        if (fabs(Err_LPF) <= Integrate_Separation_Err)
            Integrate += Ki * Err_LPF * controller_dt;
    }
    else
    {
        Integrate += Ki * Err_LPF * controller_dt;
    }
    /*******积分限幅*********************/
    if (Integrate_Limit_Flag == 1) //积分限制幅度标志
    {
        if (Integrate >= Integrate_Max)
            Integrate = Integrate_Max;
        if (Integrate <= -Integrate_Max)
            Integrate = -Integrate_Max;
    }
    /*******总输出计算*********************/
    Last_Control_OutPut = Control_OutPut;                //输出值递推
    Control_OutPut = Kp * Err_LPF                        //比例
                     + Integrate                         //积分
                     + Kd * Dis_Err_LPF / controller_dt; //已对偏差低通，此处不再对微分项单独低通
    /*******总输出限幅*********************/
    Control_OutPut = constrain(Control_OutPut, -Control_OutPut_Limit, Control_OutPut_Limit);
    /*******返回总输出*********************/
    return Control_OutPut;
}

// void PID::setpid(float kp, float ki, float kd)
// { //设置pid参数
//     p = kp;
//     i = ki;
//     d = kd;

//     seted1 = 1;
// }

// void PID::setlimit(uint16_t li)
// {
//     limit = li;
//     if (limit > 0)
//     {
//         seted2 = 1;
//     }
// }
// int PID::process(float traget, float measure)
// { //输出的值仍需限幅

//     double dt;
//     float output = 0;
//     float error;

//     if (seted1 == 0)
//         return 0;
//     if (seted2 == 0)
//         return 0;
//     pt > 0 ? pt : 0;
//     dt = (micros() - pt) * 0.000001;
//     pt = micros();
//     error = traget - measure;
//     output += error * p;
//     //Serial.print(output);
//     output += d * (error - preerror) / dt;
//     //Serial.print(output);

//     pre_ivalue += ki_process(i, error, limit, pre_ivalue) * error * dt;

//     output += pre_ivalue;
//     preerror = error;
//     //Serial.printf("dt2 %lf \n",dt);
//     //Serial.println(output);
//     //Serial.printf(" pid %f   %f   %f \n",p,i,d);
//     return (int)output;
// }
// int PID::process(float traget, float measure, float rate, float d_value)
// { //数据融合pid，输出的值仍需限幅

//     double dt;
//     float output = 0;
//     float error;

//     if (seted1 == 0)
//         return 0;
//     if (seted2 == 0)
//         return 0;
//     pt > 0 ? pt : 0;
//     dt = (micros() - pt) * 0.000001;
//     pt = micros();
//     error = traget - measure;
//     output += error * p;
//     output += d * (rate * (error - preerror) / dt + (1 - rate) * d_value); //融合

//     pre_ivalue += ki_process(i, error, limit, pre_ivalue) * error * dt;

//     output += pre_ivalue;
//     preerror = error;
// #ifdef Debug
// // Serial.printf(" hpid :  %lf   output:%d\n" ,dt,output);
// #endif
//     return (int)output;
// }

// PID h1;

// //高度pid，待完善
// void heightcontrol()
// {
//     static bool initflag = 0;
//     int high_out1;

//     if (!initflag)
//     {
//         h1.setlimit(15);

//         initflag = 1;
//     }
//     h1.setpid(posepid.p[0], posepid.i[0], posepid.d[0]);

//     high_out1 = h1.process(expect_h, altitude);
//     //Serial.println(high_out1);
//     //Serial.printf("pid: %f  %f   %f  \n",posepid.p[0], posepid.i[0], posepid.d[0]);
//     //Serial.print("high_out1");Serial.println(high_out1);
//     thr = 1250 + constrain(high_out1, -500, 500);
//     //Serial.printf("expect_h,%f altitude:%f thr:%d\n",expect_h, altitude,thr);

// #ifdef Debug

//     //Serial.printf("heihout1: %d gvz: %f", high_out1, g_vz);
// //Serial.printf("thr %d out :%d   %f   %f  \n",thr,high_out ,expect_h , altitude);
// #endif
// }
//高度pid第二版本

////////////////////////////////转化预期高度////////////////////////////////
uint8_t climb_process(int ex_h)
{
    int gap = 10;
    static int last_ex = altitude + gap;
    if (altitude < ex_h)
    {
        if (altitude > last_ex)
        {
            last_ex += gap;
            return last_ex;
        }
        else
            return last_ex;
    }
    else
        return ex_h;
}
void hover_get(float expect)
{

    static float min_error = 20;
    if (fabs(expect - altitude) < 5)
    {
        float now_error = fabs(grand_az - 8.438) * 100;
        if (now_error < min_error)
        {
            min_error = now_error;
            hover_thr = thr;
        }
    }
    else if ((grand_az - 8.438) * 100 > 400)
        thr--;
    else if ((grand_az - 8.438) * 100 < -400)
        thr++;
}
PID h1;
PID h2;
PID h3;
int hight1_out = 0;
int hight2_out = 0;
int hight3_out = 0;
void heightcontrol()
{
    static bool initflag = 0;
    static uint32_t cnt1 = 0, cnt2 = 0;
    //
    if (Start_flag)
    {
        /* if (Landed_flag == 1)
        {
            cnt1++;
            if (cnt1 % 4 == 0)
            {
                thr++;
                hover_thr = thr;
            }
            return;
        } */

        //if (Flying_flag && !Landed_flag)
        {
            cnt1 = 0;
            if (!initflag)
            {
                h1.Set_errormax(20);
                h1.Set_i_error(40);
                h1.Set_i_mode(0, 1, 1);
                h1.Set_Cutoff_Frequency(100, 20, &h1.Control_Device_Err_LPF_Parameter);
                h1.Integrate_Separation_Err = 20;
                h1.Control_OutPut_Limit = 300;
                initflag = 1;
            }
            ////////////////////////////////////////////////////////////////////////
            expect_h = posepid.ex_h;
            // hover_get(expect_h);
            if (fabs(expect_h - altitude) < 1)
                expect_h = altitude;
            // Serial.printf("%f  %ld\n",h1.Err,micros());

            //float ki = ki_process(posepid.i[0], expect_h - altitude, 20);
            h1.Set_pid(posepid.p[0], posepid.i[0], posepid.d[0]);
            h1.Set_ex_feed(expect_h, altitude);
            hight1_out = (int)h1.PID_Control_Err_LPF();
            //Serial.println(hight1_out);
        }
    }
    if (posepid.state[1])
        Serial.printf("%f,%f\n", expect_h, altitude);
}
void hight_v()
{
    static bool initflag = false;
    //Serial.printf("hight_v %f\n",v_hight*0.1);
    //Serial.printf("%f ,%f ,%f ,%d ,%f ,%f\n",posepid.ex_h, altitude, grand_az, hight1_out, h2.Control_OutPut, h3.Control_OutPut);
    if (Start_flag)
    {
        // if (Flying_flag && !Landed_flag)
        {
            if (!initflag)
            {
                h2.Set_i_error(400);
                h2.Set_i_mode(0, 1, 1);
                h2.Set_Cutoff_Frequency(200, 20, &h2.Control_Device_Err_LPF_Parameter);
                h2.Control_OutPut_Limit = 400;
                h2.Integrate_Separation_Err = 200;
                initflag = 1;
            }
            //Serial.printf("%d   %f \n",hight1_out,v_hight*0.01);
            //Serial.printf("%f  %ld\n",v_hight,micros());
            float expect = hight1_out;
            //Serial.printf("%f  %ld\n",h2.Err,micros());
            h2.Set_pid(posepid.p[1], posepid.i[1], posepid.d[1]);
            fabs(expect - v_hight * 0.01) < 1 ? expect = v_hight * 0.01 : 1;

            h2.Set_ex_feed(expect, v_hight * 0.01);
            hight2_out = (int)h2.PID_Control_Err_LPF();

            thr = hover_thr + hight2_out;
            //Serial.printf("%d %f \n",hight2_out,v_hight*0.01);
        }
    }
}

void hight_acc()
{
    static bool initflag = 0;
    //Serial.println(grand_az);

    if (Start_flag)
    {
        if (Flying_flag && !Landed_flag)
        {
            if (!initflag)
            {
                h3.Set_i_error(400);
                h3.Set_i_mode(0, 1, 1);
                h3.Set_Cutoff_Frequency(250, 50, &h3.Control_Device_Err_LPF_Parameter);
                h3.Control_OutPut_Limit = 500;
                initflag = 1;
            }
            float expect = hight2_out;

            h3.Set_pid(posepid.p[2], posepid.i[2], posepid.d[2]);
            //fabs(expect - hight2_out < 1) ? expect = hight2_out : 1;
            h3.Set_ex_feed(expect * 0.1, grand_az);
            hight3_out = (int)h3.PID_Control_Err_LPF();
            //hight3_out += (int)((hover_thr + h3.Integrate - 1000) * ((float)expect / 843.8));
            //Serial.printf("%d %f \n",hight3_out,grand_az);
            thr = hover_thr + hight3_out;
        }
    }
}

//将pid输出量分配到四个电机
void out(int *out1, int *out2,int *out3,int *out4 )
{
    static int pre1, pre2, pre3, pre4;
    bool flag = 0;
    float rate = 0.9;
    float yrate = 0;

    if (flag == 0)
    {
        pre1 = *out1;
        pre2 = *out2;
        pre3 = *out3;
        pre4 = *out4;
        flag = 1;
    }
    yrate = (thr - 1000) / 500.0 + 0.25;
    yrate = constrain(yrate, 0, 1);
    *out1 = thr + yrate * (-posoutrate[pitch] - posoutrate[yaw] - posoutrate[roll]);
    *out2 = thr + yrate * (-posoutrate[roll] + posoutrate[pitch] + posoutrate[yaw]);
    *out3 = thr + yrate * (posoutrate[roll] + posoutrate[pitch] - posoutrate[yaw]);
    *out4 = thr + yrate * (posoutrate[roll] - posoutrate[pitch] + posoutrate[yaw]);

    *out1 = *out1 * rate + (1 - rate) * pre1;
    *out2 = *out2 * rate + (1 - rate) * pre2;
    *out3 = *out3 * rate + (1 - rate) * pre3;
    *out4 = *out4 * rate + (1 - rate) * pre4;
    if (abs(Roll_angle) > 70 || abs(Pitch_angle) > 70)
    {
        *out1 = *out2 = *out3 = *out4 = 1000;
    }
    *out1 = constrain(*out1, 1000, 2000);
    *out2 = constrain(*out2, 1000, 2000);
    *out3 = constrain(*out3, 1000, 2000);
    *out4 = constrain(*out4, 1000, 2000);

    // Serial.printf(" out1:  %d  %d  %d  %d \n", *out1, *out2, *out3, *out4);
    pre1 = *out1;
    pre2 = *out2;
    pre3 = *out3;
    pre4 = *out4;
    //Serial.printf("%d,%d,%d,%d\n",pre1,pre2,pre3,pre4);
}
