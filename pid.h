
void posturepid();
void heightcontrol();
void Reset_pose_i();
void hight_acc();
void hight_v();
void out(int *out1, int *out2,int *out3,int *out4 );
#define roll 0
#define pitch 1
#define yaw 2
typedef struct
{
    float p1[3], i1[3], d1[3], p2[3], i2[3], d2[3];
    float p[3], i[3], d[3];
    float expect[3];
    float ex_h;
    float evx,xp,xi,xd;
    float evy,yp,yi,yd;
    bool state[4];
    float startflag;
} k;

void copy(k pid);
void rate_pid();

typedef struct
{

    long Last_Time;
    long  Now_Time;
    long  Time_Delta;

}Testime;

typedef  struct
{
    float Input_Butter[3];
    float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
    float a[3];
    float b[3];
}Butter_Parameter;
float Control_Device_LPF(float curr_inputer, Butter_BufferData* Buffer, Butter_Parameter* Parameter);
class PID
{
public:
    float Kp, Ki, Kd;//控制参数 PID
    float Expect;     //期望
    float FeedBack;   //反馈值
    float Err;        //偏差
    float Last_Err;   //上次偏差
    uint8_t Err_Max;    //偏差限幅值
    float Integrate_Separation_Err;//积分分离偏差值
    float Integrate;//积分值
    float Integrate_Max;//积分限幅值
    float Control_OutPut;//控制器总输出voi
    float Last_Control_OutPut;//上次控制器总输出
    float Control_OutPut_Limit;//输出限幅
    /***************************************/
    float Pre_Last_Err;//上上次偏差
    float Adaptable_Kd;//自适应微分参数
    float Last_FeedBack;//上次反馈值
    float Dis_Err;//微分量
    float Dis_Error_History[5];//历史微分量
    float Err_LPF;
    float Last_Err_LPF;
    float Dis_Err_LPF;
    float Last_Dis_Err_LPF;
    float Pre_Last_Dis_Err_LPF;
    Butter_Parameter Control_Device_Err_LPF_Parameter;
    Butter_BufferData Control_Device_LPF_Buffer;//控制器低通输入输出缓冲
    Testime PID_Controller_Dt;    //前面有Testime结构体的定义
    float Scale_Kp;
    float Scale_Ki;
    float Scale_Kd;
    bool Err_Limit_Flag = 0;
    bool Integrate_Separation_Flag = 0;
    bool Integrate_Limit_Flag = 0;
    float PID_Control_Div_LPF();
    float PID_Control_Err_LPF();
    void  Set_pid(float p, float i, float d);
    void Set_ex_feed(float ex, float fe);
    void Set_errormax(uint8_t emax);
    void Set_i_error(float imax);
    
    void Set_i_mode(bool elimitflag, bool isepara_flag, bool ilimitflag);
    void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);
    

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