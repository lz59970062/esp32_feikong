#include <Arduino.h>

#define echo 13
#define trig 2
#define ULTRA 1
#define TOF 1

#ifdef PRESS //是否启用气压计，否则超声波定高
BMP280_DEV bmp280(21, 22);
#endif
#ifdef TOF
typedef struct
{
    float LastP; //上次估算协方差 初始化值为0.02
    float Now_P; //当前估算协方差 初始化值为0
    float out;   //卡尔曼滤波器输出 初始化值为0
    float Kg;    //卡尔曼增益 初始化值为0
    float Q;     //过程噪声协方差 初始化值为0.001
    float R;     //观测噪声协方差 初始化值为0.543
} KFP;
#include <Wire.h>
unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
unsigned char dirsend_flag = 0;
unsigned int TOF_I2Caddress = 82;
#define NUMBER 6
void fsort(float *s,int n){
   int i,j,pos;
   float tempmin ,temp;
   for(i=0;i<n;i++){
      tempmin=s[i];
      temp=i;
      for(j=i;j<n;j++){
        if(s[j]>tempmin){
            pos=j;
            tempmin=s[j];
        }
      }
      temp=s[pos];
      s[pos]=s[i];
      s[i]=temp;

   }
}
#define NUM 16
float fiter2(float input)
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
float data_correction(float input){//中值滤波
    static float last_input[NUMBER]={0};
    float temp[NUMBER];
    for (int i=0;i<NUMBER;i++){
        last_input[i]=last_input[i+1];
    }
    last_input[NUMBER-1]=input;
    memcpy(temp,last_input,NUMBER*sizeof(float));
    fsort(temp,NUMBER);
    return temp[NUMBER/2];
}
void SensorRead(unsigned char *datbuf, int TOF_I2Caddress)
{
    unsigned short result = 0;
    // step 1: instruct sensor to read echoes
    Wire.beginTransmission(TOF_I2Caddress); // transmit to device
    Wire.write(byte(0x00));                 // sets distance data address (0x00)
    Wire.endTransmission();                 // stop transmitting
    // step 2: wait for readings to happen
    vTaskDelay(1); // datasheet suggests at least 30uS
    // step 3: request reading from sensor
    Wire.requestFrom(TOF_I2Caddress, 2); // request 2 bytes from slave device #82 (0x52)
    // step 5: receive reading from sensor
    if (2 <= Wire.available())
    {                            // if two bytes were received
        *datbuf++ = Wire.read(); // receive high byte (overwrites previous reading)
        *datbuf++ = Wire.read(); // receive low byte as lower 8 bits
    }
}

int ReadDistance(int TOF_I2Caddress)
{
    SensorRead(i2c_rx_buf, TOF_I2Caddress);
    lenth_val = i2c_rx_buf[0];
    lenth_val = lenth_val << 8;
    lenth_val |= i2c_rx_buf[1];
    return lenth_val;
}

#endif

float alt_init = 0;
#define LIMIT_DH 2000
void gethight(float *hight)
{
    long duration;
    float distance;
    //static float a[10]=0;
    static float pre_hight = 0;
#ifdef TOF
    distance = ReadDistance(TOF_I2Caddress);

#endif
#ifdef ULATR
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    duration = pulseIn(echo, HIGH);
    distance = duration * 0.034 / 2;
#endif
#ifdef PRESS
    if (bmp280.getAltitude(distance)) // Check if the measurement is complete
    {

        a[i] = distance;
        i--;
        double sum = 0;
        sum = a[0] + a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[7] + a[8] + a[9]; //滑动滤波
        distance = sum / 10;
        if (i == -1)
            i = 9;
    }
#endif
    if (fabs(distance - pre_hight) < LIMIT_DH)
        *hight = distance * 0.1;
    else
        *hight = pre_hight;
    if(*hight>190)
        *hight = pre_hight;
    pre_hight = *hight;
    *hight=data_correction(*hight);
}

void gethigh_init()
{
#ifdef PRESS
    bmp280.begin();
    bmp280.setTimeStandby(TIME_STANDBY_05MS);
    bmp280.startNormalConversion();
#endif
#ifdef SR
    pinMode(echo, INPUT);
    pinMode(trig, OUTPUT);
    gethight(&alt_init);
#endif
#ifdef TOF
    Wire.begin(21, 22);
    gethight(&alt_init);
    while (!alt_init)
    {
        gethight(&alt_init);
    }
#endif
}

void gethight(float *hight, float *d_value)
{
    double dt;
    static long pt = 0;
    static float pre_v;
    float distance;
    static int i = 3;
    static int a[4];
    static float prevalue = alt_init;
    
    dt = (millis() - pt) * 0.001;
    pt = millis();
#ifdef TOF
    distance = ReadDistance(TOF_I2Caddress);
    
   
#endif
#ifdef SR

    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    duration = pulseIn(echo, HIGH);
    distance = duration * 0.034 / 2;
#endif
#ifdef PRESS
    if (bmp280.getAltitude(altitude)) // Check if the measurement is complete
    {

        a[i] = altitude;
        i--;
        double sum = 0;
        sum = a[0] + a[1] + a[2] + a[3]; //滑动滤波
        altitude = sum / 10;
        if (i == -1)
            i = 9;
        *hight = altitude;
    }
#endif
    distance=data_correction(distance)*0.1;
    *hight = distance;
    float v = (distance - prevalue) / dt; //mm/s
    v==0?v=pre_v:1;
    pre_v=v;
    v=fiter2(v);
    *d_value = v * 10;              //cm/s
    prevalue = distance;
    
}
//1. 结构体类型定义

//2. 以高度为例 定义卡尔曼结构体并初始化参数

/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float kalmanFilter(KFP *kfp, float input)
{
    //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    kfp->Now_P = kfp->LastP + kfp->Q;
    //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
    //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    kfp->out = kfp->out + kfp->Kg * (input - kfp->out); //因为这一次的预测值就是上一次的输出值
    //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
    kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
    return kfp->out;
}

/**
 *调用卡尔曼滤波器 实践
 */
