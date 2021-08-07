#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include "escout.h"
#include "state_handle.h"
#include "imu.h"
#include "gethight.h"
#include <EEPROM.h>
#include "optical_flow.h"
#include <Arduino.h>
#include "pid.h"
#define Debug 1
#define LEDPIN 27
float Yaw_angle, Roll_angle, Pitch_angle;
float gx, gy, gz, ax, ay, az;
float g_vx = 0, g_vy = 0, g_vz = 0, posx = 0, posy = 0, posz = 0;
float raw_altitude, altitude;
float a_hight;
float grand_ax, grand_ay, grand_az;
float expect_h = 20;
float q0, q1, q2, q3;
int out1, out2, out3, out4;
float ex_roll, ex_pitch, ex_yaw;

KFP KFP_height = {0.02, 0, 0, 0, 0.001, 0.543};
k pid;

uint8_t fly_flag;
int lenth = sizeof(k);
byte buff[sizeof(k)];
int lowpower, verylowpower;
void Task1(void *pvParameters);
void Task2(void *pvParameters);
void vTask3(void *pvParameters);
void vTask4(void *pvParameters);
void task_high(void *pvParameters);
void task_led(void *pvParameters);
void task_hpos(void *pvParameters);
void tadk_high_acc(void *pvParameters);

//wifi在core0，其他在core1；1为大核
void setup()
{
  esc_init(25, 32, 33, 26);
  led_init();
  disableCore0WDT();
  disableCore1WDT();
  Serial.begin(115200);
  EEPROM.begin(lenth + 1);
  gethigh_init();
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);                        //关闭低电压检测,避免无限重启
  xTaskCreatePinnedToCore(Task1, "Task1", 10000, NULL, 2, NULL, 0); //最后一个参数至关重要，决定这个任务创建在哪个核上.PRO_CPU 为 0, APP_CPU 为 1,或者 tskNO_AFFINITY 允许任务在两者上运行.
  xTaskCreatePinnedToCore(Task2, "Task2", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vTask3, "rate", 20000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vTask4, "pos", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(task_high, "high_pid", 30000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(task_led, "led_blink", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task_hpos, "hpos", 4000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(task_high_acc, "high_acc", 8000, NULL, 1, NULL, 1);
  //vTaskStartScheduler();;
  //实现任务的函数名称（task1）；任务的任何名称（“ task1”等）；分配给任务的堆栈大小，以字为单位；任务输入参数（可以为NULL）；任务的优先级（0是最低优先级）；任务句柄（可以为NULL）；任务将运行的内核ID（0或1）
}

void loop()
{
}

void Task1(void *pvParameters)
{
  bool ifreaded = 0;
  int time1 = 0;
  imu_init();

  //unsigned portBASE_TYPE uxHighWaterMark;//

  Serial.println("start");
  //在这里可以添加一些代码，这样的话这个任务执行时会先执行一次这里的内容（当然后面进入while循环之后不会再执行这部分了）
  EEPROM.readBytes(0, buff, lenth);

  esc_write(1000, 1000, 1000, 1000);
  delay(1);
  while (1)
  {
    time1 += 1;
    if (time1 == 65535)
      time1 = 0;
    ///////////////////////////read pid///////////////////
    //uxHighWaterMark=uxTaskGetStackHighWaterMark( NULL);
    //Serial.print("Task1");
    //Serial.println(uxHighWaterMark);
    if (Serial.available())
    {
      Serial.readBytes(buff, lenth);
      memcpy(&pid, buff, lenth);
      EEPROM.writeBytes(0, buff, lenth);
      EEPROM.commit();
      ifreaded = 1;
      //Reset_pose_i();//清空积分项
    }
    if (ifreaded)
    {
      EEPROM.readBytes(0, buff, lenth);
      memcpy(&pid, buff, lenth);
      copy(pid);
      ifreaded = 0;
    }
    gethight(&raw_altitude);
   // raw_altitude = raw_altitude * cos(Roll_angle * 3.14159 / 180) * cos(Pitch_angle * 3.14159 / 180);
    altitude = kalmanFilter(&KFP_height, raw_altitude);
    //Serial.printf("%f   %f    %f     \n",pid.p1[0],pid.p2[1],pid.i1[2]);
    //////////////////////////////////read sensor///////////////////////////////

    //getquater();
    getdata();
    //Serial.printf("%f,%f\n",a_hight,grand_az);
    //Serial.print(Roll_angle);
    //Serial.print(',');
    //Serial.print(Pitch_angle);
    //Serial.print('\n');

#ifdef Debug

//Serial.printf("%f %f  %f    %f  %f  %f \n",pid.p1[0],pid.p1[1],pid.p1[2]);
//Serial.printf("alti:%f",altitude);
//Serial.printf("%f   %f   %f   %f   %f  %f   %f  %f   %f   %f   %f\n",Yaw_angle, Roll_angle, Pitch_angle,gx, gy, gz,altitude,q0,q1,q2,q3);
#endif
  }
}

void Task2(void *pvParameters)
{

  uint32_t time2 = 0;
  unsigned long now = 0;

  //unsigned portBASE_TYPE uxHighWaterMark;
  while (1)
  {
    static bool state = 0;

    //uxHighWaterMark=uxTaskGetStackHighWaterMark( NULL);
    //Serial.print("Task2");
    //Serial.println(uxHighWaterMark);
    //if (millis()-now==1000) Serial.printf("f: %d",time2);
    //Serial.println("i m running");
    //////////////////////////////////////////////////////////////////////////////////////////////////

    out(&out1, &out2, &out3, &out4);
    vTaskDelay(2);
    fly_handle();
    if (pid.startflag)
      esc_write(out1, out2, out3, out4);
    else
      esc_write(1000, 1000, 1000, 1000);

#ifdef Debug
//Serial.printf("ax,zy,zy: %f   %f   %f \n",ax,ay,az);
//Serial.printf("q0,q1,q2,q3: %f   %f  %f  %f \n",q0,q1,q2,q3);
//Serial.printf("%d,%d,%d,%d\n",out1,out2,out3,out4);
//Serial.printf("dt :%lf" ,dt);
#endif
  }
}
//每10次系统节拍执行一次
void vTask3(void *pvParameters)
{
  static portTickType xLastWakeTime;
  const portTickType xFrequency = 2;
  long now = micros();
  int dt3;
  bool state;
  // 使用当前时间初始化变量xLastWakeTime
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    dt3 = micros() - now;
    now = micros();
    //等待下一个周期
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    rate_pid();
    // 需要周期性执行代码放在这里
  }
}
void vTask4(void *pvParameters)
{
  static portTickType xLastWakeTime;
  const portTickType xFrequency = 5;
  long now = micros();
  bool flag = 0;
  unsigned portBASE_TYPE uxPriority;

  xLastWakeTime = xTaskGetTickCount();
  bool state = 0;
  // 使用当前时间初始化变量xLastWakeTime

  for (;;)
  {
    if (!flag)
    {
      uxPriority = uxTaskPriorityGet(NULL);
      vTaskPrioritySet(NULL, (uxPriority - 2));
      flag = 1;
    }

    //等待下一个周期
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    posturepid();
    digitalWrite(15, state);
    state = !state;

    // 需要周期性执行代码放在这里
  }
}
//每10次系统节拍执行一次
void task_high(void *pvParameters)
{
  static portTickType xLastWakeTime;
  const portTickType xFrequency = 10;

  // 使用当前时间初始化变量xLastWakeTime
  xLastWakeTime = xTaskGetTickCount();
  
  for (;;)
  {
    //等待下一个周期
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    heightcontrol();
    // 需要周期性执行代码放在这里
  }
}
void task_high_acc(void *pvParameters)
{
  static portTickType xLastWakeTime;
  const portTickType xFrequency = 5;

  // 使用当前时间初始化变量xLastWakeTime
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    //等待下一个周期
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    hight_acc();
    // 需要周期性执行代码放在这里
  }
}

void task_led(void *pvParameters)
{
  static portTickType xLastWakeTime;
  const portTickType xFrequency = 200;
  pinMode(15, INPUT);
  xLastWakeTime = xTaskGetTickCount();
  short led_flag;
  while (1)
  {

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    if (fly_flag == 2)
      digitalWrite(LEDPIN, 1);
    else
      digitalWrite(LEDPIN, 0);
  }
}

void task_hpos(void *pvParameters)
{
  static portTickType xLastWakeTime;
  const portTickType xFrequency = 50;
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //opt_control();
  }
}