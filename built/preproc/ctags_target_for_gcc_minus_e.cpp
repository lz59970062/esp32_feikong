# 1 "e:\\飞总\\电赛\\电赛\\JY901.cpp"
# 2 "e:\\飞总\\电赛\\电赛\\JY901.cpp" 2
# 3 "e:\\飞总\\电赛\\电赛\\JY901.cpp" 2

CJY901 ::CJY901 ()
{
 ucDevAddr =0x50;
}
void CJY901::StartIIC()
{
 ucDevAddr = 0x50;
 Wire.begin(21,22);
}
void CJY901::StartIIC(unsigned char ucAddr)
{
 ucDevAddr = ucAddr;
 Wire.begin(21,22);
}
void CJY901 ::CopeSerialData(unsigned char ucData)
{
 static unsigned char ucRxBuffer[250];
 static unsigned char ucRxCnt = 0;

 ucRxBuffer[ucRxCnt++]=ucData;
 if (ucRxBuffer[0]!=0x55)
 {
  ucRxCnt=0;
  return;
 }
 if (ucRxCnt<11) {return;}
 else
 {
  switch(ucRxBuffer[1])
  {
   case 0x50: memcpy(&stcTime,&ucRxBuffer[2],8);break;
   case 0x51: memcpy(&stcAcc,&ucRxBuffer[2],8);break;
   case 0x52: memcpy(&stcGyro,&ucRxBuffer[2],8);break;
   case 0x53: memcpy(&stcAngle,&ucRxBuffer[2],8);break;
   case 0x54: memcpy(&stcMag,&ucRxBuffer[2],8);break;
   case 0x55: memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
   case 0x56: memcpy(&stcPress,&ucRxBuffer[2],8);break;
   case 0x57: memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
   case 0x58: memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
  }
  ucRxCnt=0;
 }
}
void CJY901::readRegisters(unsigned char deviceAddr,unsigned char addressToRead, unsigned char bytesToRead, char * dest)
{
  int i;
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(deviceAddr, bytesToRead); //Ask for bytes, once done, bus is released by default

  while((i=Wire.available()) < bytesToRead){
Serial.println(i);
  } //Hang out until we get the # of bytes we expect

  for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = Wire.read();
}
void CJY901::writeRegister(unsigned char deviceAddr,unsigned char addressToWrite,unsigned char bytesToRead, char *dataToWrite)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToWrite);
  for(int i = 0 ; i < bytesToRead ; i++)
  Wire.write(dataToWrite[i]);
  Wire.endTransmission(); //Stop transmitting
}

short CJY901::ReadWord(unsigned char ucAddr)
{
 short sResult;
 readRegisters(ucDevAddr, ucAddr, 2, (char *)&sResult);
 return sResult;
}
void CJY901::WriteWord(unsigned char ucAddr,short sData)
{
 writeRegister(ucDevAddr, ucAddr, 2, (char *)&sData);
}
void CJY901::ReadData(unsigned char ucAddr,unsigned char ucLength,char chrData[])
{
 readRegisters(ucDevAddr, ucAddr, ucLength, chrData);
}

void CJY901::GetTime()
{
 readRegisters(ucDevAddr, 0x30, 8, (char*)&stcTime);
}
void CJY901::GetAcc()
{
 readRegisters(ucDevAddr, 0x34, 6, (char *)&stcAcc);
}
void CJY901::GetGyro()
{
 readRegisters(ucDevAddr, 0x37, 6, (char *)&stcGyro);
}

void CJY901::GetAngle()
{
 readRegisters(ucDevAddr, 0x3d, 6, (char *)&stcAngle);
}
void CJY901::GetMag()
{
 readRegisters(ucDevAddr, 0x3a, 6, (char *)&stcMag);
}
void CJY901::GetPress()
{
 readRegisters(ucDevAddr, 0x45, 8, (char *)&stcPress);
}
void CJY901::GetDStatus()
{
 readRegisters(ucDevAddr, 0x41, 8, (char *)&stcDStatus);
}
void CJY901::GetLonLat()
{
 readRegisters(ucDevAddr, 0x49, 8, (char *)&stcLonLat);
}
void CJY901::GetGPSV()
{
 readRegisters(ucDevAddr, 0x4d, 8, (char *)&stcGPSV);
}
void CJY901::GetQ()
{
 readRegisters(ucDevAddr,0x51,8,(char*)&stcSQ);
}
CJY901 JY901 = CJY901();
# 1 "e:\\飞总\\电赛\\电赛\\main.ino"
# 2 "e:\\飞总\\电赛\\电赛\\main.ino" 2
# 3 "e:\\飞总\\电赛\\电赛\\main.ino" 2
# 4 "e:\\飞总\\电赛\\电赛\\main.ino" 2
# 5 "e:\\飞总\\电赛\\电赛\\main.ino" 2
# 6 "e:\\飞总\\电赛\\电赛\\main.ino" 2
# 7 "e:\\飞总\\电赛\\电赛\\main.ino" 2
# 8 "e:\\飞总\\电赛\\电赛\\main.ino" 2
# 9 "e:\\飞总\\电赛\\电赛\\main.ino" 2
# 10 "e:\\飞总\\电赛\\电赛\\main.ino" 2
# 11 "e:\\飞总\\电赛\\电赛\\main.ino" 2
#define Debug 1
#define LEDPIN 27
float Yaw_angle, Roll_angle, Pitch_angle;
float gx, gy, gz, ax, ay, az;
float g_vx = 0, g_vy = 0, g_vz = 0, posx = 0, posy = 0, posz = 0;
float raw_altitude, altitude;
float v_hight;
float grand_ax, grand_ay, grand_az;
float expect_h = 20;
float q0, q1, q2, q3;
int out1, out2, out3, out4;
float ex_roll, ex_pitch, ex_yaw;

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
//void task_hpos(void *pvParameters);

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
  ({ do { if (__builtin_constant_p(!(((((0x3ff48000 + 0xd4))) >= 0x3ff00000) && (((0x3ff48000 + 0xd4))) <= 0x3ff13FFC)) && !(!(((((0x3ff48000 + 0xd4))) >= 0x3ff00000) && (((0x3ff48000 + 0xd4))) <= 0x3ff13FFC))) { extern __attribute__((error("(Cannot use WRITE_PERI_REG for DPORT registers use DPORT_WRITE_PERI_REG)"))) void failed_compile_time_assert(void); failed_compile_time_assert(); } (("(Cannot use WRITE_PERI_REG for DPORT registers use DPORT_WRITE_PERI_REG)" && (!(((((0x3ff48000 + 0xd4))) >= 0x3ff00000) && (((0x3ff48000 + 0xd4))) <= 0x3ff13FFC))) ? (void)0 : __assert_func ("e:\\飞总\\电赛\\电赛\\main.ino", 50, __PRETTY_FUNCTION__, "\"(Cannot use WRITE_PERI_REG for DPORT registers use DPORT_WRITE_PERI_REG)\" && (!(((((0x3ff48000 + 0xd4))) >= 0x3ff00000) && (((0x3ff48000 + 0xd4))) <= 0x3ff13FFC))")); } while(0);; (*((volatile uint32_t *)((0x3ff48000 + 0xd4)))) = (uint32_t)(0); }); //关闭低电压检测,避免无限重启
  xTaskCreatePinnedToCore(Task1, "Task1", 15000, 
# 51 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                                __null
# 51 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                    , 2, 
# 51 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                                         __null
# 51 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                             , 0); //最后一个参数至关重要，决定这个任务创建在哪个核上.PRO_CPU 为 0, APP_CPU 为 1,或者 tskNO_AFFINITY 允许任务在两者上运行.
  xTaskCreatePinnedToCore(Task2, "Task2", 10000, 
# 52 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                                __null
# 52 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                    , 1, 
# 52 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                                         __null
# 52 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                             , 1);
  xTaskCreatePinnedToCore(vTask3, "rate", 7000, 
# 53 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                               __null
# 53 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                   , 1, 
# 53 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                                        __null
# 53 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                            , 1);
  xTaskCreatePinnedToCore(vTask4, "pos", 10000, 
# 54 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                               __null
# 54 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                   , 3, 
# 54 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                                        __null
# 54 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                            , 1);
  xTaskCreatePinnedToCore(task_high, "high_pid",20000, 
# 55 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                                      __null
# 55 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                          , 1, 
# 55 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                                               __null
# 55 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                                   , 1);
  xTaskCreatePinnedToCore(task_led, "led_blink", 1024, 
# 56 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                                      __null
# 56 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                          , 2, 
# 56 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                                               __null
# 56 "e:\\飞总\\电赛\\电赛\\main.ino"
                                                                   , 0);
  //vTaskStartScheduler();
  //实现任务的函数名称（task1）；任务的任何名称（“ task1”等）；分配给任务的堆栈大小，以字为单位；任务输入参数（可以为NULL）；任务的优先级（0是最低优先级）；任务句柄（可以为NULL）；任务将运行的内核ID（0或1）
}

void loop()
{
}

void Task1(void *pvParameters)
{
  static TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1;
  bool ifreaded = 0;
  int time1 = 0;
  imu_init();
  KFP KFP_height = {0.02, 0, 0, 0, 0.001, 0.543};
  //unsigned portBASE_TYPE uxHighWaterMark;//
  xLastWakeTime = xTaskGetTickCount();
  Serial.println("start");
  //在这里可以添加一些代码，这样的话这个任务执行时会先执行一次这里的内容（当然后面进入while循环之后不会再执行这部分了）
  EEPROM.readBytes(0, buff, lenth);
  uint16_t hposcnt = 0;
  esc_write(1000, 1000, 1000, 1000);
  delay(1);
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

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
      Reset_pose_i(); //清空积分项
    }
    if (ifreaded)
    {
      EEPROM.readBytes(0, buff, lenth);
      memcpy(&pid, buff, lenth);
      copy(pid);
      ifreaded = 0;
    }
    gethight(&raw_altitude, &v_hight);
    //vTaskDelay(1);
    altitude = raw_altitude * cos(Roll_angle * 3.14159 / 180) * cos(Pitch_angle * 3.14159 / 180); //////高度修正存在问题！！！！！！！！！！！！！
    //altitude = kalmanFilter(&KFP_height, raw_altitude);
    //Serial.printf("%f   %f    %f     \n",pid.p1[0],pid.p2[1],pid.i1[2]);
    //////////////////////////////////read sensor///////////////////////////////
    //Serial.printf("vz %f\n",v_hight);
    //Serial.printf("%f,%f,%f\n", Roll_angle, Pitch_angle, Yaw_angle);
    //getquater();
    getdata();

    hposcnt++;
    if (hposcnt % 25 == 0)
      opt_co(); //25毫秒执行一次//////光流pid异常
    if (hposcnt == 75)
    {
      hposcnt = 0;
      opt_get();
    }
    //Serial.printf("%.7f,\n",grand_az);
    //Serial.print(Roll_angle);
    //Serial.print(',');
    //Serial.print(Pitch_angle);
    //Serial.print('\n');



//Serial.printf("%f %f  %f    %f  %f  %f \n",pid.p1[0],pid.p1[1],pid.p1[2]);
//Serial.printf("alti:%f",altitude);
//Serial.printf("%f   %f   %f   %f   %f  %f   %f  %f   %f   %f   %f\n",Yaw_angle, Roll_angle, Pitch_angle,gx, gy, gz,altitude,q0,q1,q2,q3);

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
    vTaskDelay(1);
    fly_handle();
    if (pid.startflag)
      esc_write(out1, out2, out3, out4);
    else
      esc_write(1000, 1000, 1000, 1000);


//Serial.printf("ax,zy,zy: %f   %f   %f \n",ax,ay,az);
//Serial.printf("q0,q1,q2,q3: %f   %f  %f  %f \n",q0,q1,q2,q3);
//Serial.printf("%d,%d,%d,%d\n",out1,out2,out3,out4);
//Serial.printf("dt :%lf" ,dt);

  }
}
//每10次系统节拍执行一次
void vTask3(void *pvParameters)
{
  static TickType_t xLastWakeTime;
  const TickType_t xFrequency = 2;

  // 使用当前时间初始化变量xLastWakeTime
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    //等待下一个周期
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    rate_pid();
    // 需要周期性执行代码放在这里
  }
}
void vTask4(void *pvParameters)
{
  static TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5;
  long now = micros();
  bool flag = 0;
  unsigned int uxPriority;
  xLastWakeTime = xTaskGetTickCount();
  bool state = 0;
  // 使用当前时间初始化变量xLastWakeTime

  for (;;)
  {
    if (!flag)
    {
      uxPriority = uxTaskPriorityGet(
# 205 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                                    __null
# 205 "e:\\飞总\\电赛\\电赛\\main.ino"
                                        );
      vTaskPrioritySet(
# 206 "e:\\飞总\\电赛\\电赛\\main.ino" 3 4
                      __null
# 206 "e:\\飞总\\电赛\\电赛\\main.ino"
                          , (uxPriority - 2));
      flag = 1;
    }

    //等待下一个周期
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    posturepid();
    //Serial.printf("%f   %f\n",Roll_angle,Pitch_angle);
    digitalWrite(15, state);
    state = !state;

    // 需要周期性执行代码放在这里
  }
}
//每10次系统节拍执行一次
void task_high(void *pvParameters)
{
  static TickType_t xLastWakeTime;
  const TickType_t xFrequency = 2;
  uint16_t cnt = 0;
  // 使用当前时间初始化变量xLastWakeTime
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    //等待下一个周期
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //Serial.printf("running");
    //Serial.println(cnt);
    if (cnt % 6 == 0)
      heightcontrol();
    if (cnt % 2 == 0)
      hight_v();
  /*   if (cnt % 2 == 0)

      {hight_acc();}  */
# 240 "e:\\飞总\\电赛\\电赛\\main.ino"
    if (cnt++ == 120)
      cnt = 0;

    //Serial.printf("%f ,%f ,%f ,%d ,%f ,%f\n", expect_h, altitude, grand_az, hight1_out, h2.Control_OutPut, h3.Control_OutPut);
  }
}

void task_led(void *pvParameters)
{
  static TickType_t xLastWakeTime;
  const TickType_t xFrequency = 200;
  pinMode(15, 0x01);
  xLastWakeTime = xTaskGetTickCount();
  short led_flag;
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    if (fly_flag == 2)
      digitalWrite(27, 1);
    else
      digitalWrite(27, 0);
  }
}

/* void task_hpos(void *pvParameters)

{

  static portTickType xLastWakeTime;

  const portTickType xFrequency = 50;

  xLastWakeTime = xTaskGetTickCount();

  bool state = 0;

  Serial.println("here2");

  while (1)

  {



    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    Serial.println("here1");

    //opt_control();

    if (!state)

      opt_get();

    state = !state;

    opt_co();

  }

} */
