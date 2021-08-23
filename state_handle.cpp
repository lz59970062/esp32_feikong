extern float alt_init;
extern float altitude;
extern bool Init_flag;
extern bool Flying_flag;
extern bool Landing_flag;
extern bool Start_flag;
extern bool Lowpower_flag;
extern bool Landed_flag;
#include <Arduino.h>
extern uint8_t fly_flag;

#define LEDPIN 27
#define BUTTON 34
#define adcPin 15

void interrupt_handle(){
    static long pretime=0;
    static bool flag=0;
    if (flag==0) {
        flag=1;
        pretime=millis();
    }
    if(flag){
        if (millis()-pretime<20){
            Start_flag=1;
            //fly_flag=2;
        }
        flag=0;
    }
    
}
long pt=0;
void led_init(){
    
    pinMode(LEDPIN,OUTPUT);
    pinMode(BUTTON,INPUT);
    attachInterrupt(BUTTON,interrupt_handle,RISING);
}
void fly_handle(){
     if (altitude-alt_init>2){
         Landed_flag=0;
         Flying_flag=1;
     }
     
     else if(altitude<=alt_init&&Flying_flag==1){
      
        Landing_flag=1;

     }
     //Serial.printf("%f,%f\n",alt_init,altitude);
}


void voltage_handle(){
    if(analogRead(adcPin)*0.00374277<3.6) Lowpower_flag=1;
    digitalWrite(15,1);
}
