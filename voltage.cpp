#include "voltage.h"
#include <Arduino.h>
extern int lowpower,verylowpower;
float voltage_read;
void adc_init(){
    pinMode(adcPin,INPUT);
}
void adc_operate(){
    voltage_read=analogRead(adcPin)*0.00374277;
    if(voltage_read<low_voltage*3){
        lowpower=1;
    }
    else lowpower=0;
    if(voltage_read<very_low*3){
        verylowpower=1;
    }
    else verylowpower=0;

}
