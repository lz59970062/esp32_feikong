#include <Wire.h>
#include "pid.h"
#include "PX4Flow.h"
#include <Arduino.h>

#define PARAM_FOCAL_LENGTH_MM 16
extern float altitude;
extern float xout,yout;
long last_check = 0;
int px = 0;
int py = 0;
float focal_length_px = (PARAM_FOCAL_LENGTH_MM) / (4.0f * 6.0f) * 1000.0f;
PX4Flow px4 = PX4Flow();
PID hposx, hposy;
extern k posepid;
void optical_init()
{
    Wire.begin(21, 22);
}
void opt_control()
{
    float x_rate, y_rate, flow_x, flow_y;
    long loop_start = millis();
    static bool initflag = 0;

    if (loop_start - last_check > 50)
    {
        px4.update_integral();
        x_rate = px4.gyro_x_rate_integral() / 10.0f;  // mrad
        y_rate = px4.gyro_y_rate_integral() / 10.0f;  // mrad
        flow_x = px4.pixel_flow_x_integral() / 10.0f; // mrad
        flow_y = px4.pixel_flow_y_integral() / 10.0f; // mrad
        int timespan = px4.integration_timespan();    // microseconds

        int quality = px4.quality_integral();
        if (quality > 100)
        {
            // Update flow rate with gyro rate
            float pixel_x = flow_x + x_rate; // mrad
            float pixel_y = flow_y + y_rate; // mrad

            // Scale based on ground distance and compute speed
            // (flow/1000) * (ground_distance/1000) / (timespan/1000000)
            float velocity_x = (float)pixel_x * altitude / timespan; // m/s
            float velocity_y = (float)pixel_y * altitude / timespan; // m/s
            //Serial.printf("%f,%f",velocity_x,velocity_y);
            // Integrate velocity to get pose estimate
            if (!initflag)
            {
                hposx.Set_errormax(2);
                hposx.Set_i_error(3);
                hposx.Set_i_mode(1, 1, 1);
                hposx.Set_Cutoff_Frequency(20, 5, &hposx.Control_Device_Div_LPF_Parameter);
                hposx.Control_OutPut_Limit = 4;
                hposy.Set_errormax(2);
                hposy.Set_i_error(3);
                hposy.Set_i_mode(1, 1, 1);
                hposy.Set_Cutoff_Frequency(20, 5, &hposy.Control_Device_Div_LPF_Parameter);
                hposy.Control_OutPut_Limit = 4;
                initflag = 1;
            }
      
            //Serial.printf(" %f,%f\n",expect_h,altitude);
            hposy.Set_pid(posepid.yp, posepid.yi, posepid.yd);
            hposx.Set_ex_feed(posepid.evx,velocity_y);
            hposy.Set_ex_feed(posepid.evy, velocity_x);
            hposx.Set_pid(posepid.xp, posepid.xi, posepid.xd);
            xout = (int)hposx.PID_Control_Div_LPF();
            yout= (int)hposy.PID_Control_Div_LPF();
            posepid.expect[roll]=xout;
            posepid.expect[pitch]=yout;

        }
    }
    last_check = loop_start;
}