/* typedef structs{
    float x,y,z;
}V;
 */

class Kalman{
    double P[4]={1.0,0,0,1.0};
    double R=0.5;
    double H[2]={1,0};
    double Q[4]={1,0,0,1};
    float lp=0,lv=0;
    public:
    void fiter(double dt, double a, double ob_p, float *position, float *velocity);
    void init(float flp,float flv);
};
