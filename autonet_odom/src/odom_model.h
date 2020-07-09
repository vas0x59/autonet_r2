#pragma ONCE

#include <cmath>


struct OdometryOut {
    double x;
    double y;
    double o;
    double vx;
    double vy;
    double vo;
};


class OdometryModel{
    public:
        OdometryModel();
        OdometryModel(double w_);
        OdometryOut calc(double dt, double dl, double dr);
        void set(double x_, double y_, double o_);
    private:
        double w = 0.34;
        double Dl = 0;
        double Dr = 0;
        double pDr = 0;
        double pDl = 0;
        double d = 0;

        double x = 0;
        double y = 0;
        double o = 0;

        double px = 0;
        double py = 0;
        double po = 0;

        double vx = 0;
        double vy = 0;
        double vo = 0;
};

OdometryModel::OdometryModel(){
    w = 0;
}


OdometryModel::OdometryModel(double w_){
    w = w_;
}

OdometryOut OdometryModel::calc(double dt, double dl, double dr){
    OdometryOut out;
    Dr = dr;
    Dl = dl;
    o += (Dr - Dl) / w / 2;
    d = (Dr + Dl) / 2;
    if (o > M_PI)
        o -= 2.0 * M_PI;
    if (o < -1.0 * M_PI)
        o += 2.0 * M_PI;
    
    x += d * cos(o);
    y += d * sin(o);

    px = x;
    py = y;
    po = o;

    vx = (x - px) / dt;
    vy = (y - py) / dt;
    vo = (o - po) / dt;
    
    pDr = Dr;
    pDl = Dl;

    out.x = x;
    out.y = y;
    out.o = o;
    out.vx = vx;
    out.vy = vy;
    out.vo = vo;

    return out;
}


void OdometryModel::set(double x_, double y_, double o_){
    x = x_;
    y = y_;
    o = o_;
}
