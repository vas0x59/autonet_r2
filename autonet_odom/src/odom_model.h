#pragma ONCE

#include <cmath>


struct OdometryOut {
    float x;
    float y;
    float o;
    float vx;
    float vy;
    float vo;
};


class OdometryModel{
    public:
        OdometryModel(float w_);
        OdometryOut calc(float dt, float dl, float dr);
        void set(float x_, float y_, float o_);
    private:
        float w = 0.34;
        float Dl = 0;
        float Dr = 0;
        float pDr = 0;
        float pDl = 0;
        float d = 0;

        float x = 0;
        float y = 0;
        float o = 0;

        float px = 0;
        float py = 0;
        float po = 0;

        float vx = 0;
        float vy = 0;
        float vo = 0;
};


OdometryModel::OdometryModel(float w_){
    w = w_;
}

OdometryOut OdometryModel::calc(float dt, float dl, float dr){
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


void OdometryModel::set(float x_, float y_, float o_){
    x = x_;
    y = y_;
    o = o_;
}
