#include "Object.h"
#include <math.h>
using namespace Eigen;
const float PI = 3.14159265358979f;

const Vector3d& Object::getState() const{
    return this->state;
}
const cv::Mat& Object::getSprite() const{
    return this->sprite;
}
void Object::update(double dt){
    Vector2d& u = this->speeds; // lazy. TODO: replace 'u' with this->speeds
    u = u * Vector2d{pow(this->accelerations(0), dt), pow(this->accelerations(1), dt)}; // may be slow, but does the trick. I dont want the object to decelerate to 0 than start moving backwars (like in the regular v = v0 + at);
    Vector2d dx;
    Vector3d& x = this->state;
    dx << cos(x(2)) * u(0) * dt,
        sin(x(2)) * u(0) * dt,
        u(1) * dt;
    x = x + dx;

    x(2) = fmod(x(2), PI * 2);
    if (x(2) > PI)
        x(2) -= 2 * PI;
    else if (x(2) < -PI)
        x(2) += 2 * PI;
}
