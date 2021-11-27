#ifndef OBJECT_H
#define OBJECT_H
#include <opencv2/core.hpp>
#include <Eigen/Dense>
// typedef PF<double, 3, 2, 2, S> Robot;
class Object {
public:
    Object(float width, float height, float rotation, const Eigen::Vector4d& limits);
    void update(double dt);
    const Eigen::Vector3d& getState() const;
    const cv::Mat& getSprite() const;
private:
    Eigen::Vector3d state; // vetor com 3 dimensoes, {x,y,rotation};
    Eigen::Vector2d speeds; // linear and angular speeds
    Eigen::Vector2d accelerations{1, 1}; // linear and angular accelerations (Dont know if actually needed for this simulation);
    cv::Mat sprite;
};
class Ball : Object {
public:
    Ball(float width, float height, float linearSpeed=0.0001, float angularSpeed=0.0, float linearAcelerationFactor=0.9, float angAccFactor=1);
};
#endif
