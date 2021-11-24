#include "World.h"

void World::update(double dt){
    this->field.update(dt);
    this->Robot.update(dt);
}
// desenha o robo dps o campo.
void World::render(cv::Mat& frame, float scale) const {
    this->field.render(frame, cv::Point(0,0), scale, this->margins);
    this->Robot.render(frame, scale, this->margins);
}
World::World(float width, float height, Eigen::Vector2d robotInitialPosition, float robotILinearSpeed, float robotILAngularSpeed) :
        Robot(width/30, height/30, robotILinearSpeed, robotILAngularSpeed, Eigen::Vector4d{robotInitialPosition(0), robotInitialPosition(1), width, height}), field(width, height){
    this->margins = {20, 20, 20 , 20};
}
