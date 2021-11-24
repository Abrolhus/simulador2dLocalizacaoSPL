#ifndef WORLD_H
#define WORLD_H
#include <opencv2/core.hpp>
#include "Robot.h"
#include "Field.h"
#include <bflib/PF.hpp>

class World {
public:
    void update(double dt);
    void render(cv::Mat& frame, float scale) const;
    World(float width, float height, Eigen::Vector2d robotInitialPosition={0.1,0.1}, float robotILinearSpeed=0.01, float robotILAngularSpeed= 0);
private:
    Eigen::Vector4d margins;
    Field field;
    Nao Robot;
};

#endif
