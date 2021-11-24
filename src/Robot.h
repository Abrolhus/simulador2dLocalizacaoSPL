#ifndef ROBOT_H
#define ROBOT_H
#include <bflib/PF.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/core.hpp>

const float PI = 3.14159265358979f;
const int S = 9; // number of sensors reading
typedef PF<double, 3, 2, 2, S> Robot;

class RobotModel {
public:
    Robot::State x = {0,0,0}; // 3x1 matrix
    //std::vector<float> x;
    Robot::Input speeds = {0,0};
    RobotModel(const Robot::State& x, float linearSpeed=0, float angularSpeed=0);
    void update(double dt);
    // float dt;
};
class Nao {
public:
    void render(cv::Mat& frame, float scale, const Vector4d& margins) const; // draws on screen
    void update(double dt);
    Nao(float width, float height, float linearSpeed, float angularSpeed, const Vector4d& limits={}, const Robot::State& x={},  const cv::Mat& visual={});
private:
    Vector2d dimensions;
    cv::Mat visual;
    RobotModel model;
    // Limits of the world
    Robot::State minState = Robot::State(0.000, 0.000, 0);
    Robot::State maxState = Robot::State(4.680, 3.200, 2 * PI);
    //Controle controle;
    // detectBall();
    // detectFeatures();

    // sensors;

};

class Controle {
//    controleProporcional(x, u, dt);

};
#endif

/*
class NAO {

    class RobotModel {
    private:
        State x;
        limitesDoCampo j;
        sensores;
        acelerometro;
        camera;
    }
    Controle control;
    Behavior comportamento;
    Visao visao;
}
*/

