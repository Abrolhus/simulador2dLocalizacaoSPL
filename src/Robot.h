
#ifndef ROBOT_H
#define ROBOT_H
#include <bflib/PF.hpp>
#include <iostream>
#include <vector>
#include <cmath>

const float PI = 3.14159265358979f;
const int S = 9; // number of sensors reading
typedef PF<double, 3, 2, 2, S> Robot;

class RobotModel {
    Robot::State x; // 3x1 matrix
    //std::vector<float> x;
    float speed = 0;
    // float dt;
};
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

class Nao {
private:
    RobotModel model;
    // Limits of the world
    Robot::State minState = Robot::State(0.000, 0.000, 0);
    Robot::State maxState = Robot::State(4.680, 3.200, 2 * PI);
    Controle controle;
    // detectBall();
    // detectFeatures();

    // sensors;

};

class Controle {
    controleProporcional(x, u, dt);

}
#endif
