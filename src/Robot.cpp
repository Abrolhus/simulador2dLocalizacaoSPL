#include "Robot.h"
using namespace cv;

void Nao::render(cv::Mat& frame, float scale, const Vector4d& margins) const{ // draws on screen
    //cv::Rect roi( cv::Point(this->limits[0]*scale, this->limits[1]*scale), cv::Point(this->limits[2]*scale, this->limits[3]));
    // desenha o campo com margens. quando for desenhar qualquer objeto (bola, linha) tera que desenhar sempre dps  das margens.
    //Mat image = Mat(this->limits[3]*scale + margins[0] + margins[2], this->limits[2]*scale + margins[1] + margins[3], CV_8UC3, this->color); // TODO: add getWidth and getHeight functions
    //drawLines(this->visual, this->lines , Scalar(255, 255, 255), 100, margins);
    //cv::Rect roi(position, Point(this->visual.cols, this->visual.rows));
    Point position(margins[0] + this->model.x(0)*scale, margins[1] + this->model.x(1)*scale); // (x0, y0) + x'*scale + y'*scale
    circle(frame, position, this->dimensions[0]*scale, Scalar(200, 0, 200), -1);
    //image.copyTo(frame(roi));
}

Nao::Nao(float width, float height, float linearSpeed, float angularSpeed, const Vector4d& limits   ,  const Robot::State& x, const Mat& visual) : model({0,0,0}, 0){
    // sim, essa parte do codigo ta bem feia.
    this->dimensions = {width, height};
    if(limits.size() != 0){
        this->maxState = {limits[2], limits[3], 2*PI};
        this->minState = {limits[0], limits[1], 0};
    }
    if(!visual.empty()){
        this->visual = visual;
    } else {
        this->visual = Mat(this->dimensions[0], this->dimensions[1], CV_8UC3, Scalar(0, 200, 200));
    }
    this->model = RobotModel(Robot::State{this->maxState[0]/2, this->maxState[1]/2, 0}, linearSpeed, angularSpeed);
    if(x.size() != 0){
        this->model.x = x;
    }
}
void Nao::update(double dt){
    this->model.update(dt);
}

////////// RobotModel //////////

// atualiza a posicao do robo de acordo com sua velocidade linear e angular;
void RobotModel::update(double dt){
    Robot::Input u = this->speeds; // lazy. TODO: replace 'u' with this->speeds
    Robot::State dx;
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
RobotModel::RobotModel(const Robot::State& x, float linearSpeed, float angularSpeed) : x(x), speeds(linearSpeed, angularSpeed){
}
