#ifndef RENDERER_H
#define RENDERER_H
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include "Object.h"

class Renderer {
public:
    void render(const Object& Object, cv::Mat& frame);
    void setScreenSize(float width, float height);
private:
    Eigen::Vector2d screenSize;
    float scaleFactor; // a diferenca do tamanho do mundo modelado pro tamanho da 'tela', O mundo modelado tem o x que varia entre 0 e 4.680, a tela vai de 0 a 720p
};
#endif
