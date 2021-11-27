#include "Renderer.h"
using namespace cv;


void Renderer::render(const Object& obj, cv::Mat &frame){
    const Mat& sprite = obj.getSprite();

    Point position(margins[0] + this->model.x(0)*scale, margins[1] + this->model.x(1)*scale); // (x0, y0) + x'*scale + y'*scale


}

