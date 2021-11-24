#include "Field.h"
using namespace cv;

void drawLines(cv::Mat &image, const vector<Vector4d> &lines, const cv::Scalar &color, float scale, const Vector4d& margins){
    for (int i = 0; i < lines.size(); i++)
    {
        cv::line(image,
                 cv::Point(lines[i](0) * scale + margins[0], lines[i](1) * scale + margins[1]),
                 cv::Point(lines[i](2) * scale + margins[0], lines[i](3) * scale + margins[1]),
                 color, 2);
    }
}

void Field::render(cv::Mat& frame, const cv::Point& position, float scale, const Vector4d& margins) const{ // draws on screen

    //cv::Rect roi( cv::Point(this->limits[0]*scale, this->limits[1]*scale), cv::Point(this->limits[2]*scale, this->limits[3]));
    // desenha o campo com margens. quando for desenhar qualquer objeto (bola, linha) tera que desenhar sempre dps  das margens.
    Mat image = Mat(this->limits[3]*scale + margins[0] + margins[2], this->limits[2]*scale + margins[1] + margins[3], CV_8UC3, this->color); // TODO: add getWidth and getHeight functions
    drawLines(image, this->lines , Scalar(255, 255, 255), 100, margins);
    cv::Rect roi(position, Point(image.cols, image.rows));
    image.copyTo(frame(roi));
}
void Field::update(double dt){
    return;
}

Field::Field(float width, float height, const cv::Scalar& color, const std::vector<Eigen::Vector4d>& lines){
//Field::Field(float width, float height, const cv::Scalar& color, const std::vector<Eigen::Vector4d>& lines){
    this->limits = {0, 0, width, height};
    this->color = color;
    if(!lines.empty()){
        this->lines = lines;
    }
    else{
        this->lines = vector<Vector4d>{Vector4d(0.000, 0.000, 4.680, 0.000),
                               Vector4d(0.000, 0.000, 0.000, 3.200),
                               Vector4d(0.000, 3.200, 4.680, 3.200),
                               Vector4d(4.680, 0.000, 4.680, 3.200),
                               Vector4d(2.340, 0.000, 2.340, 3.200), // linha do meio de campo
                               Vector4d(0.000, 1.010, 0.312, 1.010),
                               Vector4d(0.000, 2.190, 0.312, 2.190),
                               Vector4d(0.312, 1.010, 0.312, 2.190),
                               Vector4d(4.680, 1.010, 4.368, 1.010),
                               Vector4d(4.680, 2.190, 4.368, 2.190),
                               Vector4d(4.368, 1.010, 4.368, 2.190),
                               Vector4d(0.650, 1.600, 0.702, 1.600), // linha horizontal da cruz esquerda
                               Vector4d(0.676, 1.574, 0.676, 1.626),
                               Vector4d(3.978, 1.600, 4.030, 1.600),
                               Vector4d(4.004, 1.574, 4.004, 1.626)};
                            // Vector4d(0.000,  2.280,   0.920,   2.280),
                            // Vector4d(0.920,  2.280,   0.920,   3.200),
                            // Vector4d(4.190,  2.850,   4.680,   2.850),
                            // Vector4d(4.190,  2.850,   4.190,   3.200),
                            // Vector4d(4.030,  0.000,   4.680,   0.650)
    }
}

