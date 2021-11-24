#ifndef FIELD_H
#define FIELD_H
#include <array>
#include <opencv2/core.hpp>
#include <bflib/PF.hpp>

class Field {
public:
    void render(cv::Mat& frame, const cv::Point& position, float scale, const Eigen::Vector4d& margins) const; // draws on screen
    void update(double dt);
    Field(float width, float heigh, const cv::Scalar& color=cv::Scalar(10, 240, 10), const std::vector<Eigen::Vector4d>& lines = {});
private:
    std::array<float, 4> limits; // x0, y0, x1, y1  (canto superior esquerdo e canto inferior direito);
    cv::Scalar color;
    std::vector<Eigen::Vector4d> lines;
};
#endif
