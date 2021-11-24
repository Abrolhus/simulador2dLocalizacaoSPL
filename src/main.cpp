
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "Field.h"
#include "Robot.h"
#include "World.h"


using namespace cv;
//void render(Mat& frame, const Field& field){
    //field.render(frame, Point(0,0), 100, Eigen::Vector4d{20, 20, 20, 20});
//}
int main(){
    Mat frame(480, 720, CV_8UC3);

    // limites do campo (modelo), posicao do robo, velocidadeLinear, velocidadeAngular
    World world(4.680, 3.200, {0.1, 0.1}, 0.001, PI/5000);
    //render(frame, field);
    while(1){

        world.update(1000.0/30);
        world.render(frame, 100);
        imshow("frame", frame);
        waitKey(1000.0/30); // 30fps (if the computations were instant). TODO: subtract the coputations;
    }
}

