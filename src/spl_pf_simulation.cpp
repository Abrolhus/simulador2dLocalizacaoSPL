#include <bflib/PF.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#if defined PLOT && !defined PLOT_REALTIME
#include "external/matplotlibcpp.h"
#endif

using namespace std;
#if defined PLOT && !defined PLOT_REALTIME
namespace plt = matplotlibcpp;
#endif

#define PLOT_REALTIME
#ifdef PLOT_REALTIME
#include <opencv2/opencv.hpp>
#endif

//#define CONTROL_P
//#define IGUIM_CONTROL

#define N_POINTS 8
const float PI = 3.14159265358979f;

// Number of sensor readings
const int S = 9;

/*
    The Monte Carlo Filter
    -------------------------
    3 states (x, y and theta)
    2 inputs (linear and angular speeds)
    2 outputs (range and bearing) * S sensors
*/
typedef PF<double, 3, 2, 2, S> Robot;

// Limits of the world
Robot::State minState(0.000, 0.000, 0);
Robot::State maxState(4.680, 3.200, 2 * PI);

// World lines
vector<Vector4d> lines{Vector4d(0.000, 0.000, 4.680, 0.000),
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
/*
    The model function
    -------------------------
    Describes how the state changes according to an input
*/

#ifdef PLOT_REALTIME
cv::Mat image(500, 500, CV_8UC3);
#endif

void model(Robot::State &x, Robot::Input &u, double dt)
{
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

/*
    The sensor function
    -------------------------
    Describes the sensor output based on the current state and an associated data vector
*/
// void sensor(vector<Robot::Output> &y, Robot::State &x, double dt)
// {
//     double eps = 0.001;
//     double pEps = 0.01;
//     double step = (PI / 2.0) / S;
//     double angle;
//     int start = (S / 2);
//     double x1 = x(0), y1 = x(1), th = x(2);
//     double x2, y2, x3, y3, x4, y4, den, px, py, thP, dth, dist, dist_i;

//     for(int i = 0, n = -start; i < S; i++, n++)
//     {
//         angle = n * step;
//         x2 = x1 + cos(th + angle);
//         y2 = y1 + sin(th + angle);
//         dist = -1;

//         for(int j = 0; j < lines.size(); j++)
//         {
//             x3 = lines[j](0);
//             y3 = lines[j](1);
//             x4 = lines[j](2);
//             y4 = lines[j](3);

//             den = ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4));
//             if(den != 0)
//             {
//                 px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4))/den;
//                 py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4))/den;

//                 thP = atan2(py - y1, px - x1) - th;
//                 thP = fmod(thP, PI * 2);
//                 if(thP > PI)
//                     thP -= 2 * PI;

//                 dth = abs(angle - thP);
//                 if(dth < eps)
//                 {
//                     if(min(x3, x4)-pEps <= px && px <= max(x3, x4)+pEps && min(y3, y4)-pEps <= py && py <= max(y3, y4)+pEps)
//                     {
//                         dist_i = sqrt( (x1 - px)*(x1 - px) + (y1 - py)*(y1 - py) );
//                         if(dist < 0)
//                             dist = dist_i;
//                         else if(dist_i < dist)
//                             dist = dist_i;
//                     }
//                 }
//             }
//         }

//         y[i] << dist, angle;
//     }
//}

vector<Vector3d> FeatureMap{Vector3d(10, 10, 1),
                            Vector3d(244, 10, 2),
                            Vector3d(478, 10, 1),

                            Vector3d(10, 111, 1),
                            Vector3d(41, 111, 2),
                            Vector3d(447, 111, 2),
                            Vector3d(478, 111, 1),

                            Vector3d(244, 125, 3),

                            Vector3d(78, 170, 3),
                            Vector3d(244, 170, 3),
                            Vector3d(410, 170, 3),

                            Vector3d(244, 215, 3),

                            Vector3d(10, 229 , 1),
                            Vector3d(41, 229, 2),
                            Vector3d(447, 229, 2),
                            Vector3d(478, 229, 1),

                            Vector3d(10, 330, 1),
                            Vector3d(244, 330, 2),
                            Vector3d(478, 330, 1)
                            };


int pixel2int(float a)
{
    return a*100 + 10;
}


float mod(float a, float m)
{
    return a - m * floor(a / m);
}


float fixAngle(float theta)
{
    float phi;

    phi = std::fmod(theta, 2 * PI);

    if (phi > PI)
    {
        phi = phi - 2 * PI;
    }

    return phi;
}

void DetectFeatures(cv::Mat &img, Robot::State X, vector<Robot::Output> &Y, vector<Vector3d> FeatureMap)
{
    int color=0;
    float distance,bearing,dx,dy;
    cv::Mat image;
    cv::Mat cropped;
    cv::Point2f RobotState,Cropped,Feature;

    RobotState.x = 100*X[0] + 10; // estado x atual do robo
    RobotState.y = 100*X[1] + 10; // estado y atual do robo

    Cropped.x =  (RobotState.x - 110)<0?0:(RobotState.x - 110); // x da origem do crooped no sistema de coordenadas do campo
    Cropped.y =  (RobotState.y - 110)<0?0:(RobotState.y - 110); // y da origem do crooped no sistema de coordenads do campo

    cv::Rect roi = cv::Rect(Cropped.x, Cropped.y, 220, 220); // região de interese em torno do robô que o scanline procura
    cv::rectangle(img, roi, cv::Scalar(0, 255, 0)); // desenha região de interesse em verde

    cropped = cv::Mat(img, roi); // recorta a região de interesse do mapa do campo

    //cv::imshow("cropped", cropped); // descomente para ver a imagem cropped em tempo real

    // OBS: a orientação dos eixo do OPENCV: 1 - origem é no canto superior esquerdo da imagem, 2 - y cresce para baixo e x cresce para direita

    for (int y = 0; y < cropped.rows; y++) // itera nas y colunas da imagem
    {
        for (int x = 0; x < cropped.cols; x++) // itera nas x linhas da imagem
        {
            cv::Vec3b colour = cropped.at<cv::Vec3b>(y, x); // pega a cor BGR da imagem no pixel y , x

            if (colour.val[0] == 0 && colour.val[1] == 255 && colour.val[2] == 255) //se for amarelo
            {
                Feature.x = x + Cropped.x; // translação em x do sistema de coordenadas da cropped para o mapa do campo
                Feature.y = y + Cropped.y; // translação em y do sistema de coordenadas da cropped para o mapa do campo

                dx = Feature.x - RobotState.x; // distancia em x entre feature e robo
                dy = Feature.y - RobotState.y; // distancia em y entre feature e robo

                distance = sqrt(dx*dx + dy*dy); // distance radial entre feature e robo
                bearing = atan2f(dy,dx) - X[2]; // angulo entre orientação do robo e feature
                color++;
                for (int i = 0; i < FeatureMap.size(); i++) // faz uma busca no Feature map
                {

                    if (  ( int(Feature.x) == FeatureMap[i](0) ) && ( int(Feature.y) == FeatureMap[i](1) ) ) // se coordenadas do pixel amarelo (sensor) for igual a algum ponto no feature map
                    {
                        //cout <<" x: " << int(Feature.x)  << " " << FeatureMap[i](0) << " y : " << int(Feature.y)  << " "<< FeatureMap[i](1) << endl;
                        //cout <<" distance: " << distance << " bearing: " << 57.29f*bearing << endl;
                        cv::circle(img, cv::Point(FeatureMap[i](0), FeatureMap[i](1)), 2, cv::Scalar(0,0,255), CV_FILLED); // cria circulo vermelho na feature
                        cv::line(img, RobotState , Feature, cv::Scalar(0, 0, 255, 0), 1, CV_AA, 0); // cria linha em direção a feature

                        Y.push_back({distance,bearing}); // armazena a distancia e o bearing da feature em um vetor de saidas do sensor
                    }
                }
                //imshow("imge",img); // descomente para ver a imagem do campo inteiro
            }
        }
    }
}
void sensor(vector<Robot::Output> &y, Robot::State &x, double dt)
{


}



float hyperbolicSpiral(Robot::State x, float thetaDir, Robot::State goal)
{
    float thetaUp, thetaDown, rhoUp, rhoDown;
    float phi;

    //   gSizeW = 0.2;
    // deW = 2.2;
    // KrW = 0.5;
    float Kr = 0.05, de = 0.44, gSize = 0.02;
    Vector3d p(x(0), x(1), 1), ph(0, 0, 0);

    MatrixXd m_trans(3, 3), m_rot(3, 3);
    m_trans << 1, 0, -goal(0), 0, 1, -goal(1), 0, 0, 1;
    m_rot << cos(-thetaDir), -sin(-thetaDir), 0, sin(-thetaDir), cos(-thetaDir), 0, 0, 0, 1;

    ph = m_rot * m_trans * p;

    thetaUp = atan2((ph(1) - de - gSize), ph(0)) + thetaDir;
    thetaDown = atan2((ph(1) + de + gSize), ph(0)) + thetaDir;
    rhoUp = sqrt(pow(ph(0), 2) + pow((ph(1) - de - gSize), 2));
    rhoDown = sqrt(pow(ph(0), 2) + pow((ph(1) + de + gSize), 2));

    if (ph(1) > gSize)
        phi = thetaUp + PI * (2 - (de + Kr) / (rhoUp + Kr)) / 2;
    else if (ph(1) < -gSize)
        phi = thetaDown - PI * (2 - (de + Kr) / (rhoDown + Kr)) / 2;
    else
        phi = thetaDir;

    return phi;
}

#ifdef PLOT_REALTIME
void drawLines(cv::Mat &image, const vector<Vector4d> &lines, const cv::Scalar &color)
{
    for (int i = 0; i < lines.size(); i++)
    {
        cv::line(image,
                 cv::Point(10 + lines[i](0) * 100, 10 + lines[i](1) * 100),
                 cv::Point(10 + lines[i](2) * 100, 10 + lines[i](3) * 100),
                 color, 2);
    }
}

void drawParticles(cv::Mat &image, const vector<Robot::State> &PS, const cv::Scalar &color)
{
    for (int i = 0; i < PS.size(); i++)
    {
        cv::circle(image, cv::Point(10 + 100 * PS[i][0], 10 + 100 * PS[i][1]), 2, color, CV_FILLED);
    }
}

void drawFeatures(cv::Mat &image, vector<Vector3d> &PS, const cv::Scalar &color)
{
    for (int i = 0; i < PS.size(); i++)
    {
        cv::circle(image, cv::Point(PS[i][0], PS[i][1]), 4, color, CV_FILLED);
    }
}

void drawPath(cv::Mat &image, const Robot::State &XR, const vector<double> &X, const vector<double> &Y, const cv::Scalar &color, bool strip)
{
    int S = min(X.size(), Y.size());
    vector<cv::Point> points(S);
    for (int i = 0; i < S; i++)
    {
        points[i] = cv::Point(10 + 100 * X[i], 10 + 100 * Y[i]);
    }
    if (strip)
    {
        for (int i = 0; i < S - 1; i += 4)
        {
            cv::line(image, points[i], points[i + 1], color, 1);
        }
    }
    else
        cv::polylines(image, points, false, color, 1);
    cv::circle(image, points.back(), 5, color, CV_FILLED);

    cv::Point pf;
    pf.x = (10 + 100 * XR[0]) + 10 * cos(XR[2]);
    pf.y = (10 + 100 * XR[1]) + 10 * sin(XR[2]);
    cv::line(image, points.back(), pf, color, 2);
}

void drawFieldCenter(cv::Mat &image)
{
    cv::circle(image, cv::Point(244,170), 45, cv::Scalar(0,0,0), 2);
}

void drawSensor(cv::Mat &image, const Robot::State &X, const cv::Scalar &color)
{
    cv::Point2f xSnow, xR,xMid;
    float aperture = PI/6;
    float rad = 100;
    xR.x = (10 + 100 * X[0]);
    xR.y = (10 + 100 * X[1]);

    xMid.x = xR.x + rad * cos(X[2]);
    xMid.y = xR.y + rad * sin(X[2]);
    //cv::circle(image,xR,2, color, 0);
    for (int i = 0 ; i < int(aperture*57.29f); i++)
    {
        xSnow.x = xR.x + rad * cos(X[2]  + 0.01745f * i - aperture*0.5);
        xSnow.y = xR.y + rad * sin(X[2]  + 0.01745f * i - aperture*0.5);

        cv::line(image, xR, xSnow, cv::Scalar(0, 255, 255, 0), 4, CV_AA, 0);
        //return cv::line(image,xR,xSnow,cv::Scalar(0,255,255,0), 4,CV_AA,0);
    }
    cv::line(image, xR, xMid, cv::Scalar(0, 0, 0, 0), 1, CV_AA, 0);
}

// void drawSensor(cv::Mat& image, const Robot::State& X, const vector< Robot::Output >& Y, const cv::Scalar& color)
// {
//     cv::Point pt1, ptR;

//     ptR.x = 10 + 100 * X[0];
//     ptR.y = 10 + 100 * X[1];

//     for(int i = 0; i < Y.size(); i++)
//     {
//         pt1.x = 10 + 100 * ( X[0] + Y[i][0] * cos( Y[i][1] + X[2] ) );
//         pt1.y = 10 + 100 * ( X[1] + Y[i][0] * sin( Y[i][1] + X[2] ) );
//         cv::line(image, ptR, pt1, color, 1);
//         cv::circle(image, pt1, 3, color, CV_FILLED);
//     }

// }
#endif

int main(int argc, char *argv[])
{
    // Defines the standard deviations for the resample and the sensor
    double sigma_x_x = 0.04;
    double sigma_x_y = 0.04;
    double sigma_x_a = 0.30;
    double sigma_y_r = 0.10;
    double sigma_y_b = 0.00;

    // Number of particles
    int N = 1000;
    int Nmin = 100;
    int Ni = 1;
    float error_lim = 0.01;
    float error = 0;
    float dx, dy;

    // Create a new monte carlo filter for the robot with max of 1000 particles
    Robot pf(N, minState, maxState);

    // Sets the system functions
    pf.setModel(model);
    pf.setSensor(sensor);

    // Initialize the system random engine
    pf.seed();

    // Sets the resample std
    Robot::ResampleStd Q;
    Q << sigma_x_x,
        sigma_x_y,
        sigma_x_a;
    pf.setQ(Q);

    // Sets the sensor std
    Robot::SensorStd R;
    R << sigma_y_r,
        sigma_y_b;
    pf.setR(R);

    // Variables to hold the system state, the predicted state and the input x(0.676f, 1.600f, -PI/4 - PI/2); x(2.000f, 1.000f, PI/4);

    //Robot::State x(0.676f, 1.600f, -PI/4 - PI/2);
    Robot::State x(1.676f, 1.600f, -PI/3 - PI/2);
    Robot::State xP;
    Robot::Input u;
    cv::circle(image, cv::Point(x[0]*100+10, x[1]*100+10), 5, cv::Scalar(255, 255, 0), -1);
    cv::imshow("imagem", image);
    cv::waitKey();

    // Sensor readings
    vector<Robot::Output> y(S);

    // Initializes the input variable (linear speed = 1.0f m/s ; angular speed = 0.2f rad/s)
    u << 1.00f, 1.57f;
    // u << 0, 0;

    // Auxiliary variables to plot
    vector<double> X, Y, XP, YP;

    // Defines the simulation (3s of duration, 0.01s for sample time)
    double T = 100;
    double dt = 0.01;

    // Realtime plot initialization
    // xF(0) = 4.680 - 0.676;
    // xF(1) = 1.600;
    //xF(2) = PI/4;

#define CONTROL_P
#ifdef CONTROL_P
    float ex, ey, eth, dp, gamma, alpha, beta, v = 0.104f, w = 1.57f;
    float eps = 0.01;
    float n = 0.2, nk = 0;
    Robot::State xF(n * 2.340f, n * 1.600f, PI / 4);
    float k[3] = {0.22, 0.62, -0.12};
    //cv::circle(
    //float k[3] = {0.22, 0.62, -0.12};
    float kp = 1, ki = 0.01, kd = 0.01;
    float Alpha = 0, dAlpha = 0;

#endif

#ifdef IGUIM_CONTROL
    float theta, v = 0.104f, lastAlpha = 0, alpha, beta, ex, ey, gamma, kp = 1, kd = 0.0f;
    float eps = 0.01;
    float n = 0.2, nk = 0;
    Robot::State xF(0.312f, 2.190f, 0);
#endif

    // Run the simulation
    double t = 0;
    while (t < T)
    {
// Simulate one frame to get the sensor readings
// This is not necessary on a real system as the y vector will come from a real sensor
#ifdef CONTROL_P
        // float a,b;
        // a = mod(3*PI/2,2*PI);
        // b = fixAngle(3*PI);
        // cout << "mod:" << a << "fix:" << b << endl;

        ex = xF(0) - x(0);
        ey = xF(1) - x(1);

        dp = sqrt(ex * ex + ey * ey);
        // P+=dp*dt;
        //cout << "erro = " << dp << "   "<< "Ierro=" << P << endl;
        eth = fixAngle(xF(2) - x(2));

        if ((abs(dp) > eps)) //&&  )
        {
            gamma = atan2(ey, ex);
            alpha = fixAngle(gamma - x(2));
            beta = fixAngle(xF(2) - gamma);
            //if(nk<5)
            //{
            //u(0) = v;
            // u(1) = k[1]*alpha;
            // }

            if (nk < N_POINTS)
            {
                dAlpha = alpha - dAlpha;
                u(0) = v;                                     // + ki*P
                u(1) = kp * alpha + ki * Alpha + kd * dAlpha; //+ k[2]*beta;
                Alpha += alpha;
                dAlpha = alpha;
            }
            else if (nk == N_POINTS)
            {
                u(0) = min(k[0] * dp, v);
                u(1) = k[1] * alpha + k[2] * beta;
            }
            else
            {
                u(0) = 0.0f;
                u(1) = 0.0f;
            }
            // if((-1*PI < alpha < -1*PI/2)||(PI/2 < alpha < PI))
            // {
            //     u(0) = -1*u(0);
            //     alpha = fixAngle(alpha + PI);
            //     beta = fixAngle(beta + PI);

            // }
            //u(1) = k[1]*alpha + k[2]*beta;
            //cout << alpha << endl;
            //cout << u(1) << endl;
            //}
            x(0) += u(0) * cos(x(2)) * dt;
            x(1) += u(0) * sin(x(2)) * dt;
            //x(2) = hyperbolicSpiral(x,beta);
            x(2) += u(1) * dt;
        }
        else
        {
            nk++;
            if ((nk <= N_POINTS))
            {
                //cout << nk << endl;
                //cout << "Mudando Ponto" << " "  << xF(0) << " " << xF(1) << " " << xF(2) << endl;
                Alpha = 0;
                dAlpha = 0;
                xF(0) = nk * n * 2.340f;
                xF(1) = nk * n * 1.600f;
                xF(2) = -nk * n * PI;
            }
            else if (nk > N_POINTS)
            {
                u(0) = 0.0f;
                u(1) = 0.0f;
            }
        }
        cout << nk << endl;
        //cout << " x "<< xF(0)<< " " << " y " << xF(1) << endl;

#endif

#ifdef IGUIM_CONTROL

        ex = xF(0) - x(0);
        ey = xF(1) - x(1);
        gamma = atan2(ey, ex);
        beta = fixAngle(xF(2) - gamma);

        theta = hyperbolicSpiral(x, beta, xF);
        alpha = theta - x(2);
        alpha = fixAngle(alpha);

        u(0) = -v * fabs((alpha) / PI) + v; // -vDeltaGolfabs(alpha)/limiarTheta
        u(1) = kp * alpha / PI + kd * (alpha - lastAlpha);

        lastAlpha = alpha;

        cout << alpha << endl;

        x(0) += u(0) * cos(x(2)) * dt;
        x(1) += u(1) * sin(x(2)) * dt;
        x(2) += u(1) * dt;

#endif

        // cout << hyperbolicSpiral(x,0.0f) << endl;
        pf.simulate(x, y, u, dt);

        // Run the PF with the sensor readings
        pf.run(xP, y, u, dt);

        // Store the system state and the predicted state
        // On a real system the system state isn't available, just the prediction
        X.push_back(x(0));
        Y.push_back(x(1));
        XP.push_back(xP(0));
        YP.push_back(xP(1));

        // Descomente para varia o número de partículas
        //dx = (x(0) - xP(0));
        //dy = (x(1) - xP(1));
        //error = sqrt(dx*dx + dy*dy);
        //pf.VaryN(error,Nmin,Ni,error_lim,N);
        //cout << "error = " << error << endl;
        //cout << "xP = " << xP << endl;

        // Increment the simulation time
        t += dt;

// Realtime plot
#ifdef PLOT_REALTIME
        image.setTo(cv::Scalar(255, 255, 255));

        drawLines(image, lines, cv::Scalar(0, 0, 0));
        drawFieldCenter(image);
        //drawLines(img, lines, cv::Scalar(0, 0, 0));
        drawFeatures(image, FeatureMap, cv::Scalar(255, 0, 0));
        //drawParticles(image, pf.particles(), cv::Scalar(255, 0, 0));
        drawSensor(image, x, cv::Scalar(0, 255, 0));
        //drawSensor(image, xP, y, cv::Scalar(0, 255, 0));
        drawPath(image, x, X, Y, cv::Scalar(0, 0, 0), false);
        //drawPath(image, xP, XP, YP, cv::Scalar(0, 0, 255), false);
        DetectFeatures(image,x,y,FeatureMap);
        cv::circle(image, cv::Point(x[0]*100+10, x[1]*100+10), 5, cv::Scalar(0, 0, 200), -1);
        cv::circle(image, cv::Point(k[0]*100+10, k[1]*100+10), 5, cv::Scalar(0, 200, 200), -1);
        cv::circle(image, cv::Point(xF(0)*100+10, xF(1)*100+10), 5, cv::Scalar(0, 200, 200), -1);

    //float k[3] = {0.22, 0.62, -0.12};
        cv::imshow("Robot Localization PF", image);
        cv::waitKey((int)(dt * 1000));
#endif
    }

// Static Plot
#ifdef PLOT_REALTIME
    cv::imshow("Robot Localization PF", image);
    cv::waitKey(0);
#endif

#if defined PLOT && !defined PLOT_REALTIME
    vector<double> x_, y_;
    plt::title("Position");
    for (int i = 0; i < lines.size(); i++)
    {
        x_.resize(2);
        y_.resize(2);
        x_[0] = lines[i](0);
        y_[0] = lines[i](1);
        x_[1] = lines[i](2);
        y_[1] = lines[i](3);
        plt::plot(x_, y_, "k");
    }
    for (int i = 0; i < pf.particles().size(); i++)
    {
        x_.resize(1);
        y_.resize(1);
        x_[0] = pf.particles()[i](0);
        y_[0] = pf.particles()[i](1);
        plt::plot(x_, y_, "m.");
    }
    plt::named_plot("Real", X, Y, "b");
    x_.resize(1);
    y_.resize(1);
    x_[0] = X[X.size() - 1];
    y_[0] = Y[Y.size() - 1];
    plt::plot(x_, y_, "bX");
    plt::named_plot("PF", XP, YP, "r");
    x_[0] = XP[XP.size() - 1];
    y_[0] = YP[YP.size() - 1];
    plt::plot(x_, y_, "rX");
    plt::legend();
    plt::show();
#endif

    return 0;
}
