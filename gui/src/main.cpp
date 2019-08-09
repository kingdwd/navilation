#include <iostream>
#include <string>
#include <opencv4/opencv2/core.hpp>


#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <thread>
#include <chrono>
#include <Spline.hpp>
#include "view.hpp"
#include "UU.h"
#include "data.h"
#include "MpcModel.hpp"
#include "grampc.hpp"
#include "grampc_util.h"
#include <QApplication>
#include <QTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "U.h"

#include "ViewBuilder.cpp"
#include "viewModel.hpp"


using namespace std;
using namespace epi;
using Clock = std::chrono::high_resolution_clock ;
using namespace cv;
using namespace std::literals;
using Trajectory = std::vector<State>;

auto stepSize = 0.01s;
auto tol_xy = 5;
auto tol_phi = 0.1;
auto car = std::make_shared<Vehicle>(std::make_unique<DynamicModel>(System(State{0,0,0,0,0}, stepSize.count())));
enum class Mode{
    AUTO, MANUAL
};
auto mode = Mode::AUTO;
enum Key{
    none = -1,
    j = 106,
    k = 107,
    l = 108,
    u = 117,
    i = 105,
    o = 111,
    h = 104,
    oe = 246
};

bool waitUntilEscape(grampc::Grampc& mpc, const State& xdes){
    Clock::time_point t1 = Clock::now();
    int key = waitKey(1);
    cout<< "soll wert: " << xdes<< "\n";

    auto state = car->state;
    if(std::isnan(xdes[0]) ||
         std::isnan(xdes[1]) ||
         std::isnan(xdes[2]) ||
         std::isnan(xdes[3]) ||
         std::isnan(xdes[4]) ){
       std::cout<<"warning: at least one value for xdes is nan. Replacing by current value \n";
       //xdes = state;
    }
    state.val[2] = std::fmod(state.val[2], M_PI);
    auto error =state-xdes;
    auto l2_xy = sqrt(error.val[0]*error.val[0] + error.val[1]*error.val[1]);
    auto l2_phi = sqrt(error.val[2]*error.val[2]);

    if(l2_xy > tol_xy || l2_phi > tol_phi){
        mpc.setparam_real_vector("xdes", xdes.val);
        cout<<"Error is : ["<< l2_xy <<","<< l2_phi <<"]\n";
        mpc.run();
        double u_F = mpc.getSolution()->unext[0];
        double u_phi = mpc.getSolution()->unext[1];
        //cout << "Calculated u: [" << u_F << "; " << u_phi << "] \n";
        car->drive(u_F, u_phi);
        State estim{mpc.getSolution()->xnext};

        //cout << "estimation x: [" << estim << "] \n";
        //cout << "workspace x: [" << workspace << "] \n";
        cout << "diff x: [" << estim - car->state << "] \n";

        mpc.setparam_real_vector("x0", car->state.val);
    }
    else {
        return false;
    }

    Clock::time_point t2 = Clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 );

    auto cycle = stepSize - duration;
    if(cycle.count() > 0)
        std::this_thread::sleep_for(stepSize - duration);
    cout <<"\n millis: state " << cycle.count() << "\n";

    return true;
}

int waitUntilEscapeInManual(){

    Clock::time_point t1 = Clock::now();
    int key = waitKey(5);
    switch (key) {
        case Key::none:
            car->drive(0, 0);
            break;
        case Key::i:
            car->drive(1, 0);
            break;
        case Key::k:
            car->drive(-1, 0);
            break;

        case Key::u:
            car->drive(1, 1);
            break;
        case Key::j:
            car->drive(-1, 1);
            break;

        case Key::o:
            car->drive(1, -1);
            break;
        case Key::l:
            car->drive(-1, -1);
            break;

        case Key::h:
            car->drive(0, 1);
            break;
        case Key::oe:
            car->drive(0, -1);
            break;
    }
    Clock::time_point t2 = Clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 );

    std::this_thread::sleep_for(stepSize - duration);
    return key != 27;
}
//int main(int argc, char** args)
//{
//    //ViewBuilder view(car);
//    grampc::ProblemDescription *model = new MpcModel();
//    grampc::Grampc mpc(model);
//
//    //view.show();
//    ////createTrackbar("hi", "Display window", &pos, 100, show);
//    //while(waitUntilEscape()){} // Wait for a keystroke in the window
//    return 0;
//
//}

Trajectory createTraj(const spline::Points& spline, const double vMax = 150){
    Trajectory traj;
    constexpr auto minX = 1e-6;
    auto splineSize = spline.size();
    auto xLast = spline[splineSize-1][0];
    auto yLast = spline[splineSize-1][1];
    auto v = vMax;
    /* last 10% */
    auto last10 = splineSize*0.85;
    for(int i=0; i<splineSize-1; i++){
        auto x = spline[i][0];
        auto y = spline[i][1];
        auto dx = spline[i+1][0]-x;
        auto dy = spline[i+1][1]-y;
        double alpha;
        if(dx > minX){
            alpha = atan(dy/dx);
        } else if(dx<-minX){
            alpha = U::Math::sgn(dy)*M_PI + atan(dy/dx);
        } else {
            if(traj.empty()) {
                continue;
            }
            alpha = traj[i-1][2];
        }
        if(i>last10){
            auto dist = sqrt(pow(xLast-x,2) + pow(yLast-y,2));
            if(dist < 2*v){
                v = sqrt(dist);
            }
        }
        traj.push_back(
            {spline[i][0]
            , spline[i][1]
            , alpha
            , v
            , 0
            });
    }
    /*last element*/
    traj.push_back({
        spline[splineSize-1][0],
        spline[splineSize-1][1],
        traj[traj.size()-1][2],
        0,
        0
    });
    return traj;
}


int doit(const spline::Points& path){

    ViewBuilder view(car);
    //view.setPath(path);

    Trajectory traj = createTraj(path);
    for(int i=0; i<path.size(); i++ ){
        cout<<"path: " << path[i] <<"\n";
        cout<<"Trajectory: " << traj[i] <<"\n";
    }
    constexpr typeInt NX = MpcModel::X_DIM;
    constexpr typeInt NU = MpcModel::U_DIM;

    typeRNum FINAL_STATE_COST[NX] = {8,8,8,124,1};
    typeRNum STATE_COST[NX] = {8,8,128,16,1};
    typeRNum INPUT_COST[NX] = {1,131072};

    grampc::ProblemDescription *model = new MpcModel(FINAL_STATE_COST, STATE_COST, INPUT_COST);
    grampc::Grampc mpc(model);

    /********* Parameter definition *********/
    /* Initial values and setpoints of the states, inputs, parameters, penalties and Lagrangian mmultipliers, setpoints for the states and inputs */
    ctypeRNum x0[NX] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

    //State xdes = { 500, -200.0, 0, 0.0, 0.0 };

    //Trajectory traj = createCircle(200);

    /* Initial values, setpoints and limits of the inputs */
    ctypeRNum u0[NU] = { 0.0, 0.0 };
    ctypeRNum udes[NU] = { 0.0, 0.0 };
    ctypeRNum umax[NU] = { 1.0, 0.5 };
    ctypeRNum umin[NU] = { -1.0, -0.5 };

    /* Time variables */
    ctypeRNum Thor = 0.1;  /* Prediction horizon */

    ctypeRNum dt = stepSize.count(); /* Sampling time */
    typeRNum t = 0.0;              /* time at the current sampling step */

    /********* Option param definition *******/
    ctypeRNum ConstraintsAbsTol[1] = {1};

    ctypeInt MaxGradIter = 60;
    ctypeInt MaxMultIter =1;
    ctypeInt Nhor = 20;

    ctypeRNum PenaltyMin = 40000;
    ctypeRNum LineSearchMax = 3;




    /********* set parameters *********/
    mpc.setparam_real_vector("x0", x0);
    mpc.setparam_real_vector("u0", u0);
    mpc.setparam_real_vector("udes", udes);
    mpc.setparam_real_vector("umax", umax);
    mpc.setparam_real_vector("umin", umin);

    mpc.setparam_real("Thor", Thor);

    mpc.setparam_real("dt", dt);
    mpc.setparam_real("t0", t);

    /********* set options *********/
    mpc.setopt_int("Nhor", Nhor);
    mpc.setopt_int("MaxGradIter", MaxGradIter);
    mpc.setopt_int("MaxMultIter", MaxMultIter);

    //mpc.setopt_real("PenaltyMin", PenaltyMin);
    //mpc.setopt_real("LineSearchMax", LineSearchMax);

    mpc.setopt_real_vector("ConstraintsAbsTol", ConstraintsAbsTol);
    //mpc.setopt_string("ConvergenceCheck", "on");
    //grampc_estim_penmin(mpc.grampc_, 1);


    view.show();
    //createTrackbar("hi", "Display window", &pos, 100, show);
    if(mode == Mode::AUTO) {
        for(int i=0; i<traj.size(); i++){
            while (waitUntilEscape(mpc, traj[i]) && i !=traj.size()-1) {} // Wait for a keystroke in the window
        }
    } else {
        while (waitUntilEscapeInManual()) {} // Wait for a keystroke in the window
    }
    return 0;
}

int main(int arg0, char** args)
{

    QApplication app(arg0, args);
    View view(std::make_unique<ViewModel>());

    spline::Points points;
    points.push_back({0,0});
    points.push_back({1,0});
    //points.push_back({150,-50});
    points.push_back({300,0});
    points.push_back({450,-50});
    points.push_back({600,-500});
    points.push_back({601,0});

    spline::Points spline = spline::interpol(points);


    app.exec();
    return doit(spline);


}

