#include <iostream>
#include <string>
#include <opencv2/core.hpp>


#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <thread>
#include <chrono>
#include "UU.h"
#include "data.h"
#include "MpcModel.hpp"
#include "grampc.hpp"

#include "U.h"
#include "ViewBuilder.cpp"

using namespace std;
using Clock = std::chrono::high_resolution_clock ;
using namespace cv;
using namespace std::literals;

auto stepSize = 0.01s;
auto car = std::make_shared<epi::Vehicle>(std::make_unique<epi::DynamicModel>(epi::System(epi::State{0,0,0,0,0}, stepSize.count())));

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
bool waitUntilEscape(grampc::Grampc &mpc){
    Clock::time_point t1 = Clock::now();
    mpc.run();
    int key = waitKey(5);
    //cout<< "pressed key: " << key << "\n";

    //switch (key){
    //    case Key::none: car->drive(0,0); break;
    //    case Key::i: car->drive(1,0); break;
    //    case Key::k: car->drive(-1,0); break;

    //    case Key::u: car->drive(1,1); break;
    //    case Key::j: car->drive(-1,1); break;

    //    case Key::o: car->drive(1,-1); break;
    //    case Key::l: car->drive(-1,-1); break;

    //    case Key::h: car->drive(0,1); break;
    //    case Key::oe: car->drive(0,-1); break;
    //}
    double u_F = mpc.getSolution()->unext[0];
    double u_phi = mpc.getSolution()->unext[1];
    cout<< "Calculated u: [" << u_F << "; " << u_phi << "] \n";
    car->drive(mpc.getSolution()->unext[0], mpc.getSolution()->unext[1]);
    epi::State estim{mpc.getSolution()->xnext};
    epi::State workspace{mpc.getWorkspace()->x};

    cout<< "estimation x: [" <<  estim<< "] \n";
    cout<< "workspace x: [" <<  workspace<< "] \n";
    cout<< "diff x: [" <<  estim-car->state<< "] \n";

    mpc.setparam_real_vector("x0", car->state.val);

    Clock::time_point t2 = Clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 );

    std::this_thread::sleep_for(stepSize - duration);
    //cout <<"\n millis: state " << duration << "\n";

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

int main(int argc, char** args)
{
    ViewBuilder view(car);
    constexpr typeInt NX = MpcModel::X_DIM;
    constexpr typeInt NU = MpcModel::U_DIM;

    typeRNum FINAL_STATE_COST[NX] = {1,1,1,0.1,0.1};
    typeRNum STATE_COST[NX] = {1,1,1,1,1};
    typeRNum INPUT_COST[NX] = {1,5};
    grampc::ProblemDescription *model = new MpcModel(FINAL_STATE_COST, STATE_COST, INPUT_COST);
    grampc::Grampc mpc(model);

	/********* Parameter definition *********/
	/* Initial values and setpoints of the states, inputs, parameters, penalties and Lagrangian mmultipliers, setpoints for the states and inputs */
	ctypeRNum x0[NX] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	ctypeRNum xdes[NX] = { 200.0, -100.0, -1.0, 0.0, 0.0 };

	/* Initial values, setpoints and limits of the inputs */
	ctypeRNum u0[NU] = { 0.0, 0.0 };
	ctypeRNum udes[NU] = { 0.0, 0.0 };
	ctypeRNum umax[NU] = { 1.0, 0.5 };
	ctypeRNum umin[NU] = { -1.0, -0.5 };

	/* Time variables */
	ctypeRNum Thor = 0.05;  /* Prediction horizon */

	ctypeRNum dt = (typeRNum)0.01; /* Sampling time */
	typeRNum t = 0.0;              /* time at the current sampling step */

    /********* Option param definition *******/
    ctypeRNum ConstraintsAbsTol[1] = {1e-2};

    ctypeInt MaxGradIter = 10;
    ctypeInt Nhor = 20;




	/********* set parameters *********/
	mpc.setparam_real_vector("x0", x0);
	mpc.setparam_real_vector("xdes", xdes);
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

    mpc.setopt_real_vector("ConstraintsAbsTol", ConstraintsAbsTol);
    //mpc.setopt_string("Integrator", "ruku45");

    view.show();
    //createTrackbar("hi", "Display window", &pos, 100, show);
    while(waitUntilEscape(mpc)){} // Wait for a keystroke in the window
    return 0;

}
