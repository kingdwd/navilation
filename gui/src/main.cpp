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
#include "grampc_util.h"

#include "U.h"
#include "ViewBuilder.cpp"

using namespace std;
using Clock = std::chrono::high_resolution_clock ;
using namespace cv;
using namespace std::literals;

auto stepSize = 0.01s;
auto tol_xy = 5;
auto tol_phi = 0.15;
auto car = std::make_shared<epi::Vehicle>(std::make_unique<epi::DynamicModel>(epi::System(epi::State{0,0,0,0,0}, stepSize.count())));
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
bool waitUntilEscape(grampc::Grampc&& mpc, epi::State xdes){
    Clock::time_point t1 = Clock::now();
    int key = waitKey(5);
    //cout<< "pressed key: " << key << "\n";

    if(mode == Mode::MANUAL) {
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
    }
    else {
        auto state = car->state;
        state.val[2] = std::fmod(state.val[2], M_PI);
        auto error =state-xdes;
        auto l2_xy = sqrt(error.val[0]*error.val[0] + error.val[1]*error.val[1]);
        auto l2_phi = sqrt(error.val[2]*error.val[2]);

        if(l2_xy > tol_xy || l2_phi > tol_phi){
            cout<<"Error is : "<< error <<"\n";
            mpc.run();
            double u_F = mpc.getSolution()->unext[0];
            double u_phi = mpc.getSolution()->unext[1];
            cout << "Calculated u: [" << u_F << "; " << u_phi << "] \n";
            car->drive(u_F, u_phi);
            epi::State estim{mpc.getSolution()->xnext};
            epi::State workspace{mpc.getWorkspace()->x};

            cout << "estimation x: [" << estim << "] \n";
            cout << "workspace x: [" << workspace << "] \n";
            cout << "diff x: [" << estim - car->state << "] \n";

            mpc.setparam_real_vector("x0", car->state.val);
        }
    }

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

    typeRNum FINAL_STATE_COST[NX] = {10,10,100,1,1};
    typeRNum STATE_COST[NX] = {10,10,100,1,1};
    typeRNum INPUT_COST[NX] = {1,100};
    grampc::ProblemDescription *model = new MpcModel(FINAL_STATE_COST, STATE_COST, INPUT_COST);
    grampc::Grampc mpc(model);

	/********* Parameter definition *********/
	/* Initial values and setpoints of the states, inputs, parameters, penalties and Lagrangian mmultipliers, setpoints for the states and inputs */
	ctypeRNum x0[NX] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

	epi::State xdes = { 5, -300.0, -2.0, 0.0, 0.0 };

	/* Initial values, setpoints and limits of the inputs */
	ctypeRNum u0[NU] = { 0.0, 0.0 };
	ctypeRNum udes[NU] = { 0.0, 0.0 };
	ctypeRNum umax[NU] = { 1.0, 0.5 };
	ctypeRNum umin[NU] = { -1.0, -0.5 };

	/* Time variables */
	ctypeRNum Thor = 3;  /* Prediction horizon */

	ctypeRNum dt = stepSize.count(); /* Sampling time */
	typeRNum t = 0.0;              /* time at the current sampling step */

    /********* Option param definition *******/
    ctypeRNum ConstraintsAbsTol[1] = {1e-1};

    ctypeInt MaxGradIter = 90;
    ctypeInt MaxMultIter = 3;
    ctypeInt Nhor = 30;




	/********* set parameters *********/
	mpc.setparam_real_vector("x0", x0);
	mpc.setparam_real_vector("xdes", xdes.val);
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

    mpc.setopt_real_vector("ConstraintsAbsTol", ConstraintsAbsTol);
    //mpc.setopt_string("ConvergenceCheck", "on");
    //grampc_estim_penmin(mpc.grampc_, 1);


    view.show();
    //createTrackbar("hi", "Display window", &pos, 100, show);
    while(waitUntilEscape(std::move(mpc), xdes)){} // Wait for a keystroke in the window
    return 0;

}
