//
// Created by sigi on 12.06.19.
//

#include "kinematic_model.hpp"
#include "vehicle.hpp"
#include "mpc_factory.hpp"
#include "model_const.hpp"
#include <gtest/gtest.h>
#include <math.h>
#include <mpc_model.hpp>

using namespace grampc;
using namespace epi;
using namespace ::testing;
// Define a type which holds an unsigned integer value
template<std::size_t> struct int_{};

template <class Tuple, size_t Pos>
std::ostream& print_tuple(std::ostream& out, const Tuple& t, int_<Pos> ) {
    out << std::get< std::tuple_size<Tuple>::value-Pos >(t) << ',';
    return print_tuple(out, t, int_<Pos-1>());
}

template <class Tuple>
std::ostream& print_tuple(std::ostream& out, const Tuple& t, int_<1> ) {
    return out << std::get<std::tuple_size<Tuple>::value-1>(t);
}

template <class... Args>
std::ostream& operator<<(std::ostream& out, const std::tuple<Args...>& t) {
    out << '(';
    print_tuple(out, t, int_<sizeof...(Args)>());
    return out << ')';
}
static long errorStatus = 99999999999;
class ControllerTest : public TestWithParam<tuple<int, int, double, int, int, int, int>> {
protected:
    virtual void SetUp(){

        typeRNum COST_XY = get<3>(GetParam());
        typeRNum COST_PHI = get<4>(GetParam());
        typeRNum COST_VELO = get<5>(GetParam());
        typeRNum FINAL_STATE_COST[NX] = {COST_XY, COST_XY*10, COST_PHI, 10000};
        typeRNum STATE_COST[NX] = {1,1,1,1};
        typeRNum INPUT_COST[NX] = {1,1};

        auto kinModel = std::make_shared<KinematicCarModel>();
        grampc::ProblemDescription *model = new MpcModel(kinModel, FINAL_STATE_COST, STATE_COST, INPUT_COST);
        mpc = new grampc::Grampc(model);
        createMpc(mpc);
        std::cout<<"params: " << GetParam() << std::endl;

        ///********* Parameter definition *********/
        ///* Initial values and setpoints of the states, inputs, parameters, penalties and Lagrangian mmultipliers, setpoints for the states and inputs */
        //ctypeRNum x0[NX] = { 0.0, 0.0, 0.0, 0.0, 0.0 };


        ctypeRNum xdes[NX] = { desX, desY, desPhi, 0 };
        ///* Initial values, setpoints and limits of the inputs */
        //ctypeRNum u0[NU] = { 0.0, 0.0 };
        //ctypeRNum udes[NU] = { 0.0, 0.0 };
        ctypeRNum umax[NU] = { 1.0, 0.5 };
        ctypeRNum umin[NU] = { -1.0, -0.5 };


        ///********* Option param definition *******/
        ctypeRNum ConstraintsAbsTol[5] = {1,1,0.1,10};

        ctypeInt MaxMultIter = get<0>(GetParam());
        ctypeInt MaxGradIter= get<1>(GetParam());

        ///* Time variables */
        ctypeRNum Thor = get<2>(GetParam());  /* Prediction horizon */
        ctypeInt Nhor = get<6>(GetParam());//Thor/dt;

        /********* set parameters *********/
        mpc->setparam_real_vector("xdes", xdes);
        mpc->setparam_real_vector("umax", umax);
        mpc->setparam_real_vector("umin", umin);

        mpc->setparam_real("Thor", Thor);

        //mpc->setparam_real("dt", dt);
        //mpc->setparam_real("t0", t);

        ///********* set options *********/
        mpc->setopt_int("Nhor", Nhor);
        mpc->setopt_int("MaxGradIter", MaxGradIter);
        mpc->setopt_int("MaxMultIter", MaxMultIter);
        mpc->setopt_string("ConvergenceCheck", "on");
        mpc->setopt_string("ScaleProblem", "on");

        mpc->setopt_real_vector("ConstraintsAbsTol", ConstraintsAbsTol);
        auto initState = State{0,0,0,0,0,0};
        mpc->setparam_real_vector("x0", initState.val);
    }

    double uF, uPhi;
    ctypeRNum dt = epi::STEP_SIZE.count(); /* Sampling time */
    typeRNum t = 0.0;              /* time at the current sampling step */

    constexpr static typeInt NX = MpcModel::X_DIM;
    constexpr static typeInt NU = MpcModel::U_DIM;

    double desX = 50;
    double desY = -100;
    double desPhi = -0.3;

    typeRNum rwsReferenceIntegration[2 * NX];
    grampc::Grampc* mpc;

    Vehicle car{std::make_unique<epi::DynamicCarModel>()};
    long _statusError = 0;
};

TEST_P(ControllerTest, validateOpenLoop){
    //auto& mpc = *mpcP;


    ctypeRNum Tsim = 2;
    typeInt MaxSimIter = (int)(Tsim / 0.01);
    typeInt i;
    typeInt iMPC;
    typeRNum CpuTimeVec[MaxSimIter];
    //epi::State xdes{ 115, -300.0, -2.0, 0.0, 0.0 };
    //mpc.setparam_real_vector("xdes", xdes.val);
    //printf("MPC running ...\n");
    //State state = car.getState();
    //mpc.setparam_real_vector("x0", state.val);
    State error{0,0,0,0,0,0};
    State estimState = error;
    State state = car.getState();
    for (iMPC = 0; iMPC <= MaxSimIter; iMPC++) {
        /* run grampc */
        auto tic = clock();
        mpc->run();
        auto toc = clock();
        CpuTimeVec[iMPC] = (typeRNum)((toc - tic) * 1000 / CLOCKS_PER_SEC);

        /* check solver status */
        if (mpc->getSolution()->status > 0) {
            _statusError += mpc->getSolution()->status;
            if(_statusError > errorStatus) {
                std::cout<<"Status error: " << _statusError << std::endl;
                FAIL();
            }
            //std::cout<<"Status error: " << mpc->getSolution()->status << std::endl;
            //mpc->printstatus(mpc->getSolution()->status, STATUS_LEVEL_WARN);
            //std::cout<< std::endl;
        }

        ///* reference integration of the system via heun scheme since grampc->sol->xnext is only an interpolated value */
        ffct(rwsReferenceIntegration, t, mpc->getParameters()->x0, mpc->getSolution()->unext, mpc->getSolution()->pnext, mpc->grampc_->userparam);
        for (i = 0; i < NX; i++) {
        	mpc->getSolution()->xnext[i] = mpc->getParameters()->x0[i] + dt * rwsReferenceIntegration[i];
        }
        ffct(rwsReferenceIntegration + NX, t + dt, mpc->getSolution()->xnext, mpc->getSolution()->unext, mpc->getSolution()->pnext, mpc->grampc_->userparam);
        for (i = 0; i < NX; i++) {
        	mpc->getSolution()->xnext[i] = mpc->getParameters()->x0[i] + dt * (rwsReferenceIntegration[i] + rwsReferenceIntegration[i + NX]) / 2;
        }
        uF = mpc->getSolution()->unext[0];
        uPhi = mpc->getSolution()->unext[1];
        //std::cout<<"State of car: " << state << "\n";
        estimState = State{mpc->getSolution()->xnext};
        car.drive(uF, uPhi);
        state = car.getState();
        //error = error + cv::norm(estimState - state);
        mpc->setparam_real_vector("x0", estimState.val);
        State e{estimState-state};
        t = t + dt;
        for(int i = 0; i< state.rows; i++){
            error[i] = error[i] + sqrt(e[i]*e[i]);
        }

    }
    std::cout<<"final error of estim after " << iMPC <<": " << error << "\n";
    std::cout<<"final state of real  after " << iMPC <<": " << state<< "\n";
    std::cout<<"final state of estim after " << iMPC <<": " << estimState << "\n";
    if(_statusError < errorStatus){
        std::cout<<"error status sum: " << _statusError << "\n";
        errorStatus = _statusError;
        SUCCEED();
    } else {
        FAIL();
    }
    //EXPECT_NEAR(desX, estimState[0], 3);
    //EXPECT_NEAR(desY, estimState[1], 7);
    //EXPECT_NEAR(desPhi, estimState[2], 0.3);
    //EXPECT_LT(error[0], 10);
    //EXPECT_LT(error[1], 10);
    //EXPECT_LT(error[2], 0.2);
    //EXPECT_LT(error[3], 31);

}

INSTANTIATE_TEST_CASE_P(MeaningfulTestParameters,
                        ControllerTest,
                        Combine(Range(1,3, 1)              //MaxMultIter
                                , Range(10,60, 10)          //MaxGradIter
                                , Range(0.1,0.6, 0.1)   //Thor
                                , Range(26,101, 99)        //Cost of pos x,y
                                , Range(51,101, 99)        //Cost of angle phi
                                , Range(1,2, 1)        //Cost of velocity
                                , Range(5,40,5)          //Nhor
                                ));
