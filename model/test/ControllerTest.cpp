//
// Created by sigi on 12.06.19.
//

#include "DynamicModel.hpp"
#include "Vehicule.hpp"
#include "MpcFactory.hpp"
#include "ModelConst.hpp"
#include <gtest/gtest.h>
#include <math.h>
#include <MpcModel.hpp>

using namespace grampc;
using namespace epi;

TEST(random_test, tester){
    //auto& mpc = *mpcP;

    constexpr typeInt NX = MpcModel::X_DIM;
    constexpr typeInt NU = MpcModel::U_DIM;

    typeRNum FINAL_STATE_COST[NX] = {10,10,100,1,100};
    typeRNum STATE_COST[NX] = {10,10,100,1,100};
    typeRNum INPUT_COST[NX] = {1,10};

    grampc::ProblemDescription *model = new MpcModel(FINAL_STATE_COST, STATE_COST, INPUT_COST);
    grampc::Grampc* mpc = new grampc::Grampc(model);
    createMpc(mpc);

    ///********* Parameter definition *********/
    ///* Initial values and setpoints of the states, inputs, parameters, penalties and Lagrangian mmultipliers, setpoints for the states and inputs */
    //ctypeRNum x0[NX] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

    ctypeRNum xdes[NX] = { 15, -30.0, -1.5, 0.0, 0.0 };
    ///* Initial values, setpoints and limits of the inputs */
    //ctypeRNum u0[NU] = { 0.0, 0.0 };
    //ctypeRNum udes[NU] = { 0.0, 0.0 };
    ctypeRNum umax[NU] = { 1.0, 0.5 };
    ctypeRNum umin[NU] = { -1.0, -0.5 };

    ///* Time variables */
    ctypeRNum Thor = 0.5;  /* Prediction horizon */

    //ctypeRNum dt = epi::STEP_SIZE.count(); /* Sampling time */
    //typeRNum t = 0.0;              /* time at the current sampling step */

    ///********* Option param definition *******/
    ctypeRNum ConstraintsAbsTol[5] = {5,5,0.1,100,100};

    ctypeInt MaxGradIter = 60;
    ctypeInt MaxMultIter = 1;
    ctypeInt Nhor = 20;

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

    mpc->setopt_real_vector("ConstraintsAbsTol", ConstraintsAbsTol);
    double uF, uPhi;
    constexpr double stepSize = 0.01;
    Vehicle car(std::make_unique<epi::DynamicCarModel>());

    ctypeRNum Tsim = 4.0;
    int i;
    typeInt MaxSimIter = (int)(Tsim / 0.01);
    typeInt iMPC;
    typeRNum CpuTimeVec[MaxSimIter];
    //epi::State xdes{ 115, -300.0, -2.0, 0.0, 0.0 };
    //mpc.setparam_real_vector("xdes", xdes.val);
    //printf("MPC running ...\n");
    //State state = car.getState();
    //mpc.setparam_real_vector("x0", state.val);
    for (iMPC = 0; iMPC <= MaxSimIter; iMPC++) {
        /* run grampc */
        auto tic = clock();
        mpc->run();
        auto toc = clock();
        CpuTimeVec[iMPC] = (typeRNum)((toc - tic) * 1000 / CLOCKS_PER_SEC);

        /* check solver status */
        if (mpc->getSolution()->status > 0) {
            std::cout<<"Status error: " << mpc->getSolution()->status << std::endl;
            mpc->printstatus(mpc->getSolution()->status, STATUS_LEVEL_WARN);
        }

        ///* reference integration of the system via heun scheme since grampc->sol->xnext is only an interpolated value */
        //ffct(rwsReferenceIntegration, t, grampc->param->x0, grampc->sol->unext, grampc->sol->pnext, grampc->userparam);
        //for (i = 0; i < NX; i++) {
        //	grampc->sol->xnext[i] = grampc->param->x0[i] + dt * rwsReferenceIntegration[i];
        //}
        //ffct(rwsReferenceIntegration + NX, t + dt, grampc->sol->xnext, grampc->sol->unext, grampc->sol->pnext, grampc->userparam);
        //for (i = 0; i < NX; i++) {
        //	grampc->sol->xnext[i] = grampc->param->x0[i] + dt * (rwsReferenceIntegration[i] + rwsReferenceIntegration[i + NX]) / 2;
        //}
        uF = mpc->getSolution()->xnext[0];
        uPhi = mpc->getSolution()->unext[1];
        //std::cout<<"State of car: " << state << "\n";
        State e{mpc->getSolution()->xnext};
        std::cout<<"Stellwert u: [" << uF <<"; "<< uPhi << "]\n";
        std::cout<<"State of estim: " << e << "\n";
        car.drive(uF, uPhi);
        State state = car.getState();
        mpc->setparam_real_vector("x0", state.val);
        std::cout<<"State of state: " << state << "\n";
        std::cout<<"Error of estim: " << e - state << "\n";


    }

}
