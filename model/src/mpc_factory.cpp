//
// Created by sigi on 19.08.19.
//
#include "mpc_factory.hpp"
#include "mpc_model.hpp"

void epi::createMpc(grampc::Grampc* mpc){

    constexpr typeInt NX = MpcModel::X_DIM;
    constexpr typeInt NU = MpcModel::U_DIM;
//
//    typeRNum FINAL_STATE_COST[NX] = {10,10,100,1,1};
//    typeRNum STATE_COST[NX] = {10,10,100,1,1};
//    typeRNum INPUT_COST[NX] = {1,100};
//
//    grampc::ProblemDescription *model = new mpc_model(FINAL_STATE_COST, STATE_COST, INPUT_COST);
//    auto mpc = std::make_unique<grampc::Grampc>(model);

    /********* Parameter definition *********/
    /* Initial values and setpoints of the states, inputs, parameters, penalties and Lagrangian mmultipliers, setpoints for the states and inputs */
    //ctypeRNum x0[NX] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

    //ctypeRNum xdes[NX] = { 5, -300.0, -2.0, 0.0, 0.0 };
    /* Initial values, setpoints and limits of the inputs */
    ctypeRNum u0[NU] = { 0.0, 0.0 };
    ctypeRNum udes[NU] = { 0.0, 0.0 };
    ctypeRNum umax[NU] = { 1.0, 0.5 };
    ctypeRNum umin[NU] = { -1.0, -0.5 };

    /* Time variables */
	ctypeRNum Thor = 0.05;  /* Prediction horizon */

    ctypeRNum dt = 0.01; /* Sampling time */
    typeRNum t = 0.0;              /* time at the current sampling step */

    /********* Option param definition *******/
    ctypeRNum ConstraintsAbsTol[1] = {1e-1};

    ctypeInt MaxGradIter = 30;
    ctypeInt MaxMultIter = 3;
    ctypeInt Nhor = 20;

	/********* set parameters *********/
    //mpc->setparam_real_vector("x0", x0);
    //mpc->setparam_real_vector("xdes", xdes);
	mpc->setparam_real_vector("u0", u0);
	mpc->setparam_real_vector("udes", udes);
	mpc->setparam_real_vector("umax", umax);
	mpc->setparam_real_vector("umin", umin);

	mpc->setparam_real("Thor", Thor);

	mpc->setparam_real("dt", dt);
	mpc->setparam_real("t0", t);

    /********* set options *********/
    mpc->setopt_int("Nhor", Nhor);
    mpc->setopt_int("MaxGradIter", MaxGradIter);
    mpc->setopt_int("MaxMultIter", MaxMultIter);

    mpc->setopt_real_vector("ConstraintsAbsTol", ConstraintsAbsTol);


    //return std::move(mpc);
}
