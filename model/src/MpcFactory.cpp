//
// Created by sigi on 19.08.19.
//
#include "MpcFactory.hpp"
#include "MpcModel.hpp"

grampc::Grampc* epi::createMpc(){

    constexpr typeInt NX = MpcModel::X_DIM;
    constexpr typeInt NU = MpcModel::U_DIM;

    typeRNum FINAL_STATE_COST[NX] = {10,10,100,1,1};
    typeRNum STATE_COST[NX] = {10,10,100,1,1};
    typeRNum INPUT_COST[NX] = {1,100};

    grampc::ProblemDescription *model = new MpcModel(FINAL_STATE_COST, STATE_COST, INPUT_COST);
    grampc::Grampc* mpc = new grampc::Grampc(model);

    /********* Parameter definition *********/
    /* Initial values and setpoints of the states, inputs, parameters, penalties and Lagrangian mmultipliers, setpoints for the states and inputs */

    /* Initial values, setpoints and limits of the inputs */
    ctypeRNum u0[NU] = { 0.0, 0.0 };
    ctypeRNum udes[NU] = { 0.0, 0.0 };
    ctypeRNum umax[NU] = { 1.0, 0.5 };
    ctypeRNum umin[NU] = { -1.0, -0.5 };

    /* Time variables */
	ctypeRNum Thor = 3;  /* Prediction horizon */

    ctypeRNum dt = STEP_SIZE.count(); /* Sampling time */
    typeRNum t = 0.0;              /* time at the current sampling step */

    /********* Option param definition *******/
    ctypeRNum ConstraintsAbsTol[1] = {1e-1};

    ctypeInt MaxGradIter = 90;
    ctypeInt MaxMultIter = 3;
    ctypeInt Nhor = 30;

	/********* set parameters *********/
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


    return mpc;
}
