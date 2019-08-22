//
// Created by sigi on 12.06.19.
//

#include "DynamicModel.hpp"
#include "Vehicule.hpp"
#include "MpcFactory.hpp"
#include "ModelConst.hpp"
#include <gtest/gtest.h>
#include <math.h>

using namespace grampc;
using namespace epi;

TEST(random_test, tester){
    auto mpc = createMpc();
    double uF, uPhi;
    Vehicle car{std::make_unique<DynamicCarModel>()};

	ctypeRNum Tsim = 4.0;
	int i;
	typeInt MaxSimIter = (int)(Tsim / con::STEP_SIZE.count());
    typeInt iMPC;
    typeRNum CpuTimeVec[MaxSimIter];
    State xdes{10,0,0,100,0};
    mpc->setparam_real_vector("xdes", xdes.val);
	printf("MPC running ...\n");
	for (iMPC = 0; iMPC <= MaxSimIter; iMPC++) {
		/* run grampc */
		auto tic = clock();
		mpc->run();
		auto toc = clock();
		CpuTimeVec[iMPC] = (typeRNum)((toc - tic) * 1000 / CLOCKS_PER_SEC);

		/* check solver status */
		if (mpc->getSolution()->status > 0) {
			//std::cout<<"Status error: " << mpc->getSolution()->status << "\n";
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
		car.drive(uF, uPhi);
		State state = car.getState();
		std::cout<<"State of car: " << state << "\n";

		State e{mpc->getSolution()->xnext};
		std::cout<<"error of estim: " << e << "\n";
		mpc->setparam_real_vector("x0", state.val);

	}

}
