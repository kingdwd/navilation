//
// Created by sigi on 16.08.19.
//

#include <System.hpp>
#include "data.h"
#include <chrono>
#include <queue>
#include <mutex>
#include <thread>
#include "MpcFactory.hpp"


using Clock = std::chrono::high_resolution_clock ;
using Trajectory = std::queue<epi::State>;
struct epi::System::SystemImpl{
    volatile double uF = 0.0;
    volatile double uPhi = 0.0;
    const double tol_xy = 5;
    const double tol_phi = 0.1;
    volatile bool _run = true;
    std::mutex _mutex;
    std::future<void> simulation;
    Trajectory traj;
    constexpr static typeInt NX = MpcModel::X_DIM;
    constexpr static typeInt NU = MpcModel::U_DIM;

    typeRNum FINAL_STATE_COST[NX] = {10,10,100,1,1};
    typeRNum STATE_COST[NX] = {10,10,100,1,1};
    typeRNum INPUT_COST[NX] = {1,100};

    grampc::ProblemDescription *model = new MpcModel(FINAL_STATE_COST, STATE_COST, INPUT_COST);
    grampc::Grampc* _mpc= new grampc::Grampc(model);
    System* sys;
    SystemImpl(System* sys) : sys{sys} {
        simulation = std::async(std::launch::async, &SystemImpl::run, this);
        createMpc(_mpc);
    }
    ~SystemImpl(){
        delete _mpc;
        std::cout<< "System iMPL destroyed \n";
        _run = false;
    }
    void run(){
        while(_run){
            Clock::time_point t1 = Clock::now();
            bool isInAutoMode = sys->modeProvider->state.get() == OperationMode::AUTOMATIC;
            if( isInAutoMode && !traj.empty()){
                std::unique_lock<std::mutex> lck(_mutex);
                State& xdes = traj.front();
                State state = sys->vehicle->getState();
                state.val[2] = std::fmod(state.val[2], M_PI);
                auto error =state-xdes;
                auto l2_xy = sqrt(error.val[0]*error.val[0] + error.val[1]*error.val[1]);
                auto l2_phi = sqrt(error.val[2]*error.val[2]);
                if(l2_xy < tol_xy && l2_phi < tol_phi){
                    traj.pop();
                    xdes = traj.front();
                }
                if(std::isnan(xdes[0]) ||
                   std::isnan(xdes[1]) ||
                   std::isnan(xdes[2]) ||
                   std::isnan(xdes[3]) ||
                   std::isnan(xdes[4]) ){
                    std::cout<<"warning: at least one value for xdes is nan. Replacing by current value \n";
                    //xdes = state;
                }

                _mpc->setparam_real_vector("x0", state.val);
                _mpc->setparam_real_vector("xdes", xdes.val);
                std::cout<<"Demanded x value: "<< xdes <<"]\n";
                std::cout<<"Deviation is : ["<< l2_xy <<","<< l2_phi <<"]\n";
                _mpc->run();
                uF = _mpc->getSolution()->unext[0];
                uPhi = _mpc->getSolution()->unext[1];
                std::cout<<"estimated x: "<<_mpc->getSolution()->xnext << "\n";
                std::cout << "Calculated u: [" << uF << "; " << uPhi << "] \n";

            }
            sys->vehicle->drive(uF, uPhi);

            auto duration = ( Clock::now() - t1 );


            auto cycle = std::chrono::duration_cast<std::chrono::milliseconds>(con::STEP_SIZE - duration);
            std::cout<<"cycle time reserve: " << cycle.count() << "ms\n";
            if(cycle.count() > 0)
                std::this_thread::sleep_for(cycle);
        }
    }
    void appendAsTrajectory(const spline::Points &spline, const double vMax = 150){
        if(spline.empty()) return;
        std::unique_lock<std::mutex> lk(_mutex);
        constexpr auto minX = 1e-6;
        auto splineSize = spline.size();
        auto xLast = spline[splineSize-1].x;
        auto yLast = spline[splineSize-1].y;
        auto v = vMax;
        /* last 10% */
        auto last10 = splineSize*0.85;
        for(int i=0; i<splineSize-1; i++){
            auto x = spline[i].x;
            auto y = spline[i].y;
            auto dx = spline[i+1].x-x;
            auto dy = spline[i+1].y-y;
            double alpha;
            if(dx > minX){
                alpha = atan(dy/dx);
            } else if(dx<-minX){
                alpha = U::Math::sgn(dy)*M_PI + atan(dy/dx);
            } else {
                if(traj.empty()) {
                    continue;
                }
                alpha = traj.back()[2];
            }
            if(i>last10){
                auto dist = sqrt(pow(xLast-x,2) + pow(yLast-y,2));
                if(dist < 2*v){
                    v = sqrt(dist);
                }
            }
            traj.push(
                {(double) spline[i].x
                ,(double) spline[i].y
                , alpha
                , v
                , 0
                });
        }
        /*last element*/
        traj.push({
            (double)spline[splineSize-1].x,
            (double)spline[splineSize-1].y,
            traj.back()[2],
            0,
            0
        });
    }
};

epi::System::System(std::shared_ptr<epi::Vehicle> vehicule,
                    std::shared_ptr<epi::OperationModeProvider> operationModeProvider
                    )
    : vehicle{vehicule}
    , modeProvider{operationModeProvider}
    , impl{std::make_unique<SystemImpl>(this)}
    {}

void epi::System::move(double uF, double uPhi) {
    if(modeProvider->state.get() == OperationMode::MANUAL){
        impl->uF = uF;
        impl->uPhi = uPhi;
        std::cout<<"got move command : [" <<uF <<";" <<uPhi << "] \n";
    }
}

void epi::System::move(const epi::spline::Points& points) {
    impl->appendAsTrajectory(points);
}


epi::System::~System() = default;
