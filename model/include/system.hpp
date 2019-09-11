//
// Created by sigi on 19.06.19.
//

#ifndef EPIPHANY_SYSTEM_HPP
#define EPIPHANY_SYSTEM_HPP

#include "spline.hpp"
#include "vehicle.hpp"
#include "mpc_model.hpp"
#include "operation_mode.hpp"
#include "grampc.hpp"
#include "grampc_util.h"

namespace epi {
    class System{
    public:

        System(std::shared_ptr<Vehicle> vehicle
                , std::shared_ptr<Model> model
                , std::shared_ptr<OperationModeProvider> operationModeProvider
                );
        ~System();

        void move(double uF, double uPhi);

        void move(const spline::Points& points);


        std::shared_ptr<Vehicle> const vehicle;
        std::shared_ptr<OperationModeProvider> const modeProvider;
    private:

        /**
         * model predictive controller
         */
        //std::unique_ptr<grampc::Grampc> _mpc;

        class SystemImpl;
        std::unique_ptr<SystemImpl> impl;
    };
}
#endif //EPIPHANY_SYSTEM_HPP
