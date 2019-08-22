//
// Created by sigi on 19.06.19.
//

#ifndef EPIPHANY_SYSTEM_HPP
#define EPIPHANY_SYSTEM_HPP

#include "Spline.hpp"
#include "Vehicule.hpp"
#include "MpcModel.hpp"
#include "OperationMode.hpp"
#include "grampc.hpp"
#include "grampc_util.h"

namespace epi {
    struct System{
        System(std::shared_ptr<Vehicle> vehicle
                , std::shared_ptr<OperationModeProvider> operationModeProvider
                , std::unique_ptr<grampc::Grampc> controller);
        ~System();

        void move(double uF, double uPhi);

        void move(const spline::Points& points);


        std::shared_ptr<Vehicle> const vehicle;
        std::shared_ptr<OperationModeProvider> const modeProvider;
    private:

        /**
         * model predictive controller
         */
        std::unique_ptr<grampc::Grampc> _mpc;

        class SystemImpl;
        std::unique_ptr<SystemImpl> impl;
    };
}
#endif //EPIPHANY_SYSTEM_HPP
