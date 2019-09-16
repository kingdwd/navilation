//
// Created by sigi on 15.08.19.
//

#ifndef EPIPHANY_VEHICULE_HPP
#define EPIPHANY_VEHICULE_HPP

#include "dynamic_model.hpp"
#include "ode_solver.hpp"

namespace epi {
    class Vehicle {
    public:
        explicit Vehicle(std::unique_ptr<dynamic_model> model
                , const Pose& pose = Pose{0, 0, 0}
                , const Shape& shape = Shape{100, 20}
                , const std::string &type = "car");

        const Shape shape;
        U::Var<Pose> pose;
        const std::string type;

        void drive(double longitudinal, double lateral);
        State getState() const;

    private:
        State state;
        const std::unique_ptr<dynamic_model> _dynamicModel;
        const std::unique_ptr<OdeSolver> _solver
            = std::unique_ptr<RungeKutta>(new RungeKutta([this](const State& x, double uF, double uPhi){return _dynamicModel->dx(x, uF, uPhi);}));
    };

}
#endif //EPIPHANY_VEHICULE_HPP
