//
// Created by sigi on 15.08.19.
//

#include <OdeSolver.hpp>

#include "OdeSolver.hpp"
namespace epi {
    OdeSolver::OdeSolver(const std::function<State(const State&, double, double)> fx, const std::chrono::duration<long double>& stepSize)
    : _fx{fx}
    , _stepSize(stepSize.count())
    {}

    State RungeKutta::next(const epi::State &x, double u_F, double u_phi) const {
        State k1 = _fx(x, u_F, u_phi);
        State k2 = _fx(x + _stepSize/2*k1, u_F, u_phi);
        State k3 = _fx(x + _stepSize/2*k2, u_F, u_phi);
        State k4 = _fx(x + _stepSize*k3, u_F, u_phi);

        return x + _stepSize*((k1+k4)/2 +(k2+k3))/3;
    }

    RungeKutta::RungeKutta(const std::function<State(const State&, double, double)> fx)
    : OdeSolver(fx)
    {}

    RungeMerson::RungeMerson(const std::function<State(const State&, double, double)> fx)
    : OdeSolver(fx)
    {}

    State RungeMerson::next(const State &x, double u_F, double u_phi) const {
        State k1 = _fx(x, u_F, u_phi);
        State k2 = _fx(x + _stepSize/3*k1, u_F, u_phi);
        State k3 = _fx(x + _stepSize/6*(k1+k2), u_F, u_phi);
        State k4 = _fx(x + _stepSize/8*(k1+3*k2), u_F, u_phi);
        State k5 = _fx(x +0.5*_stepSize*(k1-3*k3+4*k4), u_F, u_phi);

        return x + _stepSize*((k1 + 4*k4 + k5)/6 + (-2*k1 + 9*k3 - 8*k4 + k5)/30);
    }
}
