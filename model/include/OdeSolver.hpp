//
// Created by sigi on 19.06.19.
//

#ifndef EPIPHANY_DIFFEQUATIONSOLVER_HPP
#define EPIPHANY_DIFFEQUATIONSOLVER_HPP

#include "data.h"
#include "ModelConst.hpp"

namespace epi {
    class OdeSolver {
    public:
        OdeSolver(const std::function<State(const State&, double, double)> fx, const std::chrono::duration<long double>& stepSize = con::STEP_SIZE);
        virtual ~OdeSolver() = default;

        virtual State next(const State& x, double u_F, double u_phi) const = 0;

    protected:
        std::function<State(State, double, double)> _fx;
        const double _stepSize;
    };



    struct RungeKutta : public OdeSolver{
        explicit RungeKutta(const std::function<State(const State&, double, double)> fx);
        State next(const State& x, double u_F, double u_phi) const override;
    };

    struct RungeMerson : public OdeSolver{
        explicit RungeMerson(const std::function<State(const State&, double, double)> fx);
        State next(const State& x, double u_F, double u_phi) const override;
    };
}
#endif //EPIPHANY_DIFFEQUATIONSOLVER_HPP
